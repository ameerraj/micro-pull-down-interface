import tkinter as tk
from tkinter import ttk, messagebox
import pyvisa
import serial
import serial.tools.list_ports
import threading
import time
import csv
import math
from datetime import datetime
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ============================================================
# HARDCODED SAFETY LIMITS & CONFIG 
# ============================================================
HARDWARE_OVP = 10.5         
PSU_BAUD_RATE = 115200

# Default Rates (Seconds)
DEFAULT_DAQ_INTERVAL = 0.5
DEFAULT_PSU_INTERVAL = 0.5
DEFAULT_LOG_INTERVAL = 1.0

# ============================================================
# DAQ6510 CONSTANTS & CALIBRATION
# ============================================================
CHANNELS = [101, 102, 103, 104, 105]
CHANNEL_LIST = "(@101:105)"
NPLC = 1.0
VISA_TIMEOUT_MS = 5000

TEMPERATURE_CALIBRATION = {
    101: {"gain": 84.33, "offset": 10.78},
    102: {"gain": 84.33, "offset": 10.78},
    103: {"gain": 84.33, "offset": 10.78},
    104: {"gain": 84.33, "offset": 10.78},
    105: {"gain": 84.33, "offset": 10.78}, # crucible thermocouple
}

class ThreadedHardwareApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Decoupled Hardware Controller (Ready for PID)")
        self.root.geometry("1200x820")
        
        # ========================================================
        # 1. THREADING & SYNCHRONIZATION TOOLS
        # ========================================================
        self.state_lock = threading.Lock()
        self.stop_event = threading.Event()
        
        # Hardware interfaces
        self.rm = pyvisa.ResourceManager()
        self.psu = None
        self.daq = None
        self.psu_protocol = "SCPI"
        
        # ========================================================
        # 2. CENTRAL SHARED STATE (The "Bulletin Board")
        # ========================================================
        self.state = {
            # Connections
            "psu_connected": False,
            "daq_connected": False,
            
            # --- CONTROL TO PSU (Targets) ---
            "target_psu_v": 0.0,
            "target_psu_i": 0.0,
            "force_heater_off": False,
            
            # --- PSU TO SYSTEM (Telemetry) ---
            "actual_psu_v": 0.0,
            "actual_psu_i": 0.0,
            "psu_timestamp": 0.0,
            
            # --- DAQ TO SYSTEM (Sensors) ---
            "actual_temps": {ch: 0.0 for ch in CHANNELS},
            "actual_voltages": {ch: 0.0 for ch in CHANNELS},
            "daq_timestamp": 0.0,
            
            # --- LOGGING & CONTROL STATE ---
            "test_running": False,
            "daq_interval_s": DEFAULT_DAQ_INTERVAL,
            "psu_interval_s": DEFAULT_PSU_INTERVAL,
            "log_interval_s": DEFAULT_LOG_INTERVAL,
            "test_start_time": 0.0,
            
            # Internal Ramping logic state (to be replaced/augmented by PID)
            "ramp_active": False,
            "ramp_target_i": 0.0,
            "ramp_rate_A_sec": 0.0,
        }

        # UI & Plotting Variables
        self.log_history = []
        self.available_psu_com = []
        self.available_usb_visa = []
        
        self.t_data = []
        self.v_data = []
        self.i_data = []
        self.temp_data = {ch: [] for ch in CHANNELS}
        self.temp_channel_visible = {ch: tk.BooleanVar(value=True) for ch in CHANNELS}

        # Setup
        self.scan_ports()
        self.setup_ui()
        
        # ========================================================
        # 3. START BACKGROUND THREADS
        # ========================================================
        threading.Thread(target=self._psu_thread_loop, daemon=True).start()
        threading.Thread(target=self._daq_thread_loop, daemon=True).start()
        threading.Thread(target=self._control_thread_loop, daemon=True).start()
        threading.Thread(target=self._logger_thread_loop, daemon=True).start()
        
        # Start UI Refresh tick (runs in main thread)
        self.root.after(200, self._ui_refresh_tick)

    # ============================================================
    # BACKGROUND THREAD: TDK-LAMBDA PSU ACTUATOR
    # ============================================================
    def _psu_thread_loop(self):
        """Monitors targets and updates PSU hardware. Never holds lock during I/O."""
        last_applied_v = -1.0
        last_applied_i = -1.0
        was_forced_off = False
        last_telemetry_time = 0.0
        
        while not self.stop_event.is_set():
            if not self.psu or not self.state["psu_connected"]:
                time.sleep(0.2)
                continue
                
            # 1. READ TARGETS & SETTINGS FROM SHARED STATE
            with self.state_lock:
                target_v = self.state["target_psu_v"]
                target_i = self.state["target_psu_i"]
                force_off = self.state["force_heater_off"]
                psu_interval = self.state["psu_interval_s"]
                
            # 2. APPLY HARDWARE COMMANDS (OUTSIDE LOCK)
            try:
                if force_off and not was_forced_off:
                    if self.psu_protocol == "SCPI":
                        self._psu_write(self.psu, "VOLT 0.0")
                        self._psu_write(self.psu, "CURR 0.0")
                        self._psu_write(self.psu, "OUTP OFF")
                    else:
                        self._psu_write(self.psu, "PV 0.0")
                        self._psu_write(self.psu, "PC 0.0")
                        self._psu_write(self.psu, "OUT 0")
                    was_forced_off = True
                    last_applied_v = 0.0
                    last_applied_i = 0.0
                    
                elif not force_off:
                    was_forced_off = False
                    
                    if target_v != last_applied_v:
                        cmd = f"VOLT {target_v:.3f}" if self.psu_protocol == "SCPI" else f"PV {target_v:.3f}"
                        self._psu_write(self.psu, cmd)
                        last_applied_v = target_v
                        
                    if target_i != last_applied_i:
                        cmd = f"CURR {target_i:.3f}" if self.psu_protocol == "SCPI" else f"PC {target_i:.3f}"
                        self._psu_write(self.psu, cmd)
                        last_applied_i = target_i

                # 3. READ TELEMETRY ONLY IF THE PSU INTERVAL HAS PASSED
                current_time = time.perf_counter()
                if current_time - last_telemetry_time >= psu_interval:
                    if self.psu_protocol == "SCPI":
                        meas_v = float(self._psu_query(self.psu, "MEAS:VOLT?", wait_s=0.05))
                        meas_i = float(self._psu_query(self.psu, "MEAS:CURR?", wait_s=0.05))
                    else:
                        meas_v = float(self._psu_query(self.psu, "MV?", wait_s=0.05))
                        meas_i = float(self._psu_query(self.psu, "MC?", wait_s=0.05))
                        
                    # 4. WRITE TELEMETRY TO SHARED STATE
                    with self.state_lock:
                        self.state["actual_psu_v"] = meas_v
                        self.state["actual_psu_i"] = meas_i
                        self.state["psu_timestamp"] = time.perf_counter()
                        
                    last_telemetry_time = current_time
                    
            except Exception as e:
                self.log(f"PSU Loop Error: {e}")
                
            # Thread loops quickly (20Hz) so Ramping and Safe Off remain responsive
            time.sleep(0.05) 

    # ============================================================
    # BACKGROUND THREAD: KEITHLEY DAQ SENSOR
    # ============================================================
    def _daq_thread_loop(self):
        """Continuously pulls fresh temperatures from Keithley at its specific interval."""
        while not self.stop_event.is_set():
            # Check if active
            with self.state_lock:
                active = self.state["daq_connected"] and self.state["test_running"]
                
            if not self.daq or not active:
                time.sleep(0.2)
                continue
                
            try:
                # 1. DO I/O OUTSIDE LOCK
                daq_voltages, _ = self.read_all_channels()
                daq_temps = {ch: self.voltage_to_temperature(ch, v) for ch, v in zip(CHANNELS, daq_voltages)}
                daq_volts_dict = {ch: v for ch, v in zip(CHANNELS, daq_voltages)}
                
                # 2. QUICKLY UPDATE STATE
                with self.state_lock:
                    self.state["actual_temps"] = daq_temps
                    self.state["actual_voltages"] = daq_volts_dict
                    self.state["daq_timestamp"] = time.perf_counter()
                    
            except Exception as e:
                self.log(f"DAQ Loop Error: {e}")
                
            # 3. SLEEP DYNAMICALLY BASED ON DAQ INTERVAL
            with self.state_lock:
                daq_interval = self.state["daq_interval_s"]
                
            self.stop_event.wait(daq_interval)

    # ============================================================
    # BACKGROUND THREAD: CONTROL LOGIC (Ramp & Future PID)
    # ============================================================
    def _control_thread_loop(self):
        """Mathematical control thread. Highly deterministic timing."""
        last_time = time.perf_counter()
        
        while not self.stop_event.is_set():
            current_time = time.perf_counter()
            dt = current_time - last_time
            last_time = current_time
            
            with self.state_lock:
                # If Safe Off is triggered, cancel any ramps/logic
                if self.state["force_heater_off"]:
                    self.state["ramp_active"] = False
                    self.state["target_psu_i"] = 0.0
                    
                # Current Ramping Logic
                elif self.state["ramp_active"]:
                    current_setpoint = self.state["target_psu_i"]
                    target = self.state["ramp_target_i"]
                    rate = self.state["ramp_rate_A_sec"]
                    delta = rate * dt
                    
                    if current_setpoint < target:
                        new_setpoint = min(current_setpoint + delta, target)
                    else:
                        new_setpoint = max(current_setpoint - delta, target)
                        
                    self.state["target_psu_i"] = new_setpoint
                    
                    if math.isclose(new_setpoint, target, abs_tol=0.001):
                        self.state["ramp_active"] = False
                        self.log(f"Ramp complete. Holding at {target} A.")
                        
                # === FUTURE PID LOGIC GOES HERE ===
                # elif self.state["pid_active"]:
                #     error = self.state["target_temp_c"] - self.state["actual_temps"][105]
                #     self.state["target_psu_i"] = self.calculate_pid(error, dt)
            
            # Sleep exactly enough to maintain a 100ms loop
            elapsed = time.perf_counter() - current_time
            time.sleep(max(0, 0.1 - elapsed))

    # ============================================================
    # BACKGROUND THREAD: DATA LOGGER
    # ============================================================
    def _logger_thread_loop(self):
        """Guarantees CSV writing at exact log intervals, immune to hardware lag."""
        loop_iteration = 0
        
        while not self.stop_event.is_set():
            with self.state_lock:
                running = self.state["test_running"]
                start_t = self.state["test_start_time"]
                log_interval = self.state["log_interval_s"]
                
            if not running or not getattr(self, 'csv_file', None):
                loop_iteration = 0
                time.sleep(0.1)
                continue
                
            # Snapshot state for logging
            with self.state_lock:
                snapshot = {
                    "v": self.state["actual_psu_v"],
                    "i": self.state["actual_psu_i"],
                    "temps": dict(self.state["actual_temps"])
                }
                
            elapsed = time.time() - start_t
            
            # Write CSV
            if self.csv_writer:
                row = [datetime.now().isoformat(), round(elapsed, 3), snapshot["v"], snapshot["i"]]
                for ch in CHANNELS:
                    row.append(snapshot["temps"][ch])
                try:
                    self.csv_writer.writerow(row)
                    self.csv_file.flush()
                except Exception:
                    pass
                    
            # Append to memory for plot
            self.t_data.append(elapsed)
            self.v_data.append(snapshot["v"])
            self.i_data.append(snapshot["i"])
            for ch in CHANNELS:
                self.temp_data[ch].append(snapshot["temps"][ch])
                
            # Exact interval sleep logic
            loop_iteration += 1
            next_target_time = start_t + (loop_iteration * log_interval)
            sleep_time = next_target_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)

    # ============================================================
    # UI THREAD: MAIN REFRESH TICK
    # ============================================================
    def _ui_refresh_tick(self):
        """Reads shared state and updates all Tkinter labels and graphs."""
        if self.stop_event.is_set():
            return
            
        with self.state_lock:
            v = self.state["actual_psu_v"]
            i = self.state["actual_psu_i"]
            temps = dict(self.state["actual_temps"])
            ramp_active = self.state["ramp_active"]
            test_running = self.state["test_running"]

        # Update Labels
        self.lbl_v_meas.config(text=f"Meas V: {v:.3f} V")
        self.lbl_i_meas.config(text=f"Meas I: {i:.3f} A")
        
        temp_str = " | ".join([f"CH{ch}: {temps[ch]:.1f}°C" for ch in CHANNELS])
        self.lbl_daq_temps.config(text=temp_str)
        
        # Manage UI button states based on variables
        if not ramp_active and str(self.btn_stop_ramp['state']) == tk.NORMAL:
            self.btn_stop_ramp.config(state=tk.DISABLED)

        # Redraw plots if testing
        if test_running and self.t_data:
            self.update_plots()

        self.root.after(200, self._ui_refresh_tick)

    # ============================================================
    # HARDWARE COMMUNICATION HELPERS (Called by background threads)
    # ============================================================
    def _psu_write(self, ser_inst, cmd):
        ser_inst.write(f"{cmd}\r\n".encode("ascii"))
        ser_inst.flush()

    def _psu_query(self, ser_inst, cmd, wait_s=0.08):
        ser_inst.reset_input_buffer()
        self._psu_write(ser_inst, cmd)
        time.sleep(wait_s)
        raw = ser_inst.read_until(b"\r")
        if ser_inst.in_waiting > 0:
            raw += ser_inst.read(ser_inst.in_waiting)
        return raw.decode("ascii", errors="replace").strip()
        
    def read_all_channels(self):
        self.daq.write(':TRAC:CLEAR "defbuffer1"')
        self.daq.write(":INIT")
        self.daq.query("*OPC?")
        raw_data = self.daq.query_ascii_values(f':TRAC:DATA? 1, {len(CHANNELS)}, "defbuffer1", READ, REL')
        readings = [float(raw_data[i]) for i in range(0, len(raw_data), 2)]
        timestamps = [float(raw_data[i+1]) for i in range(0, len(raw_data), 2)]
        return readings, timestamps

    def voltage_to_temperature(self, channel, voltage):
        cal = TEMPERATURE_CALIBRATION.get(channel)
        if not cal: return 0.0
        return (voltage * 1000) * cal["gain"] + cal["offset"]

    # ============================================================
    # UI EVENT HANDLERS (Write to Shared State)
    # ============================================================
    def connect_psu_thread(self):
        port = self.combo_psu_port.get().strip()
        if not port: return
        self.btn_psu_conn.config(state=tk.DISABLED)
        threading.Thread(target=self._connect_psu, args=(port,), daemon=True).start()

    def _connect_psu(self, port):
        try:
            temp_ser = serial.Serial(port=port, baudrate=PSU_BAUD_RATE, timeout=0.6, write_timeout=1.0)
            temp_ser.dtr = True; temp_ser.rts = True
            time.sleep(0.15)
            
            self._psu_write(temp_ser, "INST:NSEL 6")
            time.sleep(0.1)
            temp_ser.reset_input_buffer()
            idn = self._psu_query(temp_ser, "*IDN?", wait_s=0.15)
            
            self._psu_write(temp_ser, "SYST:SET REM")
            self._psu_write(temp_ser, f"VOLT:PROT:LEV {HARDWARE_OVP}")
            self._psu_write(temp_ser, "VOLT 0.0")
            self._psu_write(temp_ser, "CURR 0.0")
            self._psu_write(temp_ser, "OUTP ON")
            
            self.psu = temp_ser
            with self.state_lock:
                self.state["psu_connected"] = True
                
            self.log(f"PSU Connected: {idn}")
            self.root.after(0, lambda: [
                self.btn_psu_conn.config(text="PSU Connected", state=tk.DISABLED),
                self.btn_apply.config(state=tk.NORMAL),
                self.btn_safe_off.config(state=tk.NORMAL),
                self.check_enable_test()
            ])
        except Exception as e:
            self.log(f"PSU Connection failed: {e}")
            self.root.after(0, lambda: self.btn_psu_conn.config(state=tk.NORMAL))

    def connect_daq_thread(self):
        port = self.combo_daq_port.get().strip()
        if not port: return
        self.btn_daq_conn.config(state=tk.DISABLED)
        threading.Thread(target=self._connect_daq, args=(port,), daemon=True).start()

    def _connect_daq(self, port):
        try:
            temp_daq = self.rm.open_resource(port)
            temp_daq.timeout = VISA_TIMEOUT_MS
            temp_daq.write_termination = "\n"; temp_daq.read_termination = "\n"
            
            idn = temp_daq.query("*IDN?").strip()
            temp_daq.write(":ABOR"); temp_daq.write("*RST"); temp_daq.query("*OPC?")
            temp_daq.write(f':SENS:FUNC "VOLT:DC", {CHANNEL_LIST}')
            temp_daq.write(f":SENS:VOLT:DC:RANG:AUTO ON, {CHANNEL_LIST}")
            temp_daq.write(f":SENS:VOLT:DC:NPLC {NPLC}, {CHANNEL_LIST}")
            temp_daq.write(f":ROUT:SCAN:CRE {CHANNEL_LIST}")
            temp_daq.write(":ROUT:SCAN:COUNT:SCAN 1")
            temp_daq.query("*OPC?")
            
            self.daq = temp_daq
            with self.state_lock:
                self.state["daq_connected"] = True
                
            self.log(f"DAQ Connected: {idn}")
            self.root.after(0, lambda: [
                self.btn_daq_conn.config(text="DAQ Connected", state=tk.DISABLED),
                self.check_enable_test()
            ])
        except Exception as e:
            self.log(f"DAQ Connect Error: {e}")
            self.root.after(0, lambda: self.btn_daq_conn.config(state=tk.NORMAL))

    def update_system_rates(self):
        try:
            daq = float(self.entry_daq_interval.get())
            psu = float(self.entry_psu_interval.get())
            log = float(self.entry_log_interval.get())
            
            if daq <= 0 or psu <= 0 or log <= 0:
                raise ValueError("Rates must be > 0")
                
            with self.state_lock:
                self.state["daq_interval_s"] = daq
                self.state["psu_interval_s"] = psu
                self.state["log_interval_s"] = log
                
            self.log(f"Rates Updated -> DAQ: {daq}s | PSU: {psu}s | Log: {log}s")
        except Exception as e:
            self.log("Error updating rates. Ensure inputs are positive numbers.")

    def start_current_ramp(self):
        try:
            target_v = float(self.entry_target_v.get())
            target_i = float(self.entry_target_i.get())
            rate_min = float(self.entry_ramp_rate.get())
            
            with self.state_lock:
                self.state["target_psu_v"] = target_v
                self.state["ramp_target_i"] = target_i
                self.state["ramp_rate_A_sec"] = rate_min / 60.0
                self.state["force_heater_off"] = False
                self.state["ramp_active"] = True
                
            self.btn_stop_ramp.config(state=tk.NORMAL)
            self.log(f"Ramp triggered: -> {target_i}A at {rate_min} A/min.")
        except Exception as e:
            self.log(f"Input error: {e}")

    def stop_current_ramp(self):
        with self.state_lock:
            self.state["ramp_active"] = False
        self.log("Ramp manually stopped.")

    def safe_heater_off(self):
        with self.state_lock:
            self.state["ramp_active"] = False
            self.state["force_heater_off"] = True
            self.state["target_psu_i"] = 0.0
            self.state["target_psu_v"] = 0.0
        self.btn_stop_ramp.config(state=tk.DISABLED)
        self.log("🛑 SAFE OFF TRIGGERED. Heater disabled via hardware.")

    def start_test(self):
        # Force a read of the rates before starting
        self.update_system_rates()
        
        try:
            filename = f"daq_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
            self.csv_file = open(filename, mode='w', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            headers = ["Timestamp", "Elapsed_s", "PSU_V", "PSU_A"]
            headers.extend([f"CH{ch}_Temp_C" for ch in CHANNELS])
            self.csv_writer.writerow(headers)
            
            self.t_data.clear(); self.v_data.clear(); self.i_data.clear()
            for ch in CHANNELS: self.temp_data[ch].clear()
                
            with self.state_lock:
                self.state["test_start_time"] = time.time()
                self.state["test_running"] = True
                
            self.btn_start_test.config(state=tk.DISABLED)
            self.btn_stop_test.config(state=tk.NORMAL)
            self.log(f"Logging started: {filename}")
        except Exception as e:
            self.log(f"Test start error: {e}")

    def stop_test(self):
        with self.state_lock:
            self.state["test_running"] = False
        if getattr(self, 'csv_file', None):
            self.csv_file.close()
        self.btn_start_test.config(state=tk.NORMAL)
        self.btn_stop_test.config(state=tk.DISABLED)
        self.log("Logging stopped.")

    # ============================================================
    # UI CONSTRUCTION & PLOTTING BOILERPLATE
    # ============================================================
    def scan_ports(self):
        try:
            self.available_usb_visa = [r for r in self.rm.list_resources() if "USB" in r or "TCPIP" in r]
        except: pass
        try:
            ports = [p.device for p in serial.tools.list_ports.comports()]
            self.available_psu_com = ports if ports else ["COM4"]
        except:
            self.available_psu_com = ["COM4"]

    def setup_ui(self):
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # === SYSTEM RATES & LOGGING FRAME ===
        rates_frame = ttk.LabelFrame(main_frame, text="System Rates & Logging", padding=8)
        rates_frame.pack(fill=tk.X, pady=(0, 8))
        
        # Row 0: Intervals
        ttk.Label(rates_frame, text="DAQ Poll (s):").grid(row=0, column=0, padx=5, sticky="e")
        self.entry_daq_interval = ttk.Entry(rates_frame, width=8)
        self.entry_daq_interval.insert(0, str(DEFAULT_DAQ_INTERVAL))
        self.entry_daq_interval.grid(row=0, column=1, padx=5, sticky="w")
        
        ttk.Label(rates_frame, text="PSU Poll (s):").grid(row=0, column=2, padx=5, sticky="e")
        self.entry_psu_interval = ttk.Entry(rates_frame, width=8)
        self.entry_psu_interval.insert(0, str(DEFAULT_PSU_INTERVAL))
        self.entry_psu_interval.grid(row=0, column=3, padx=5, sticky="w")
        
        ttk.Label(rates_frame, text="Log Rate (s):").grid(row=0, column=4, padx=5, sticky="e")
        self.entry_log_interval = ttk.Entry(rates_frame, width=8)
        self.entry_log_interval.insert(0, str(DEFAULT_LOG_INTERVAL))
        self.entry_log_interval.grid(row=0, column=5, padx=5, sticky="w")

        self.btn_apply_rates = ttk.Button(rates_frame, text="Update Rates", command=self.update_system_rates)
        self.btn_apply_rates.grid(row=0, column=6, padx=15)
        
        # Row 1: Start/Stop
        self.btn_start_test = ttk.Button(rates_frame, text="Start Data Log", command=self.start_test, state=tk.DISABLED)
        self.btn_start_test.grid(row=1, column=0, columnspan=2, pady=8, sticky="ew", padx=5)
        self.btn_stop_test = ttk.Button(rates_frame, text="Stop Data Log", command=self.stop_test, state=tk.DISABLED)
        self.btn_stop_test.grid(row=1, column=2, columnspan=2, pady=8, sticky="ew", padx=5)

        # === CONTROL FRAME ===
        control_frame = ttk.Frame(main_frame)
        control_frame.pack(fill=tk.X, pady=(0, 10))

        psu_frame = ttk.LabelFrame(control_frame, text="TDK-Lambda Power Supply", padding=10)
        psu_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        self.combo_psu_port = ttk.Combobox(psu_frame, values=self.available_psu_com, width=15)
        self.combo_psu_port.set(self.available_psu_com[0] if self.available_psu_com else "")
        self.combo_psu_port.grid(row=0, column=0, padx=2, pady=2)
        self.btn_psu_conn = ttk.Button(psu_frame, text="Connect PSU", command=self.connect_psu_thread)
        self.btn_psu_conn.grid(row=0, column=1, padx=2, pady=2)
        
        ttk.Label(psu_frame, text="Target Volts:").grid(row=1, column=0, sticky="e")
        self.entry_target_v = ttk.Entry(psu_frame, width=8); self.entry_target_v.grid(row=1, column=1, sticky="w")
        self.entry_target_v.insert(0, "0.0")
        ttk.Label(psu_frame, text="Target Amps:").grid(row=2, column=0, sticky="e")
        self.entry_target_i = ttk.Entry(psu_frame, width=8); self.entry_target_i.grid(row=2, column=1, sticky="w")
        self.entry_target_i.insert(0, "0.0")
        ttk.Label(psu_frame, text="Ramp (A/min):").grid(row=3, column=0, sticky="e")
        self.entry_ramp_rate = ttk.Entry(psu_frame, width=8); self.entry_ramp_rate.grid(row=3, column=1, sticky="w")
        self.entry_ramp_rate.insert(0, "0.5")

        self.btn_apply = ttk.Button(psu_frame, text="Start Ramp", command=self.start_current_ramp, state=tk.DISABLED)
        self.btn_apply.grid(row=4, column=0, pady=5)
        self.btn_stop_ramp = ttk.Button(psu_frame, text="Stop Ramp", command=self.stop_current_ramp, state=tk.DISABLED)
        self.btn_stop_ramp.grid(row=4, column=1, pady=5)
        self.btn_safe_off = ttk.Button(psu_frame, text="🛑 SAFE OFF", command=self.safe_heater_off, state=tk.DISABLED)
        self.btn_safe_off.grid(row=5, column=0, columnspan=2, pady=5, sticky="ew")

        self.lbl_v_meas = ttk.Label(psu_frame, text="Meas V: -- V", font=("Arial", 11, "bold"), foreground="green")
        self.lbl_v_meas.grid(row=0, column=2, rowspan=2, padx=15)
        self.lbl_i_meas = ttk.Label(psu_frame, text="Meas I: -- A", font=("Arial", 11, "bold"), foreground="green")
        self.lbl_i_meas.grid(row=2, column=2, rowspan=2, padx=15)

        daq_frame = ttk.LabelFrame(control_frame, text="Keithley DAQ6510", padding=10)
        daq_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(5, 0))
        self.combo_daq_port = ttk.Combobox(daq_frame, values=self.available_usb_visa)
        if self.available_usb_visa: self.combo_daq_port.set(self.available_usb_visa[0])
        self.combo_daq_port.pack(fill=tk.X, pady=2)
        self.btn_daq_conn = ttk.Button(daq_frame, text="Connect DAQ", command=self.connect_daq_thread)
        self.btn_daq_conn.pack(fill=tk.X, pady=2)
        self.lbl_daq_temps = ttk.Label(daq_frame, text="Temps: -- °C", font=("Arial", 11))
        self.lbl_daq_temps.pack(pady=10)

        log_frame = ttk.LabelFrame(main_frame, text="System Log", padding=5)
        log_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=(10, 0))
        self.log_text = tk.Text(log_frame, height=6, state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True)

        graph_frame = ttk.Frame(main_frame)
        graph_frame.pack(fill=tk.BOTH, expand=True)
        self.fig = Figure(figsize=(12, 4), dpi=100); self.fig.patch.set_facecolor('#f0f0f0')
        self.ax_pwr_v = self.fig.add_subplot(121)
        self.ax_pwr_i = self.ax_pwr_v.twinx()
        self.ax_pwr_v.set_title("Power Supply")
        self.ax_pwr_v.set_xlabel("Time (s)")
        self.ax_pwr_v.set_ylabel("Voltage (V)", color="blue")
        self.ax_pwr_i.set_ylabel("Current (A)", color="red")
        self.line_v, = self.ax_pwr_v.plot([], [], 'b-')
        self.line_i, = self.ax_pwr_i.plot([], [], 'r-')

        self.ax_temp = self.fig.add_subplot(122)
        self.ax_temp.set_title("Thermocouples")
        self.line_temps = {ch: self.ax_temp.plot([], [], label=f"CH {ch}")[0] for ch in CHANNELS}
        self.ax_temp.legend(fontsize=8, loc='upper left')

        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def log(self, message):
        line = f"[{time.strftime('%H:%M:%S')}] {message}\n"
        self.log_history.append(line)
        if len(self.log_history) > 1000: self.log_history = self.log_history[-1000:]
        def append():
            try:
                self.log_text.config(state=tk.NORMAL)
                self.log_text.insert(tk.END, line)
                self.log_text.see(tk.END)
                self.log_text.config(state=tk.DISABLED)
            except: pass
        self.root.after(0, append)

    def update_plots(self):
        self.line_v.set_data(self.t_data, self.v_data)
        self.line_i.set_data(self.t_data, self.i_data)
        for ch in CHANNELS: self.line_temps[ch].set_data(self.t_data, self.temp_data[ch])
        self.ax_pwr_v.relim(); self.ax_pwr_v.autoscale_view()
        self.ax_pwr_i.relim(); self.ax_pwr_i.autoscale_view()
        self.ax_temp.relim(); self.ax_temp.autoscale_view()
        self.canvas.draw()

    def check_enable_test(self):
        with self.state_lock:
            psu = self.state["psu_connected"]
            daq = self.state["daq_connected"]
        if psu or daq:
            self.btn_start_test.config(state=tk.NORMAL)

    def on_closing(self):
        self.stop_event.set()
        if getattr(self, 'csv_file', None): self.csv_file.close()
        
        # Cleanup HW
        if self.psu:
            try:
                self._psu_write(self.psu, "OUTP OFF" if self.psu_protocol == "SCPI" else "OUT 0")
                self.psu.close()
            except: pass
        if self.daq:
            try:
                self.daq.write(":ABOR")
                self.daq.close()
            except: pass
            
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = ThreadedHardwareApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
