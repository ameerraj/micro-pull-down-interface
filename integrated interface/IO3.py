import tkinter as tk
from tkinter import ttk, messagebox
import pyvisa
import serial
import serial.tools.list_ports
import threading
import time
import csv
import copy
import math
from datetime import datetime
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from simple_pid import PID

# ============================================================
# CONFIGURATION & CONSTANTS
# ============================================================
HARDWARE_OVP = 26.0
BAUD_RATE = 9600
CRUCIBLE_MASS_KG = 0.0098  # 9.8g
CRUCIBLE_CP = 130.0        # J/(kg*K) for Iridium

CHANNELS = [101, 102, 103, 104, 105]
CHANNEL_LIST = "(@101:105)"
NPLC = 5.0 # High NPLC to suppress 50Hz switching noise
SCAN_TIMEOUT_SECONDS = 10.0
VISA_TIMEOUT_MS = 5000

TEMPERATURE_CALIBRATION = {
    101: {"gain": 84.33, "offset": 10.78},
    102: {"gain": 84.33, "offset": 20.78},
    103: {"gain": 84.33, "offset": 30.78},
    104: {"gain": 84.33, "offset": 40.78},
    105: {"gain": 84.33, "offset": 50.78},
}

# ============================================================
# THREAD-SAFE SHARED STATE
# ============================================================
class SystemState:
    def __init__(self):
        self.lock = threading.Lock()
        self.ema_alpha = 0.3 # 0.0 to 1.0. Lower = heavier filtering
        
        self.data = {
            "daq": {
                "temps": {ch: 0.0 for ch in CHANNELS},
                "raw_voltages": {ch: 0.0 for ch in CHANNELS},
                "last_updated": 0.0,
                "latency_ms": 0.0
            },
            "psu": {
                "v_meas": 0.0,
                "i_meas": 0.0,
                "target_current": 0.0, # The actual Stellgröße commanded to hardware
                "last_updated": 0.0,
                "latency_ms": 0.0
            },
            "actuator": {
                "pos_mm": 0.0,
                "hw_time_ms": 0,
                "last_updated": 0.0,
                "latency_ms": 0.0
            },
            "control": {
                "emergency_stop": False,
                "manual_ramping": False,
                "target_ramp_i": 0.0,
                "ramp_rate_A_sec": 0.0,
                "max_i_limit": 3.0,
                "max_v_limit": 24.0,
                # PID Parameters
                "pid_active": False,
                "target_temp": 1200.0,
                "control_channel": 101,
                "kp": 0.01,
                "ki": 0.001,
                "kd": 0.0
            }
        }

    def get_state(self):
        with self.lock:
            return copy.deepcopy(self.data)

    def trigger_estop(self):
        with self.lock:
            self.data["control"]["emergency_stop"] = True
            self.data["psu"]["target_current"] = 0.0
            self.data["control"]["manual_ramping"] = False
            self.data["control"]["pid_active"] = False

    def update_daq(self, raw_voltages_dict, calculated_temps_dict, latency):
        with self.lock:
            for ch in CHANNELS:
                self.data["daq"]["raw_voltages"][ch] = raw_voltages_dict[ch]
                
                # Apply EMA Filter to temperature
                current_temp = self.data["daq"]["temps"][ch]
                new_temp = calculated_temps_dict[ch]
                if current_temp == 0.0: # First run initialization
                    self.data["daq"]["temps"][ch] = new_temp
                else:
                    self.data["daq"]["temps"][ch] = (self.ema_alpha * new_temp) + ((1 - self.ema_alpha) * current_temp)
            
            self.data["daq"]["last_updated"] = time.time()
            self.data["daq"]["latency_ms"] = latency

    def update_psu(self, v, i, latency):
        with self.lock:
            self.data["psu"]["v_meas"] = v
            self.data["psu"]["i_meas"] = i
            self.data["psu"]["last_updated"] = time.time()
            self.data["psu"]["latency_ms"] = latency

    def get_psu_target(self):
        with self.lock:
            return self.data["psu"]["target_current"]

    def update_actuator(self, pos, hw_time, latency):
        with self.lock:
            self.data["actuator"]["pos_mm"] = pos
            self.data["actuator"]["hw_time_ms"] = hw_time
            self.data["actuator"]["last_updated"] = time.time()
            self.data["actuator"]["latency_ms"] = latency

# ============================================================
# MAIN APPLICATION
# ============================================================
class UnifiedHardwareApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Multithreaded + PID  Controller & DAQ")
        self.root.geometry("1400x850") 
        
        # Core Architecture
        self.state = SystemState()
        self.shutdown_event = threading.Event()
        self.threads = []
        
        # Hardware References
        self.rm = pyvisa.ResourceManager() 
        self.psu = None
        self.actuator = None
        self.daq = None
        
        # Logging Arrays for Main Thread
        self.logging_active = False
        self.poll_interval = 1.0
        self.csv_file = None
        self.csv_writer = None
        self.t_data, self.v_data, self.i_data, self.pos_data = [], [], [], []
        self.temp_data = {ch: [] for ch in CHANNELS} 
        self.start_time = None

        self.scan_ports() 
        self.setup_ui()
        self.start_daemons()

    def scan_ports(self):
        try:
            visa_resources = self.rm.list_resources()
            self.available_usb = [r for r in visa_resources if "USB" in r]
            self.available_serial = [r for r in visa_resources if "ASRL" in r]
        except Exception:
            self.available_usb, self.available_serial = [], []

        com_ports = serial.tools.list_ports.comports()
        self.available_com = [p.device for p in com_ports]

    # ------------------- THREAD MANAGEMENT -------------------
    def start_daemons(self):
        workers = [
            (self.psu_worker, "PSU_Thread"),
            (self.daq_worker, "DAQ_Thread"),
            (self.actuator_worker, "Actuator_Thread"),
            (self.control_worker, "Master_Control_Thread"),
            (self.logging_worker, "CSV_Logging_Thread")
        ]
        
        for target, name in workers:
            t = threading.Thread(target=target, name=name, daemon=True)
            t.start()
            self.threads.append(t)
            
        self.root.after(100, self.ui_update_loop)

    # ------------------- WORKER THREADS -------------------
    def psu_worker(self):
        """Dedicated thread for TDK-Lambda PSU serial comms."""
        last_sent_current = -1.0
        
        while not self.shutdown_event.is_set():
            if self.psu:
                try:
                    t_start = time.perf_counter()
                    v_meas = float(self.psu.query("MEAS:VOLT?").strip())
                    i_meas = float(self.psu.query("MEAS:CURR?").strip())
                    latency = (time.perf_counter() - t_start) * 1000
                    self.state.update_psu(v_meas, i_meas, latency)
                    
                    target_i = self.state.get_psu_target()
                    if abs(target_i - last_sent_current) > 0.005:
                        self.psu.write(f"CURR {target_i:.3f}")
                        last_sent_current = target_i
                        
                except Exception as e:
                    self.log(f"PSU Comms Error: {e}")
            
            self.shutdown_event.wait(0.1)

    def daq_worker(self):
        """Dedicated thread for Keithley DAQ scans."""
        while not self.shutdown_event.is_set():
            if self.daq:
                try:
                    t_start = time.perf_counter()
                    
                    self.daq.write(':TRAC:CLEAR "defbuffer1"')
                    self.daq.write(":INIT")
                    
                    scan_timeout = time.time() + SCAN_TIMEOUT_SECONDS
                    while "IDLE" not in self.daq.query(":TRIG:STAT?").strip().upper():
                        if time.time() > scan_timeout:
                            raise TimeoutError("DAQ scan timed out.")
                        if self.shutdown_event.is_set():
                            break
                        time.sleep(0.05)
                        
                    raw_data = self.daq.query_ascii_values(f':TRAC:DATA? 1, {len(CHANNELS)}, "defbuffer1", READ, REL')
                    latency = (time.perf_counter() - t_start) * 1000
                    
                    voltages = {CHANNELS[i//2]: float(raw_data[i]) for i in range(0, len(raw_data), 2)}
                    temps = {ch: (voltages[ch] * TEMPERATURE_CALIBRATION[ch]["gain"] + TEMPERATURE_CALIBRATION[ch]["offset"]) for ch in CHANNELS}
                    
                    self.state.update_daq(voltages, temps, latency)
                except Exception as e:
                    self.log(f"DAQ Error: {e}")
            
            self.shutdown_event.wait(0.1 if not self.daq else 0.05)

    def actuator_worker(self):
        """Dedicated thread for Actuator position."""
        while not self.shutdown_event.is_set():
            if self.actuator:
                try:
                    t_start = time.perf_counter()
                    self.actuator.reset_input_buffer()
                    self.actuator.write(b"P\r\n") 
                    self.actuator.flush()
                    
                    raw = self.actuator.readline()
                    latency = (time.perf_counter() - t_start) * 1000
                    
                    if raw:  
                        text = raw.decode("ascii", errors="replace").strip()
                        parts = text.split(",")
                        pos_mm = float(parts[0].replace(",", "."))
                        hw_time = int(parts[1]) if len(parts) > 1 else 0
                        self.state.update_actuator(pos_mm, hw_time, latency)
                except Exception:
                    pass
            self.shutdown_event.wait(0.2)

    def control_worker(self):
        """The Master Metronome: 10Hz thread for Ramping, PID, and Safety."""
        dt = 0.1 # 10 Hz
        pid = PID(Kp=0.01, Ki=0.001, Kd=0.0, setpoint=0.0, sample_time=dt)
        pid.set_auto_mode(False)
        
        while not self.shutdown_event.is_set():
            s = self.state.get_state()
            now = time.time()
            
            # --- TIERED WATCHDOG SAFETY CHECK ---
            if self.daq and not s["control"]["emergency_stop"]:
                daq_age = now - s["daq"]["last_updated"]
                if daq_age > 10.0:
                    self.state.trigger_estop()
                    self.log("🛑 E-STOP: DAQ frozen for >10s.")
                elif daq_age > 5.0:
                    self.shutdown_event.wait(dt)
                    continue 

            # --- CONTROL LOGIC ---
            if s["control"]["emergency_stop"]:
                if pid.auto_mode: pid.set_auto_mode(False)
                
            elif s["control"]["manual_ramping"]:
                if pid.auto_mode: pid.set_auto_mode(False)
                current_target = s["psu"]["target_current"]
                final_target = s["control"]["target_ramp_i"]
                delta = s["control"]["ramp_rate_A_sec"] * dt
                
                if current_target < final_target:
                    new_target = min(current_target + delta, final_target)
                else:
                    new_target = max(current_target - delta, final_target)
                
                new_target = min(new_target, s["control"]["max_i_limit"])
                
                with self.state.lock:
                    self.state.data["psu"]["target_current"] = new_target
                    if math.isclose(new_target, final_target, abs_tol=0.001):
                        self.state.data["control"]["manual_ramping"] = False
                        self.log(f"Ramp complete. Holding at {final_target} A.")
                        
            elif s["control"]["pid_active"]:
                # Bumpless Transfer Initialization
                if not pid.auto_mode:
                    current_i = s["psu"]["target_current"]
                    pid.tunings = (s["control"]["kp"], s["control"]["ki"], s["control"]["kd"])
                    pid.setpoint = s["control"]["target_temp"]
                    
                    # Strict Clamp: +/- 0.5A from the steady state at moment of engagement
                    lower_limit = max(0.0, current_i - 0.5)
                    upper_limit = min(s["control"]["max_i_limit"], current_i + 0.5)
                    pid.output_limits = (lower_limit, upper_limit)
                    
                    pid.set_auto_mode(True, last_output=current_i)
                    self.log(f"PID Activated. Holding {pid.setpoint}°C around {current_i:.2f}A.")

                pv_temp = s["daq"]["temps"][s["control"]["control_channel"]]
                new_target_i = pid(pv_temp)
                
                with self.state.lock:
                    self.state.data["psu"]["target_current"] = new_target_i
            else:
                if pid.auto_mode: pid.set_auto_mode(False)

            self.shutdown_event.wait(dt)

    def logging_worker(self):
        """Independent thread for writing to CSV precisely at poll_interval."""
        while not self.shutdown_event.is_set():
            if self.logging_active and self.csv_writer and self.start_time:
                s = self.state.get_state()
                elapsed = time.time() - self.start_time
                
                power = s["psu"]["v_meas"] * s["psu"]["i_meas"]
                heat_rate = power / (CRUCIBLE_MASS_KG * CRUCIBLE_CP)
                
                row = [
                    datetime.now().isoformat(), round(elapsed, 3), 
                    s["psu"]["v_meas"], s["psu"]["i_meas"], round(heat_rate, 3),
                    s["actuator"]["pos_mm"], s["actuator"]["hw_time_ms"],
                    round(s["psu"]["latency_ms"], 2), round(s["actuator"]["latency_ms"], 2), round(s["daq"]["latency_ms"], 2)
                ]
                for ch in CHANNELS:
                    row.extend([s["daq"]["raw_voltages"][ch], round(s["daq"]["temps"][ch], 2)])
                
                try:
                    self.csv_writer.writerow(row)
                    self.csv_file.flush()
                    
                    self.t_data.append(elapsed)
                    self.v_data.append(s["psu"]["v_meas"])
                    self.i_data.append(s["psu"]["i_meas"])
                    self.pos_data.append(s["actuator"]["pos_mm"])
                    for ch in CHANNELS:
                        self.temp_data[ch].append(s["daq"]["temps"][ch])
                except Exception as e:
                    print(f"Logging error: {e}")
                    
            self.shutdown_event.wait(self.poll_interval if self.logging_active else 0.5)

    # ------------------- UI & MAIN LOOP -------------------
    def ui_update_loop(self):
        if self.shutdown_event.is_set(): return
        s = self.state.get_state()
        
        self.lbl_v_meas.config(text=f"Meas V: {s['psu']['v_meas']:.3f} V")
        self.lbl_i_meas.config(text=f"Meas I: {s['psu']['i_meas']:.3f} A")
        self.lbl_pos.config(text=f"Pos: {s['actuator']['pos_mm']:.2f} mm")
        
        heat_rate = (s["psu"]["v_meas"] * s["psu"]["i_meas"]) / (CRUCIBLE_MASS_KG * CRUCIBLE_CP)
        self.lbl_heat_rate.config(text=f"Heat Rate: {heat_rate:.2f} K/s")
        
        temp_str = " | ".join([f"CH{ch}: {s['daq']['temps'][ch]:.1f}°C" for ch in CHANNELS])
        self.lbl_daq_temps.config(text=temp_str)
        
        if s["control"]["emergency_stop"]:
            self.btn_stop_ramp.config(state=tk.DISABLED)
            self.btn_apply.config(state=tk.DISABLED)
            self.btn_start_pid.config(state=tk.DISABLED)
            self.btn_stop_pid.config(state=tk.DISABLED)
            
        if self.logging_active and len(self.t_data) > 0:
            self.update_plots()

        self.root.after(100, self.ui_update_loop)

    def on_closing(self):
        self.log("Initiating safe orchestrated shutdown...")
        self.shutdown_event.set()
        
        for t in self.threads:
            if t.is_alive(): t.join(timeout=1.5)
                
        if self.psu:
            try:
                self.psu.write("VOLT 0.0")
                self.psu.write("CURR 0.0")
                self.psu.write("OUTP OFF")
                self.psu.write("SYST:REM LOC")
                self.psu.close()
            except: pass
        if self.daq:
            try:
                self.daq.write(":ABOR")
                self.daq.close()
            except: pass
        if self.actuator:
            try: self.actuator.close()
            except: pass
        if self.csv_file:
            self.csv_file.close()

        self.root.destroy()

    # ------------------- COMMANDS & UI ACTIONS -------------------
    def connect_psu(self):
        port = self.combo_psu_port.get().strip()
        try:
            self.psu = self.rm.open_resource(
                port, baud_rate=115200, data_bits=8, parity=pyvisa.constants.Parity.none,
                stop_bits=pyvisa.constants.StopBits.one, read_termination="\r\n", write_termination="\r", timeout=2000
            )
            self.psu.write("INST:NSEL 6")
            self.psu.write(f"VOLT:PROT:LEV {HARDWARE_OVP}")
            self.psu.write("VOLT 0.0")
            self.psu.write("CURR 0.0")
            self.psu.write("OUTP ON")
            
            self.btn_psu_conn.config(text="PSU Connected", state=tk.DISABLED)
            self.btn_apply.config(state=tk.NORMAL)
            self.btn_start_pid.config(state=tk.NORMAL)
            self.btn_safe_off.config(state=tk.NORMAL)
            self.log("PSU connected. Control thread taking over.")
        except Exception as e:
            self.log(f"PSU Connection failed: {e}")

    def start_current_ramp(self):
        try:
            with self.state.lock:
                self.state.data["control"]["max_v_limit"] = float(self.entry_max_v.get())
                self.state.data["control"]["max_i_limit"] = float(self.entry_max_i.get())
                self.state.data["control"]["target_ramp_i"] = float(self.entry_target_i.get())
                self.state.data["control"]["ramp_rate_A_sec"] = float(self.entry_ramp_rate.get()) / 60.0
                self.state.data["control"]["manual_ramping"] = True
                self.state.data["control"]["pid_active"] = False # Ensure PID is off
                self.state.data["control"]["emergency_stop"] = False
                
                target_v = float(self.entry_target_v.get())
                self.psu.write(f"VOLT {target_v}")
                
            self.btn_stop_ramp.config(state=tk.NORMAL)
            self.btn_start_pid.config(state=tk.NORMAL)
            self.btn_stop_pid.config(state=tk.DISABLED)
            self.log("Ramp parameters passed to Master Control Thread.")
        except ValueError:
            messagebox.showerror("Input Error", "Invalid numeric inputs.")

    def stop_current_ramp(self):
        with self.state.lock:
            self.state.data["control"]["manual_ramping"] = False
        self.btn_stop_ramp.config(state=tk.DISABLED)
        self.log("Ramp manually stopped.")

    def start_pid_hold(self):
        try:
            target_temp = float(self.entry_target_temp.get())
            with self.state.lock:
                self.state.data["control"]["manual_ramping"] = False
                self.state.data["control"]["pid_active"] = True
                self.state.data["control"]["target_temp"] = target_temp
                
            self.btn_start_pid.config(state=tk.DISABLED)
            self.btn_stop_pid.config(state=tk.NORMAL)
            self.btn_stop_ramp.config(state=tk.DISABLED)
        except ValueError:
            messagebox.showerror("Input Error", "Invalid target temperature.")

    def stop_pid_hold(self):
        with self.state.lock:
            self.state.data["control"]["pid_active"] = False
            
        self.btn_start_pid.config(state=tk.NORMAL)
        self.btn_stop_pid.config(state=tk.DISABLED)
        self.log("PID deactivated. Holding at last calculated current.")

    def safe_heater_off(self):
        self.state.trigger_estop()
        self.btn_stop_ramp.config(state=tk.DISABLED)
        self.btn_stop_pid.config(state=tk.DISABLED)
        self.log("🛑 SAFE OFF TRIGGERED by User.")

    def start_test(self):
        try:
            self.poll_interval = float(self.entry_poll_interval.get())
            if self.poll_interval <= 0: raise ValueError
        except ValueError:
            return messagebox.showerror("Error", "Invalid interval.")
            
        self.t_data.clear(); self.v_data.clear(); self.i_data.clear(); self.pos_data.clear()
        for ch in CHANNELS: self.temp_data[ch].clear()
            
        self.start_time = time.time()
        filename = f"daq_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.csv_file = open(filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        headers = ["ISO_Timestamp", "Elapsed_s", "PSU_V", "PSU_A", "Heating_Rate_Ks", "Pos_mm", "Act_HW_ms", "Lat_PSU", "Lat_Act", "Lat_DAQ"]
        for ch in CHANNELS: headers.extend([f"CH{ch}_V", f"CH{ch}_C"])
        self.csv_writer.writerow(headers)
        
        self.logging_active = True
        self.btn_start_test.config(state=tk.DISABLED)
        self.btn_stop_test.config(state=tk.NORMAL)
        self.log(f"Logging thread started at {self.poll_interval}s intervals.")

    def stop_test(self):
        self.logging_active = False
        if self.csv_file: self.csv_file.close()
        self.btn_start_test.config(state=tk.NORMAL)
        self.btn_stop_test.config(state=tk.DISABLED)
        self.log("Logging thread stopped.")

    def connect_actuator(self):
        port = self.combo_act_port.get().strip()
        try:
            self.actuator = serial.Serial(port, BAUD_RATE, timeout=0.8)
            time.sleep(1.0)
            self.actuator.reset_input_buffer()
            self.btn_act_conn.config(text="Actuator Connected", state=tk.DISABLED)
            self.lbl_act_status.config(text="Status: Ready")
            self.btn_start_test.config(state=tk.NORMAL)
        except Exception as e: self.log(str(e))

    def connect_daq(self):
        port = self.combo_daq_port.get().strip()
        try:
            self.daq = self.rm.open_resource(port)
            self.daq.timeout = VISA_TIMEOUT_MS
            self.daq.write_termination = "\n"
            self.daq.read_termination = "\n"
            
            self.daq.write("*RST")
            self.daq.write(f':SENS:FUNC "VOLT:DC", {CHANNEL_LIST}')
            self.daq.write(f":SENS:VOLT:DC:NPLC {NPLC}, {CHANNEL_LIST}")
            self.daq.write(f":ROUT:SCAN:CRE {CHANNEL_LIST}")
            self.daq.write(":ROUT:SCAN:COUNT:SCAN 1")
            
            self.btn_daq_conn.config(text="DAQ Connected", state=tk.DISABLED)
            self.lbl_daq_status.config(text="Status: Configured")
            self.btn_start_test.config(state=tk.NORMAL)
        except Exception as e: self.log(str(e))

    def update_plots(self):
        self.line_v.set_data(self.t_data, self.v_data)
        self.line_i.set_data(self.t_data, self.i_data)
        self.line_pos.set_data(self.t_data, self.pos_data)
        for ch in CHANNELS: self.line_temps[ch].set_data(self.t_data, self.temp_data[ch])

        for ax in [self.ax_pwr_v, self.ax_pwr_i, self.ax_pos, self.ax_temp]:
            ax.relim()
            ax.autoscale_view()
        self.canvas.draw_idle()

    # ------------------- UI LAYOUT -------------------
    def setup_ui(self):
        style = ttk.Style()
        if "clam" in style.theme_names(): style.theme_use("clam")

        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        control_frame = ttk.Frame(main_frame)
        control_frame.pack(fill=tk.X, pady=(0, 15))
        
        control_frame.columnconfigure(0, weight=5)
        control_frame.columnconfigure(1, weight=3)
        control_frame.columnconfigure(2, weight=3)

        # 1. PSU & PID CONTROLS
        psu_frame = ttk.LabelFrame(control_frame, text="Power Supply & Temperature Control", padding=10)
        psu_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        for i in range(4): psu_frame.columnconfigure(i, weight=1)

        ttk.Label(psu_frame, text="Port:").grid(row=0, column=0, sticky="e", padx=5, pady=4)
        self.combo_psu_port = ttk.Combobox(psu_frame, values=self.available_serial)
        if self.available_serial: self.combo_psu_port.set(self.available_serial[0])
        self.combo_psu_port.grid(row=0, column=1, sticky="ew", padx=5, pady=4)
        self.btn_psu_conn = ttk.Button(psu_frame, text="Connect PSU", command=self.connect_psu)
        self.btn_psu_conn.grid(row=0, column=2, columnspan=2, sticky="ew", padx=5, pady=4)

        ttk.Label(psu_frame, text="Max V Limit:").grid(row=1, column=0, sticky="e", padx=5, pady=4)
        self.entry_max_v = ttk.Entry(psu_frame, width=8); self.entry_max_v.insert(0, "24.0"); self.entry_max_v.grid(row=1, column=1, sticky="ew", padx=5, pady=4)
        ttk.Label(psu_frame, text="Max I Limit:").grid(row=1, column=2, sticky="e", padx=5, pady=4)
        self.entry_max_i = ttk.Entry(psu_frame, width=8); self.entry_max_i.insert(0, "3.0"); self.entry_max_i.grid(row=1, column=3, sticky="ew", padx=5, pady=4)

        ttk.Label(psu_frame, text="Target V:").grid(row=2, column=0, sticky="e", padx=5, pady=4)
        self.entry_target_v = ttk.Entry(psu_frame, width=8); self.entry_target_v.insert(0, "0.0"); self.entry_target_v.grid(row=2, column=1, sticky="ew", padx=5, pady=4)
        ttk.Label(psu_frame, text="Target I:").grid(row=2, column=2, sticky="e", padx=5, pady=4)
        self.entry_target_i = ttk.Entry(psu_frame, width=8); self.entry_target_i.insert(0, "0.0"); self.entry_target_i.grid(row=2, column=3, sticky="ew", padx=5, pady=4)

        ttk.Label(psu_frame, text="Ramp (A/min):").grid(row=3, column=0, sticky="e", padx=5, pady=4)
        self.entry_ramp_rate = ttk.Entry(psu_frame, width=8); self.entry_ramp_rate.insert(0, "0.5"); self.entry_ramp_rate.grid(row=3, column=1, sticky="ew", padx=5, pady=4)
        self.btn_apply = ttk.Button(psu_frame, text="Start Ramp", command=self.start_current_ramp, state=tk.DISABLED); self.btn_apply.grid(row=3, column=2, sticky="ew", padx=5, pady=4)
        self.btn_stop_ramp = ttk.Button(psu_frame, text="Stop Ramp", command=self.stop_current_ramp, state=tk.DISABLED); self.btn_stop_ramp.grid(row=3, column=3, sticky="ew", padx=5, pady=4)

        ttk.Label(psu_frame, text="PID Target Temp (°C):").grid(row=4, column=0, sticky="e", padx=5, pady=4)
        self.entry_target_temp = ttk.Entry(psu_frame, width=8); self.entry_target_temp.insert(0, "1200.0"); self.entry_target_temp.grid(row=4, column=1, sticky="ew", padx=5, pady=4)
        self.btn_start_pid = ttk.Button(psu_frame, text="Start PID Hold", command=self.start_pid_hold, state=tk.DISABLED); self.btn_start_pid.grid(row=4, column=2, sticky="ew", padx=5, pady=4)
        self.btn_stop_pid = ttk.Button(psu_frame, text="Stop PID Hold", command=self.stop_pid_hold, state=tk.DISABLED); self.btn_stop_pid.grid(row=4, column=3, sticky="ew", padx=5, pady=4)

        self.btn_safe_off = tk.Button(psu_frame, text="🛑 SAFELY TURN OFF HEATER", bg="#ffcccc", fg="#cc0000", font=("Arial", 10, "bold"), command=self.safe_heater_off, state=tk.DISABLED)
        self.btn_safe_off.grid(row=5, column=0, columnspan=4, sticky="ew", padx=5, pady=(10, 5))

        readings_frame = ttk.Frame(psu_frame)
        readings_frame.grid(row=6, column=0, columnspan=4, pady=5)
        self.lbl_v_meas = ttk.Label(readings_frame, text="Meas V: -- V", font=("Consolas", 12, "bold"), foreground="#0066cc"); self.lbl_v_meas.pack(side=tk.LEFT, padx=15)
        self.lbl_i_meas = ttk.Label(readings_frame, text="Meas I: -- A", font=("Consolas", 12, "bold"), foreground="#cc0000"); self.lbl_i_meas.pack(side=tk.LEFT, padx=15)
        self.lbl_heat_rate = ttk.Label(readings_frame, text="Heat Rate: -- K/s", font=("Consolas", 11), foreground="#d97706"); self.lbl_heat_rate.pack(side=tk.LEFT, padx=15)

        # 2. ACTUATOR & LOGGING
        act_frame = ttk.LabelFrame(control_frame, text="Linear Actuator & Logging", padding=10)
        act_frame.grid(row=0, column=1, sticky="nsew", padx=(5, 5))
        act_frame.columnconfigure(1, weight=1)

        ttk.Label(act_frame, text="Port:").grid(row=0, column=0, sticky="e", padx=5, pady=4)
        self.combo_act_port = ttk.Combobox(act_frame, values=self.available_com)
        if self.available_com: self.combo_act_port.set(self.available_com[0])
        self.combo_act_port.grid(row=0, column=1, sticky="ew", padx=5, pady=4)
        
        self.btn_act_conn = ttk.Button(act_frame, text="Connect Actuator", command=self.connect_actuator)
        self.btn_act_conn.grid(row=1, column=0, columnspan=2, sticky="ew", padx=5, pady=4)
        self.lbl_act_status = ttk.Label(act_frame, text="Status: Disconnected", foreground="gray"); self.lbl_act_status.grid(row=2, column=0, columnspan=2, pady=(0, 10))
        self.lbl_pos = ttk.Label(act_frame, text="Pos: -- mm", font=("Consolas", 14, "bold"), foreground="#2ca02c"); self.lbl_pos.grid(row=3, column=0, columnspan=2, pady=10)

        ttk.Label(act_frame, text="Poll Interval (s):").grid(row=4, column=0, sticky="e", padx=5, pady=4)
        self.entry_poll_interval = ttk.Entry(act_frame, width=8); self.entry_poll_interval.insert(0, "1.0"); self.entry_poll_interval.grid(row=4, column=1, sticky="ew", padx=5, pady=4)

        log_btn_frame = ttk.Frame(act_frame); log_btn_frame.grid(row=5, column=0, columnspan=2, sticky="ew", pady=(10, 0))
        log_btn_frame.columnconfigure(0, weight=1); log_btn_frame.columnconfigure(1, weight=1)
        self.btn_start_test = ttk.Button(log_btn_frame, text="Start Logging", command=self.start_test, state=tk.DISABLED); self.btn_start_test.grid(row=0, column=0, sticky="ew", padx=(0, 2))
        self.btn_stop_test = ttk.Button(log_btn_frame, text="Stop Logging", command=self.stop_test, state=tk.DISABLED); self.btn_stop_test.grid(row=0, column=1, sticky="ew", padx=(2, 0))

        # 3. DAQ CONTROLS
        daq_frame = ttk.LabelFrame(control_frame, text="Keithley DAQ6510", padding=10)
        daq_frame.grid(row=0, column=2, sticky="nsew", padx=(5, 0))
        daq_frame.columnconfigure(0, weight=1)

        ttk.Label(daq_frame, text="VISA Port:").grid(row=0, column=0, sticky="w", padx=5, pady=(0, 2))
        self.combo_daq_port = ttk.Combobox(daq_frame, values=self.available_usb)
        if self.available_usb: self.combo_daq_port.set(self.available_usb[0])
        self.combo_daq_port.grid(row=1, column=0, sticky="ew", padx=5, pady=(0, 4))
        self.btn_daq_conn = ttk.Button(daq_frame, text="Connect DAQ", command=self.connect_daq); self.btn_daq_conn.grid(row=2, column=0, sticky="ew", padx=5, pady=4)
        self.lbl_daq_status = ttk.Label(daq_frame, text="Status: Disconnected", foreground="gray"); self.lbl_daq_status.grid(row=3, column=0, pady=(0, 15))
        self.lbl_daq_temps = ttk.Label(daq_frame, text="Temps: -- °C", font=("Consolas", 11), justify="left", wraplength=220); self.lbl_daq_temps.grid(row=4, column=0, sticky="w", padx=5)

        # GRAPHS
        graph_frame = ttk.Frame(main_frame); graph_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        self.fig = Figure(figsize=(12, 4), dpi=100); self.fig.patch.set_facecolor('#f0f0f0')

        self.ax_pwr_v = self.fig.add_subplot(131); self.ax_pwr_i = self.ax_pwr_v.twinx()
        self.ax_pwr_v.set_title("Power Supply", fontsize=10, weight="bold"); self.ax_pwr_v.set_xlabel("Time (s)", fontsize=9); self.ax_pwr_v.set_ylabel("Voltage (V)", color="#0066cc", fontsize=9); self.ax_pwr_i.set_ylabel("Current (A)", color="#cc0000", fontsize=9); self.ax_pwr_v.grid(True, linestyle="--", alpha=0.5)
        self.line_v, = self.ax_pwr_v.plot([], [], color='#0066cc', label="Volts"); self.line_i, = self.ax_pwr_i.plot([], [], color='#cc0000', label="Amps")

        self.ax_pos = self.fig.add_subplot(132); self.ax_pos.set_title("Actuator Position", fontsize=10, weight="bold"); self.ax_pos.set_xlabel("Time (s)", fontsize=9); self.ax_pos.set_ylabel("Position (mm)", color="#2ca02c", fontsize=9); self.ax_pos.grid(True, linestyle="--", alpha=0.5)
        self.line_pos, = self.ax_pos.plot([], [], color='#2ca02c')

        self.ax_temp = self.fig.add_subplot(133); self.ax_temp.set_title("Thermocouple Arrays", fontsize=10, weight="bold"); self.ax_temp.set_xlabel("Time (s)", fontsize=9); self.ax_temp.set_ylabel("Temperature (°C)", fontsize=9); self.ax_temp.grid(True, linestyle="--", alpha=0.5)
        colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728', '#9467bd']
        self.line_temps = {ch: self.ax_temp.plot([], [], color=colors[i%len(colors)], label=f"CH {ch}")[0] for i, ch in enumerate(CHANNELS)}
        self.ax_temp.legend(fontsize=8, loc='upper left')

        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame); self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # LOGS
        log_frame = ttk.LabelFrame(main_frame, text="System Log", padding=5); log_frame.pack(fill=tk.X, pady=(10, 0))
        scrollbar = ttk.Scrollbar(log_frame); scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        self.log_text = tk.Text(log_frame, height=6, state=tk.DISABLED, yscrollcommand=scrollbar.set, font=("Consolas", 9))
        self.log_text.pack(fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.log_text.yview)

    def log(self, message):
        def append():
            self.log_text.config(state=tk.NORMAL)
            self.log_text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {message}\n")
            self.log_text.see(tk.END)
            self.log_text.config(state=tk.DISABLED)
        self.root.after(0, append)

if __name__ == "__main__":
    root = tk.Tk()
    app = UnifiedHardwareApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
