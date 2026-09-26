import tkinter as tk
from tkinter import ttk, messagebox
import pyvisa
import serial
import threading
import time
import csv
import math
from datetime import datetime
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ============================================================
# HARDCODED SAFETY LIMITS & PORT CONFIG
# ============================================================
PSU_PORT = "ASRL3::INSTR"
HARDWARE_OVP = 26.0

ACTUATOR_PORT = "COM3"
BAUD_RATE = 9600
ACTUATOR_STARTUP_SECONDS = 5.0
SERIAL_RESPONSE_TIMEOUT_SECONDS = 0.8
POLL_INTERVAL_SECONDS = 1.0

# ============================================================
# THERMAL CONSTANTS
# ============================================================
CRUCIBLE_MASS_KG = 0.0098  # 9.8g
CRUCIBLE_CP = 130.0        # J/(kg*K) for Iridium

# ============================================================
# DAQ6510 CONSTANTS & CALIBRATION
# ============================================================
DAQ_PORT = "USB0::0x05E6::0x6510::04437244::INSTR"
CHANNELS = [101, 102, 103, 104, 105]
CHANNEL_LIST = "(@101:105)"
NPLC = 1.0
SCAN_TIMEOUT_SECONDS = 10.0
VISA_TIMEOUT_MS = 5000

# temperature = gain * measured Voltage (mV) + offset 
TEMPERATURE_CALIBRATION = {
    101: {"gain": 84.33, "offset": 10.78}, # thermocouple
    102: {"gain": 84.33, "offset": 20.78},
    103: {"gain": 84.33, "offset": 30.78},
    104: {"gain": 84.33, "offset": 40.78},
    105: {"gain": 84.33, "offset": 50.78},
}

class UnifiedHardwareApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Advanced Hardware Controller & DAQ")
        self.root.geometry("1400x850") 
        
        # Hardware & DAQ States
        self.rm = pyvisa.ResourceManager() 
        self.psu = None
        self.actuator = None
        self.daq = None
        
        self.test_running = False
        self.csv_file = None
        self.csv_writer = None

        # User-defined safety ceilings
        self.max_v_limit = 24.0
        self.max_i_limit = 3.0

        # Ramping States
        self.ramping_active = False
        self.psu_target_i = 0.0
        self.psu_current_setpoint = 0.0
        self.ramp_rate_A_per_sec = 0.0
        self.last_ramp_time = 0.0

        # Plot Data Arrays
        self.t_data = []
        self.v_data = []
        self.i_data = []
        self.pos_data = []
        self.temp_data = {ch: [] for ch in CHANNELS} 
        self.start_time = None

        self.setup_ui()
        
        # Start unified polling thread
        self.polling_active = True
        self.poll_thread = threading.Thread(target=self.hardware_poll_loop, daemon=True)
        self.poll_thread.start()

    def setup_ui(self):
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # ==================== TOP ROW: CONTROLS ====================
        control_frame = ttk.Frame(main_frame)
        control_frame.pack(fill=tk.X, pady=(0, 10))

        # --- PSU CONTROLS ---
        psu_frame = ttk.LabelFrame(control_frame, text="TDK-Lambda Power Supply", padding=10)
        psu_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))

        self.btn_psu_conn = ttk.Button(psu_frame, text="Connect PSU", command=self.connect_psu)
        self.btn_psu_conn.grid(row=0, column=0, columnspan=4, pady=5, sticky="ew")

        ttk.Label(psu_frame, text="Max Allowable V:").grid(row=1, column=0, sticky="w")
        self.entry_max_v = ttk.Entry(psu_frame, width=8)
        self.entry_max_v.grid(row=1, column=1, padx=5, pady=2)
        self.entry_max_v.insert(0, "24.0")

        ttk.Label(psu_frame, text="Max Allowable I:").grid(row=1, column=2, sticky="w")
        self.entry_max_i = ttk.Entry(psu_frame, width=8)
        self.entry_max_i.grid(row=1, column=3, padx=5, pady=2)
        self.entry_max_i.insert(0, "3.0")

        ttk.Label(psu_frame, text="Target Voltage (V):").grid(row=2, column=0, sticky="w")
        self.entry_target_v = ttk.Entry(psu_frame, width=8)
        self.entry_target_v.grid(row=2, column=1, padx=5, pady=2)
        self.entry_target_v.insert(0, "0.0")

        ttk.Label(psu_frame, text="Target Current (A):").grid(row=2, column=2, sticky="w")
        self.entry_target_i = ttk.Entry(psu_frame, width=8)
        self.entry_target_i.grid(row=2, column=3, padx=5, pady=2)
        self.entry_target_i.insert(0, "0.0")

        ttk.Label(psu_frame, text="Ramp Rate (A/min):").grid(row=3, column=0, sticky="w")
        self.entry_ramp_rate = ttk.Entry(psu_frame, width=8)
        self.entry_ramp_rate.grid(row=3, column=1, padx=5, pady=2)
        self.entry_ramp_rate.insert(0, "0.5")

        self.btn_apply = ttk.Button(psu_frame, text="Start Ramp", command=self.start_current_ramp, state=tk.DISABLED)
        self.btn_apply.grid(row=3, column=2, pady=5, sticky="ew", padx=2)

        self.btn_stop_ramp = ttk.Button(psu_frame, text="Stop Ramp (Hold)", command=self.stop_current_ramp, state=tk.DISABLED)
        self.btn_stop_ramp.grid(row=3, column=3, pady=5, sticky="ew", padx=2)

        self.btn_safe_off = ttk.Button(psu_frame, text="🛑 SAFELY TURN OFF HEATER", command=self.safe_heater_off, state=tk.DISABLED)
        self.btn_safe_off.grid(row=4, column=0, columnspan=4, pady=5, sticky="ew")

        self.lbl_v_meas = ttk.Label(psu_frame, text="Meas V: -- V", font=("Arial", 11, "bold"), foreground="green")
        self.lbl_v_meas.grid(row=5, column=0, columnspan=2, pady=5)
        self.lbl_i_meas = ttk.Label(psu_frame, text="Meas I: -- A", font=("Arial", 11, "bold"), foreground="green")
        self.lbl_i_meas.grid(row=5, column=2, columnspan=2, pady=5)

        self.lbl_heat_rate = ttk.Label(psu_frame, text="Heating Rate: -- K/s", font=("Arial", 11, "bold"), foreground="darkorange")
        self.lbl_heat_rate.grid(row=6, column=0, columnspan=4, pady=2)

        # --- ACTUATOR CONTROLS ---
        act_frame = ttk.LabelFrame(control_frame, text="Linear Actuator", padding=10)
        act_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(5, 5))

        self.btn_act_conn = ttk.Button(act_frame, text="Connect Actuator", command=self.connect_actuator_thread)
        self.btn_act_conn.grid(row=0, column=0, columnspan=2, pady=5, sticky="ew")

        self.lbl_act_status = ttk.Label(act_frame, text="Status: Disconnected")
        self.lbl_act_status.grid(row=1, column=0, columnspan=2, pady=5)

        self.lbl_pos = ttk.Label(act_frame, text="Pos: -- mm", font=("Arial", 14, "bold"), foreground="blue")
        self.lbl_pos.grid(row=2, column=0, columnspan=2, pady=10)

        self.btn_start_test = ttk.Button(act_frame, text="Start Data Logging", command=self.start_test, state=tk.DISABLED)
        self.btn_start_test.grid(row=3, column=0, pady=5, padx=2, sticky="ew")

        self.btn_stop_test = ttk.Button(act_frame, text="Stop Logging", command=self.stop_test, state=tk.DISABLED)
        self.btn_stop_test.grid(row=3, column=1, pady=5, padx=2, sticky="ew")

        # --- DAQ CONTROLS ---
        daq_frame = ttk.LabelFrame(control_frame, text="Keithley DAQ6510", padding=10)
        daq_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 0))

        self.btn_daq_conn = ttk.Button(daq_frame, text="Connect DAQ", command=self.connect_daq_thread)
        self.btn_daq_conn.pack(fill=tk.X, pady=5)

        self.lbl_daq_status = ttk.Label(daq_frame, text="Status: Disconnected")
        self.lbl_daq_status.pack(pady=5)

        self.lbl_daq_temps = ttk.Label(daq_frame, text="Temps: -- °C", font=("Arial", 11), wraplength=200)
        self.lbl_daq_temps.pack(pady=10)

        # ==================== MIDDLE ROW: GRAPHS ====================
        graph_frame = ttk.Frame(main_frame)
        graph_frame.pack(fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(12, 4), dpi=100)
        self.fig.patch.set_facecolor('#f0f0f0')

        # Subplot 1: Power
        self.ax_pwr_v = self.fig.add_subplot(131)
        self.ax_pwr_i = self.ax_pwr_v.twinx()
        self.ax_pwr_v.set_title("Power Supply")
        self.ax_pwr_v.set_xlabel("Time (s)")
        self.ax_pwr_v.set_ylabel("Voltage (V)", color="blue")
        self.ax_pwr_i.set_ylabel("Current (A)", color="red")
        
        self.line_v, = self.ax_pwr_v.plot([], [], 'b-', label="Voltage")
        self.line_i, = self.ax_pwr_i.plot([], [], 'r-', label="Current")

        # Subplot 2: Position
        self.ax_pos = self.fig.add_subplot(132)
        self.ax_pos.set_title("Actuator Position")
        self.ax_pos.set_xlabel("Time (s)")
        self.ax_pos.set_ylabel("Position (mm)", color="green")
        self.line_pos, = self.ax_pos.plot([], [], 'g-', label="Position")

        # Subplot 3: DAQ Temperatures
        self.ax_temp = self.fig.add_subplot(133)
        self.ax_temp.set_title("Thermocouples")
        self.ax_temp.set_xlabel("Time (s)")
        self.ax_temp.set_ylabel("Temperature (°C)")
        
        self.line_temps = {}
        for ch in CHANNELS:
            self.line_temps[ch], = self.ax_temp.plot([], [], label=f"CH {ch}")
        self.ax_temp.legend(fontsize=8, loc='upper left')

        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # ==================== BOTTOM ROW: LOGS ====================
        log_frame = ttk.LabelFrame(main_frame, text="System Log", padding=5)
        log_frame.pack(fill=tk.X, pady=(10, 0))

        self.log_text = tk.Text(log_frame, height=6, state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def log(self, message):
        def append():
            self.log_text.config(state=tk.NORMAL)
            self.log_text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {message}\n")
            self.log_text.see(tk.END)
            self.log_text.config(state=tk.DISABLED)
        self.root.after(0, append)

    # ------------------- PSU METHODS -------------------
    def connect_psu(self):
        try:
            self.psu = self.rm.open_resource(
                PSU_PORT, baud_rate=115200, data_bits=8,
                parity=pyvisa.constants.Parity.none,
                stop_bits=pyvisa.constants.StopBits.one,
                read_termination="\r\n", write_termination="\r", timeout=2000
            )
            self.psu.write("INST:NSEL 6")
            self.psu.write(f"VOLT:PROT:LEV {HARDWARE_OVP}")
            self.psu.write("VOLT 0.0")
            self.psu.write("CURR 0.0")
            self.psu.write("OUTP ON")
            
            self.btn_psu_conn.config(text="PSU Connected", state=tk.DISABLED)
            self.btn_apply.config(state=tk.NORMAL)
            self.btn_safe_off.config(state=tk.NORMAL)
            self.log("PSU connected. Output enabled at 0V / 0A.")
        except Exception as e:
            self.log(f"PSU Connection failed: {e}")

    def start_current_ramp(self):
        if not self.psu: return
        try:
            self.max_v_limit = float(self.entry_max_v.get())
            self.max_i_limit = float(self.entry_max_i.get())
            target_v = float(self.entry_target_v.get())
            target_i = float(self.entry_target_i.get())
            ramp_rate_A_min = float(self.entry_ramp_rate.get())

            if target_v > self.max_v_limit or target_i > self.max_i_limit:
                messagebox.showerror("Limit Error", f"Targets exceed maximum stored limits ({self.max_v_limit}V, {self.max_i_limit}A).")
                return

            self.psu.write(f"VOLT {target_v}")
            current_setpoint_str = self.psu.query("CURR?").strip()
            self.psu_current_setpoint = float(current_setpoint_str)

            self.psu_target_i = target_i
            self.ramp_rate_A_per_sec = ramp_rate_A_min / 60.0
            self.last_ramp_time = time.time()
            self.ramping_active = True
            self.btn_stop_ramp.config(state=tk.NORMAL)
            
            self.log(f"Starting Current Ramp: {self.psu_current_setpoint}A -> {target_i}A at {ramp_rate_A_min} A/min.")
        except ValueError:
            messagebox.showerror("Input Error", "Please ensure all limits and targets are valid numbers.")

    def stop_current_ramp(self):
        if self.ramping_active:
            self.ramping_active = False
            self.btn_stop_ramp.config(state=tk.DISABLED)
            self.log(f"Ramp manually stopped. Holding at {self.psu_current_setpoint:.3f} A.")

    def safe_heater_off(self):
        if not self.psu: return
        self.ramping_active = False
        self.btn_stop_ramp.config(state=tk.DISABLED)
        try:
            self.psu.write("VOLT 0.0")
            self.psu.write("CURR 0.0")
            self.psu.write("OUTP OFF")
            self.log("🛑 SAFE OFF TRIGGERED. Heater set to 0V / 0A and output disabled.")
        except Exception as e:
            self.log(f"Error during Safe Off: {e}")

    def process_ramping(self):
        if not self.ramping_active or not self.psu:
            return

        now = time.time()
        dt = now - self.last_ramp_time
        self.last_ramp_time = now
        delta_i = self.ramp_rate_A_per_sec * dt

        if self.psu_current_setpoint < self.psu_target_i:
            self.psu_current_setpoint = min(self.psu_current_setpoint + delta_i, self.psu_target_i)
        elif self.psu_current_setpoint > self.psu_target_i:
            self.psu_current_setpoint = max(self.psu_current_setpoint - delta_i, self.psu_target_i)

        if self.psu_current_setpoint > self.max_i_limit:
            self.psu_current_setpoint = self.max_i_limit
            self.ramping_active = False
            self.btn_stop_ramp.config(state=tk.DISABLED)
            self.log("Ramp stopped: Hit max current limit.")

        self.psu.write(f"CURR {self.psu_current_setpoint:.3f}")

        if math.isclose(self.psu_current_setpoint, self.psu_target_i, abs_tol=0.001):
            self.ramping_active = False
            self.btn_stop_ramp.config(state=tk.DISABLED)
            self.log(f"Ramp complete. Reached target {self.psu_target_i} A.")

    # ------------------- ACTUATOR METHODS -------------------
    def connect_actuator_thread(self):
        self.btn_act_conn.config(state=tk.DISABLED)
        self.lbl_act_status.config(text="Status: Connecting & Resetting...")
        threading.Thread(target=self._connect_actuator, daemon=True).start()

    def _connect_actuator(self):
        try:
            self.actuator = serial.Serial(ACTUATOR_PORT, BAUD_RATE, timeout=SERIAL_RESPONSE_TIMEOUT_SECONDS)
            self.log(f"Actuator port opened. Waiting {ACTUATOR_STARTUP_SECONDS}s for reset...")
            time.sleep(ACTUATOR_STARTUP_SECONDS)
            self.actuator.reset_input_buffer()
            
            def update_ui():
                self.btn_act_conn.config(text="Actuator Connected")
                self.lbl_act_status.config(text="Status: Ready")
                self.check_enable_test()
            self.root.after(0, update_ui)
            self.log("Actuator ready.")
        except Exception as e:
            self.log(f"Actuator connection failed: {e}")
            self.root.after(0, lambda: self.btn_act_conn.config(state=tk.NORMAL))

    # ------------------- DAQ METHODS -------------------
    def connect_daq_thread(self):
        self.btn_daq_conn.config(state=tk.DISABLED)
        self.lbl_daq_status.config(text="Status: Connecting...")
        threading.Thread(target=self._connect_daq, daemon=True).start()

    def _connect_daq(self):
        try:
            self.daq = self.rm.open_resource(DAQ_PORT)
            self.daq.timeout = VISA_TIMEOUT_MS
            self.daq.write_termination = "\n"
            self.daq.read_termination = "\n"
            
            identification = self.daq.query("*IDN?").strip()
            self.log(f"DAQ Connected: {identification}")
            self.configure_daq()

            def update_ui():
                self.btn_daq_conn.config(text="DAQ Connected")
                self.lbl_daq_status.config(text="Status: Configured")
                self.check_enable_test()
            self.root.after(0, update_ui)
        except Exception as e:
            self.log(f"DAQ Connection failed: {e}")
            self.root.after(0, lambda: self.btn_daq_conn.config(state=tk.NORMAL))

    def configure_daq(self):
        self.daq.write(":ABOR")
        self.daq.write("*RST")
        self.daq.query("*OPC?")
        self.daq.write("*CLS")

        terminal_position = self.daq.query(":ROUT:TERM?").strip()
        if "REAR" not in terminal_position.upper():
            raise RuntimeError("The DAQ6510 physical TERMINALS switch is not set to REAR.")

        self.daq.write(f':SENS:FUNC "VOLT:DC", {CHANNEL_LIST}')
        self.daq.write(f":SENS:VOLT:DC:RANG:AUTO ON, {CHANNEL_LIST}")
        self.daq.write(f":SENS:VOLT:DC:NPLC {NPLC}, {CHANNEL_LIST}")
        self.daq.write(f":ROUT:SCAN:CRE {CHANNEL_LIST}")
        self.daq.write(":ROUT:SCAN:COUNT:SCAN 1")
        self.daq.query("*OPC?")
        
        errors = self.get_instrument_errors()
        if errors:
            raise RuntimeError("DAQ configuration error(s): " + " | ".join(errors))

    def get_instrument_errors(self, maximum_errors=10):
        errors = []
        for _ in range(maximum_errors):
            error_message = self.daq.query(":SYST:ERR?").strip()
            if error_message.startswith("0,") or error_message.startswith("+0,") or "No error" in error_message:
                break
            errors.append(error_message)
        return errors

    def wait_for_scan(self):
        start_time = time.time()
        while True:
            trigger_state = self.daq.query(":TRIG:STAT?").strip().upper()
            if "IDLE" in trigger_state:
                return
            if time.time() - start_time > SCAN_TIMEOUT_SECONDS:
                self.daq.write(":ABOR")
                raise TimeoutError("DAQ6510 scan timed out.")
            time.sleep(0.02)

    def read_all_channels(self):
        self.daq.write(':TRAC:CLEAR "defbuffer1"')
        self.daq.write(":INIT")
        self.wait_for_scan()

        # Added REL parameter to ask for relative timestamps
        raw_data = self.daq.query_ascii_values(f':TRAC:DATA? 1, {len(CHANNELS)}, "defbuffer1", READ, REL')
        
        # We expect 2 values per channel (reading, time, reading, time...)
        if len(raw_data) != len(CHANNELS) * 2:
            raise RuntimeError(f"Expected {len(CHANNELS) * 2} data points, received {len(raw_data)}")
        
        readings = []
        timestamps = []
        for i in range(0, len(raw_data), 2):
            readings.append(float(raw_data[i]))
            timestamps.append(float(raw_data[i+1]))
            
        return readings, timestamps

    def voltage_to_temperature(self, channel, voltage):
        cal = TEMPERATURE_CALIBRATION.get(channel)
        if not cal: return 0.0
        return voltage * cal["gain"] + cal["offset"]

    # ------------------- LOGGING & TEST CONTROL -------------------
    def check_enable_test(self):
        if self.actuator or self.daq:
            self.btn_start_test.config(state=tk.NORMAL)

    def start_test(self):
        self.test_running = True
        self.btn_start_test.config(state=tk.DISABLED)
        self.btn_stop_test.config(state=tk.NORMAL)
        
        self.t_data.clear()
        self.v_data.clear()
        self.i_data.clear()
        self.pos_data.clear()
        for ch in CHANNELS:
            self.temp_data[ch].clear()
            
        self.start_time = time.time()
        
        filename = f"daq_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.csv_file = open(filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        # Updated Headers to include HW timestamps
        headers = [
            "PC_Timestamp", "PC_Elapsed_s", 
            "PSU_Voltage_V", "PSU_Current_A", "Heating_Rate_Ks", 
            "Actuator_Position_mm", "Actuator_HW_Time_ms"
        ]
        for ch in CHANNELS:
            headers.extend([f"CH{ch}_Voltage_V", f"CH{ch}_Temp_C", f"CH{ch}_HW_Time_s"])
            
        self.csv_writer.writerow(headers)
        self.log(f"Started synchronized logging to {filename}")

    def stop_test(self):
        self.test_running = False
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
        self.btn_start_test.config(state=tk.NORMAL)
        self.btn_stop_test.config(state=tk.DISABLED)
        self.log("Data logging stopped.")

    def update_plots(self):
        if not self.t_data: return
        self.line_v.set_data(self.t_data, self.v_data)
        self.line_i.set_data(self.t_data, self.i_data)
        self.line_pos.set_data(self.t_data, self.pos_data)
        
        for ch in CHANNELS:
            self.line_temps[ch].set_data(self.t_data, self.temp_data[ch])

        self.ax_pwr_v.relim()
        self.ax_pwr_v.autoscale_view()
        self.ax_pwr_i.relim()
        self.ax_pwr_i.autoscale_view()
        
        self.ax_pos.relim()
        self.ax_pos.autoscale_view()

        self.ax_temp.relim()
        self.ax_temp.autoscale_view()

        self.canvas.draw()

    # ------------------- CORE POLLING LOOP -------------------
    def hardware_poll_loop(self):
        while self.polling_active:
            loop_start = time.time()
            self.process_ramping()

            current_time_str = datetime.now().isoformat()
            elapsed = loop_start - self.start_time if (self.test_running and self.start_time) else 0.0
            
            # Initialize empty variables to handle missed connections gracefully
            v_meas, i_meas, heat_rate, pos_mm, act_time_ms = None, None, None, None, None
            daq_voltages = [None] * len(CHANNELS)
            daq_temps = [None] * len(CHANNELS)
            daq_times = [None] * len(CHANNELS)

            # 1. Query PSU
            if self.psu:
                try:
                    v_meas = float(self.psu.query("MEAS:VOLT?").strip())
                    i_meas = float(self.psu.query("MEAS:CURR?").strip())
                    power_watts = v_meas * i_meas
                    heat_rate = power_watts / (CRUCIBLE_MASS_KG * CRUCIBLE_CP)
                    
                    self.root.after(0, lambda v=v_meas, i=i_meas, hr=heat_rate: [
                        self.lbl_v_meas.config(text=f"Meas V: {v:.3f} V"),
                        self.lbl_i_meas.config(text=f"Meas I: {i:.3f} A"),
                        self.lbl_heat_rate.config(text=f"Heating Rate: {hr:.2f} K/s")
                    ])
                except Exception:
                    pass

            # 2. Query Actuator
            if self.actuator and self.test_running:
                try:
                    self.actuator.reset_input_buffer()
                    self.actuator.write(b"P\n")
                    self.actuator.flush()
                    raw = self.actuator.readline()
                    text = raw.decode("ascii", errors="replace").strip()
                    
                    # Split string to see if Arduino passed a timestamp e.g., "10.50,45032"
                    parts = text.split(",")
                    pos_mm = float(parts[0].replace(",", "."))
                    act_time_ms = int(parts[1]) if len(parts) > 1 else None
                    
                    self.root.after(0, lambda p=pos_mm: self.lbl_pos.config(text=f"Pos: {p:.2f} mm"))
                except Exception:
                    pass

            # 3. Query DAQ
            if self.daq and self.test_running:
                try:
                    # Unpack both voltages and hardware timestamps
                    daq_voltages, daq_times = self.read_all_channels()
                    daq_temps = [self.voltage_to_temperature(ch, v) for ch, v in zip(CHANNELS, daq_voltages)]
                    
                    temp_str = " | ".join([f"CH{ch}: {t:.1f}°C" for ch, t in zip(CHANNELS, daq_temps)])
                    self.root.after(0, lambda ts=temp_str: self.lbl_daq_temps.config(text=ts))
                except Exception:
                    pass

            # 4. Log & Plot
            if self.test_running:
                if self.csv_writer:
                    row = [
                        current_time_str, 
                        round(elapsed, 3), 
                        v_meas if v_meas is not None else "", 
                        i_meas if i_meas is not None else "", 
                        round(heat_rate, 3) if heat_rate is not None else "",
                        pos_mm if pos_mm is not None else "",
                        act_time_ms if act_time_ms is not None else ""
                    ]
                    
                    # Interleave DAQ voltage, temp, and hardware timestamp
                    for v, t, hw_time in zip(daq_voltages, daq_temps, daq_times):
                        row.extend([
                            v if v is not None else "", 
                            t if t is not None else "",
                            hw_time if hw_time is not None else ""
                        ])
                    
                    self.csv_writer.writerow(row)
                    self.csv_file.flush()

                # Always plot against the PC `elapsed` time for uniform X-axis alignment
                self.t_data.append(elapsed)
                self.v_data.append(v_meas if v_meas is not None else float('nan'))
                self.i_data.append(i_meas if i_meas is not None else float('nan'))
                self.pos_data.append(pos_mm if pos_mm is not None else float('nan'))
                
                for i, ch in enumerate(CHANNELS):
                    self.temp_data[ch].append(daq_temps[i] if daq_temps[i] is not None else float('nan'))

                self.root.after(0, self.update_plots)

            # 5. Enforce Exact Polling Frequency
            sleep_time = POLL_INTERVAL_SECONDS - (time.time() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def on_closing(self):
        self.polling_active = False
        if self.test_running:
            self.stop_test()
        
        if self.psu:
            try:
                self.psu.write("OUTP OFF")
                self.psu.write("SYST:REM LOC")
                self.psu.close()
            except: pass
        
        if self.actuator:
            try: self.actuator.close() 
            except: pass

        if self.daq:
            try:
                self.daq.write(":ABOR")
                self.daq.close()
            except: pass
            
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = UnifiedHardwareApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
