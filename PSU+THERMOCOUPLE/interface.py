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
HARDWARE_OVP = 10.5         # Safe OVP ceiling for 0-10V 1000A TDK-Lambda GSP
PSU_BAUD_RATE = 115200
DEFAULT_POLL_INTERVAL = 1.0

# ============================================================
# DAQ6510 CONSTANTS & CALIBRATION
# ============================================================
CHANNELS = [101, 102, 103, 104, 105]
CHANNEL_LIST = "(@101:105)"
NPLC = 1.0
VISA_TIMEOUT_MS = 5000

# temperature = gain * measured Voltage (mV) + offset
TEMPERATURE_CALIBRATION = {
    101: {"gain": 84.33, "offset": 10.78},
    102: {"gain": 84.33, "offset": 10.78},
    103: {"gain": 84.33, "offset": 10.78},
    104: {"gain": 84.33, "offset": 10.78},
    105: {"gain": 84.33, "offset": 10.78}, # crucible thermocouple
}


class ThermalPIDController:
    """Discrete PI controller with EMA filtering, integral clamping, and current slew-rate limiting."""

    def __init__(self, kp=2.0, ki=0.05, kd=0.0, ema_alpha=0.3):
        self.kp = kp
        self.ki = ki
        self.kd = kd  # Kept at 0.0 per design
        self.ema_alpha = ema_alpha

        self.filtered_temp = None
        self.integral_power = 0.0
        self.last_time = None

    def reset(self, initial_power=0.0, initial_temp=None):
        self.filtered_temp = initial_temp
        self.integral_power = max(0.0, initial_power)
        self.last_time = time.time()

    def filter_temperature(self, raw_temp):
        if self.filtered_temp is None:
            self.filtered_temp = raw_temp
        else:
            self.filtered_temp = (
                self.ema_alpha * raw_temp + (1.0 - self.ema_alpha) * self.filtered_temp
            )
        return self.filtered_temp

    def compute(
        self,
        raw_temp,
        setpoint_temp,
        v_meas,
        i_meas,
        v_target,
        current_i_setpoint,
        max_v_limit,
        max_i_limit,
        max_rate_a_per_sec,
        pre_filtered_temp=None,
    ):
        now = time.time()
        dt = now - self.last_time if self.last_time is not None else 1.0
        dt = max(dt, 1e-3)
        self.last_time = now

        # 1. EMA Noise Filtering
        if pre_filtered_temp is not None:
            self.filtered_temp = pre_filtered_temp
            t_filt = pre_filtered_temp
        else:
            t_filt = self.filter_temperature(raw_temp)
        error = setpoint_temp - t_filt

        # 2. Proportional Term (Watts)
        p_term = self.kp * error

        # 3. Integral Term with Clamping (Anti-Windup)
        max_power = max(0.0, max_v_limit * max_i_limit)
        self.integral_power += self.ki * error * dt
        self.integral_power = max(0.0, min(self.integral_power, max_power))

        # 4. Total Commanded Power (D = 0)
        cmd_power = max(0.0, min(p_term + self.integral_power, max_power))

        # 5. Convert Power (W) -> Current (A)
        if v_meas is not None and i_meas is not None and v_meas > 0.05 and i_meas > 0.05:
            # Estimate live furnace resistance R = V / I -> I = sqrt(P / R)
            r_load = v_meas / i_meas
            desired_current = math.sqrt(cmd_power / r_load)
        else:
            # Startup fallback before significant current flows
            v_ref = v_meas if (v_meas is not None and v_meas > 0.1) else max(v_target, 0.1)
            desired_current = cmd_power / v_ref

        # Clamp to user's maximum allowable current
        desired_current = max(0.0, min(desired_current, max_i_limit))

        # 6. Maximum Change Rate (Slew-Rate Limiter)
        max_delta_i = max_rate_a_per_sec * dt
        delta_i = desired_current - current_i_setpoint
        if abs(delta_i) > max_delta_i:
            applied_current = current_i_setpoint + math.copysign(max_delta_i, delta_i)
        else:
            applied_current = desired_current

        applied_current = max(0.0, min(applied_current, max_i_limit))
        return applied_current, cmd_power, t_filt


class UnifiedHardwareApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Advanced Hardware Controller & DAQ (with PI Thermal Hold)")
        self.root.geometry("1480x850")

        # INDIVIDUAL HARDWARE LOCKS
        self.psu_lock = threading.Lock()
        self.daq_lock = threading.Lock()

        # VISA BACKEND FOR USBTMC DAQ6510
        self.rm = pyvisa.ResourceManager()

        self.psu = None           # serial.Serial instance (TDK-Lambda GSP)
        self.psu_protocol = "SCPI"
        self.daq = None           # pyvisa Resource (Keithley DAQ6510)

        self.test_running = False
        self.csv_file = None
        self.csv_writer = None
        self.loop_iteration = 0   # Absolute timeline tracker
        self.poll_interval = DEFAULT_POLL_INTERVAL

        # User-defined safety ceilings
        self.max_v_limit = 10.0
        self.max_i_limit = 3.0

        # Manual Ramping States
        self.ramping_active = False
        self.psu_target_i = 0.0
        self.psu_current_setpoint = 0.0
        self.ramp_rate_A_per_sec = 0.0
        self.last_ramp_time = 0.0

        # PID Thermal Hold States
        self.pid_active = False
        self.pid_controller = ThermalPIDController(kp=2.0, ki=0.05, kd=0.0, ema_alpha=0.3)
        self.pid_channel = 105
        self.pid_target_temp = 100.0
        self.pid_max_rate_A_per_sec = 0.5 / 60.0
        self.last_pid_power_w = 0.0
        self.last_pid_filt_temp = None

        # Continuous EMA state across all thermocouple channels
        self.ema_temps = {ch: None for ch in CHANNELS}

        # Log history
        self.log_history = []

        # Port lists
        self.available_psu_com = []
        self.available_usb_visa = []

        # Plot Data Arrays
        self.t_data = []
        self.v_data = []
        self.i_data = []
        self.temp_data = {ch: [] for ch in CHANNELS}
        self.ema_temp_data = {ch: [] for ch in CHANNELS}

        self.temp_channel_visible = {
            ch: tk.BooleanVar(value=True) for ch in CHANNELS
        }
        self.start_time = None

        self.scan_ports()
        self.setup_ui()

        # Start unified polling thread
        self.polling_active = True
        self.poll_thread = threading.Thread(target=self.hardware_poll_loop, daemon=True)
        self.poll_thread.start()

    def scan_ports(self):
        try:
            resources = self.rm.list_resources()
            self.available_usb_visa = [r for r in resources if "USB" in r or "TCPIP" in r]
            print(f"DEBUG VISA Resources: {resources}")
        except Exception as e:
            print(f"VISA Scan Error: {e}")
            self.available_usb_visa = []

        try:
            com_ports = serial.tools.list_ports.comports()
            self.available_psu_com = []
            for p in com_ports:
                if "2504:0002" in p.hwid or p.device == "COM4":
                    self.available_psu_com.insert(0, p.device)
                else:
                    self.available_psu_com.append(p.device)

            if not self.available_psu_com:
                self.available_psu_com = ["COM4"]
        except Exception as e:
            print(f"COM Port Scan Error: {e}")
            self.available_psu_com = ["COM4"]

    def setup_ui(self):
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        logging_frame = ttk.LabelFrame(main_frame, text="Synchronized Data Logging", padding=8)
        logging_frame.pack(fill=tk.X, pady=(0, 8))
        logging_frame.columnconfigure(1, weight=1)
        logging_frame.columnconfigure(2, weight=1)
        logging_frame.columnconfigure(3, weight=1)

        ttk.Label(logging_frame, text="Poll Interval (s):").grid(row=0, column=0, sticky="w", padx=(0, 6))
        self.entry_poll_interval = ttk.Entry(logging_frame, width=10)
        self.entry_poll_interval.grid(row=0, column=1, sticky="w", padx=(0, 12))
        self.entry_poll_interval.insert(0, "1.0")

        self.btn_start_test = ttk.Button(logging_frame, text="Start Log", command=self.start_test, state=tk.DISABLED)
        self.btn_start_test.grid(row=0, column=2, sticky="ew", padx=2)
        self.btn_stop_test = ttk.Button(logging_frame, text="Stop Log", command=self.stop_test, state=tk.DISABLED)
        self.btn_stop_test.grid(row=0, column=3, sticky="ew", padx=2)

        control_frame = ttk.Frame(main_frame)
        control_frame.pack(fill=tk.X, pady=(0, 10))

        # --- 1. PSU CONTROLS ---
        psu_frame = ttk.LabelFrame(control_frame, text="TDK-Lambda Power Supply (GSP 10-1000)", padding=10)
        psu_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))

        ttk.Label(psu_frame, text="Port:").grid(row=0, column=0, sticky="w", padx=2)
        self.combo_psu_port = ttk.Combobox(psu_frame, values=self.available_psu_com, width=15)
        self.combo_psu_port.set(self.available_psu_com[0])
        self.combo_psu_port.grid(row=0, column=1, padx=5, pady=5, sticky="ew")

        self.btn_psu_conn = ttk.Button(psu_frame, text="Connect PSU", command=self.connect_psu_thread)
        self.btn_psu_conn.grid(row=0, column=2, columnspan=2, pady=5, sticky="ew")

        ttk.Label(psu_frame, text="Max Allowable V:").grid(row=1, column=0, sticky="w")
        self.entry_max_v = ttk.Entry(psu_frame, width=8)
        self.entry_max_v.grid(row=1, column=1, padx=5, pady=2)
        self.entry_max_v.insert(0, "10.0")

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

        # --- 2. PID THERMAL HOLD CONTROLS ---
        pid_frame = ttk.LabelFrame(control_frame, text="PID Temperature Hold (D = 0)", padding=10)
        pid_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=5)
        for col in range(4):
            pid_frame.columnconfigure(col, weight=1)

        ttk.Label(pid_frame, text="Input Channel:").grid(row=0, column=0, sticky="w", pady=2)
        self.combo_pid_ch = ttk.Combobox(pid_frame, values=[str(ch) for ch in CHANNELS], state="readonly", width=8)
        self.combo_pid_ch.set("105")
        self.combo_pid_ch.grid(row=0, column=1, sticky="ew", padx=4, pady=2)

        ttk.Label(pid_frame, text="Target Temp (°C):").grid(row=0, column=2, sticky="w", pady=2)
        self.entry_pid_target = ttk.Entry(pid_frame, width=8)
        self.entry_pid_target.grid(row=0, column=3, sticky="ew", padx=4, pady=2)
        self.entry_pid_target.insert(0, "100.0")

        ttk.Label(pid_frame, text="P Gain (Kp):").grid(row=1, column=0, sticky="w", pady=2)
        self.entry_pid_kp = ttk.Entry(pid_frame, width=8)
        self.entry_pid_kp.grid(row=1, column=1, sticky="ew", padx=4, pady=2)
        self.entry_pid_kp.insert(0, "2.0")

        ttk.Label(pid_frame, text="I Gain (Ki):").grid(row=1, column=2, sticky="w", pady=2)
        self.entry_pid_ki = ttk.Entry(pid_frame, width=8)
        self.entry_pid_ki.grid(row=1, column=3, sticky="ew", padx=4, pady=2)
        self.entry_pid_ki.insert(0, "0.05")

        ttk.Label(pid_frame, text="EMA Alpha (0-1):").grid(row=2, column=0, sticky="w", pady=2)
        self.entry_pid_alpha = ttk.Entry(pid_frame, width=8)
        self.entry_pid_alpha.grid(row=2, column=1, sticky="ew", padx=4, pady=2)
        self.entry_pid_alpha.insert(0, "0.3")

        ttk.Label(pid_frame, text="Max Rate (A/min):").grid(row=2, column=2, sticky="w", pady=2)
        self.entry_pid_max_rate = ttk.Entry(pid_frame, width=8)
        self.entry_pid_max_rate.grid(row=2, column=3, sticky="ew", padx=4, pady=2)
        self.entry_pid_max_rate.insert(0, "1.0")

        self.btn_update_pid = ttk.Button(pid_frame, text="Update Live Gains", command=self.update_pid_parameters)
        self.btn_update_pid.grid(row=3, column=0, columnspan=4, sticky="ew", pady=(6, 4))

        self.btn_pid_hold = ttk.Button(pid_frame, text="PID Hold", command=self.start_pid_hold, state=tk.DISABLED)
        self.btn_pid_hold.grid(row=4, column=0, columnspan=2, sticky="ew", padx=(0, 2), pady=4)

        self.btn_pid_stop = ttk.Button(pid_frame, text="Stop PID", command=self.stop_pid_hold, state=tk.DISABLED)
        self.btn_pid_stop.grid(row=4, column=2, columnspan=2, sticky="ew", padx=(2, 0), pady=4)

        self.lbl_pid_status = ttk.Label(
            pid_frame,
            text="PID: Idle | EMA: -- °C | Out: -- W (-- A)",
            font=("Arial", 9, "bold"),
            foreground="blue",
        )
        self.lbl_pid_status.grid(row=5, column=0, columnspan=4, pady=(4, 0))

        # --- 3. DAQ CONTROLS ---
        daq_frame = ttk.LabelFrame(control_frame, text="Keithley DAQ6510", padding=10)
        daq_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(5, 0))

        self.combo_daq_port = ttk.Combobox(daq_frame, values=self.available_usb_visa)
        if self.available_usb_visa:
            self.combo_daq_port.set(self.available_usb_visa[0])
        self.combo_daq_port.pack(fill=tk.X, pady=5)

        self.btn_daq_conn = ttk.Button(daq_frame, text="Connect DAQ", command=self.connect_daq_thread)
        self.btn_daq_conn.pack(fill=tk.X, pady=5)

        self.lbl_daq_status = ttk.Label(daq_frame, text="Status: Disconnected")
        self.lbl_daq_status.pack(pady=5)

        self.lbl_daq_temps = ttk.Label(daq_frame, text="Temps: -- °C", font=("Arial", 10), wraplength=240)
        self.lbl_daq_temps.pack(pady=(6, 5))

        channel_filter = ttk.LabelFrame(daq_frame, text="Graph Channels (Raw & EMA)", padding=5)
        channel_filter.pack(fill=tk.X, pady=(3, 5))

        for col in range(3):
            channel_filter.columnconfigure(col, weight=1)

        for index, ch in enumerate(CHANNELS):
            ttk.Checkbutton(
                channel_filter,
                text=f"CH {ch}",
                variable=self.temp_channel_visible[ch],
                command=self._on_temp_channel_visibility_changed,
            ).grid(row=index // 3, column=index % 3, sticky="w", padx=3, pady=2)

        channel_buttons = ttk.Frame(channel_filter)
        channel_buttons.grid(row=2, column=0, columnspan=3, sticky="ew", pady=(4, 0))
        channel_buttons.columnconfigure(0, weight=1)
        channel_buttons.columnconfigure(1, weight=1)
        ttk.Button(
            channel_buttons, text="Select All", command=lambda: self._set_all_temp_channels(True)
        ).grid(row=0, column=0, sticky="ew", padx=(0, 2))
        ttk.Button(
            channel_buttons, text="Clear All", command=lambda: self._set_all_temp_channels(False)
        ).grid(row=0, column=1, sticky="ew", padx=(2, 0))

        # ==================== BOTTOM ROW: SYSTEM LOG ====================
        log_frame = ttk.LabelFrame(main_frame, text="System Log", padding=5)
        log_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=(10, 0))

        self.log_text = tk.Text(log_frame, height=6, state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True)

        # ==================== MIDDLE ROW: GRAPHS ====================
        graph_frame = ttk.Frame(main_frame)
        graph_frame.pack(fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(13, 4), dpi=100)
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

        # Subplot 2: Raw DAQ Temperatures
        self.ax_temp = self.fig.add_subplot(132)
        self.ax_temp.set_title("Thermocouples (Raw)")
        self.ax_temp.set_xlabel("Time (s)")
        self.ax_temp.set_ylabel("Temperature (°C)")

        self.line_temps = {}
        for ch in CHANNELS:
            self.line_temps[ch], = self.ax_temp.plot([], [], label=f"CH {ch}")
        self.ax_temp.legend(fontsize=8, loc='upper left')

        # Subplot 3: EMA Filtered Thermocouples
        self.ax_ema_temp = self.fig.add_subplot(133)
        self.ax_ema_temp.set_title("Thermocouples (EMA Filtered)")
        self.ax_ema_temp.set_xlabel("Time (s)")
        self.ax_ema_temp.set_ylabel("EMA Temp (°C)")

        self.line_ema_temps = {}
        for ch in CHANNELS:
            self.line_ema_temps[ch], = self.ax_ema_temp.plot([], [], label=f"CH {ch} EMA")
        self.ax_ema_temp.legend(fontsize=8, loc='upper left')

        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def log(self, message):
        line = f"[{time.strftime('%H:%M:%S')}] {message}\n"
        self.log_history.append(line)
        if len(self.log_history) > 1000:
            self.log_history = self.log_history[-1000:]

        def append():
            self._append_to_text_widget(self.log_text, line)

        self.root.after(0, append)

    @staticmethod
    def _append_to_text_widget(widget, line):
        try:
            widget.config(state=tk.NORMAL)
            widget.insert(tk.END, line)
            widget.see(tk.END)
            widget.config(state=tk.DISABLED)
        except tk.TclError:
            pass

    # ------------------- PSU PYSERIAL METHODS -------------------
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

    def check_psu_Command(self, ser_inst, cmd):
        ser_inst.reset_input_buffer()
        self._psu_write(ser_inst, cmd)
        time.sleep(0.08)

        if ser_inst.in_waiting > 0:
            reply = ser_inst.read(ser_inst.in_waiting).decode("ascii", errors="replace").strip()
            self.log(f"PSU reply to '{cmd}': {reply}")
            return

        if self.psu_protocol == "SCPI":
            err = self._psu_query(ser_inst, "SYST:ERR?")
            if err and not err.startswith(("0,", "+0,")):
                self.log(f"PSU SCPI Error on '{cmd}': {err}")
            else:
                self.log(f"'{cmd}' -> OK ({err or 'No error'})")

    def connect_psu_thread(self):
        selected_port = self.combo_psu_port.get().strip()
        if not selected_port:
            messagebox.showerror("Error", "Please select or type a COM port for the PSU (e.g., COM4).")
            return

        if selected_port.upper().startswith("ASRL"):
            num = "".join(filter(str.isdigit, selected_port))
            selected_port = f"COM{num}"

        self.btn_psu_conn.config(state=tk.DISABLED)
        self.combo_psu_port.config(state=tk.DISABLED)
        self.log(f"Attempting to connect to PSU on {selected_port} via PySerial...")

        threading.Thread(target=self._connect_psu, args=(selected_port,), daemon=True).start()

    def _connect_psu(self, port):
        temp_ser = None
        try:
            with self.psu_lock:
                temp_ser = serial.Serial(
                    port=port,
                    baudrate=PSU_BAUD_RATE,
                    bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE,
                    stopbits=serial.STOPBITS_ONE,
                    timeout=0.6,
                    write_timeout=1.0,
                    rtscts=False,
                    dsrdtr=False,
                )
                temp_ser.dtr = True
                temp_ser.rts = True
                time.sleep(0.15)
                temp_ser.reset_input_buffer()
                temp_ser.reset_output_buffer()

                connected_idn = None
                self.psu_protocol = None

                for addr in [6, 1, 0]:
                    self._psu_write(temp_ser, f"INST:NSEL {addr}")
                    time.sleep(0.1)
                    temp_ser.reset_input_buffer()
                    idn = self._psu_query(temp_ser, "*IDN?", wait_s=0.15)
                    if idn:
                        connected_idn = idn
                        self.psu_protocol = "SCPI"
                        self.log(f"✅ PSU Connected (SCPI Addr {addr}): {connected_idn}")
                        break

                    resp = self._psu_query(temp_ser, f"ADR {addr:02d}", wait_s=0.15)
                    if "OK" in resp:
                        connected_idn = self._psu_query(temp_ser, "IDN?", wait_s=0.15)
                        self.psu_protocol = "GEN"
                        self.log(f"✅ PSU Connected (GEN Addr {addr:02d}): {connected_idn}")
                        break

                if not connected_idn or not self.psu_protocol:
                    raise RuntimeError(
                        f"No response from PSU on {port}. "
                        "Verify Front Panel COMM -> IntF is set to 'USB' and Addr=6."
                    )

                if self.psu_protocol == "SCPI":
                    for cmd in [
                        "SYST:SET REM",
                        f"VOLT:PROT:LEV {HARDWARE_OVP}",
                        "VOLT 0.0",
                        "CURR 0.0",
                        "OUTP ON",
                    ]:
                        self.check_psu_Command(temp_ser, cmd)
                else:
                    for cmd in [
                        "RMT 1",
                        f"OVP {HARDWARE_OVP}",
                        "PV 0.0",
                        "PC 0.0",
                        "OUT 1",
                    ]:
                        self.check_psu_Command(temp_ser, cmd)

                self.psu = temp_ser

            def update_ui():
                self.btn_psu_conn.config(text="PSU Connected", state=tk.DISABLED)
                self.btn_apply.config(state=tk.NORMAL)
                self.btn_safe_off.config(state=tk.NORMAL)
                self.check_enable_test()
                self.log(f"PSU active in REM mode ({self.psu_protocol}). Output enabled at 0V / 0A.")

            self.root.after(0, update_ui)

        except Exception as e:
            self.log(f"PSU Connection failed: {e}")
            if temp_ser and temp_ser.is_open:
                try:
                    temp_ser.close()
                except Exception:
                    pass
            self.root.after(0, lambda: [
                self.btn_psu_conn.config(state=tk.NORMAL),
                self.combo_psu_port.config(state=tk.NORMAL),
            ])

    def _write_psu_current(self, current_val):
        with self.psu_lock:
            if self.psu_protocol == "SCPI":
                self._psu_write(self.psu, f"CURR {current_val:.3f}")
            else:
                self._psu_query(self.psu, f"PC {current_val:.3f}")

    def start_current_ramp(self):
        if not self.psu:
            return
        if self.pid_active:
            self.stop_pid_hold()
        try:
            self.max_v_limit = float(self.entry_max_v.get())
            self.max_i_limit = float(self.entry_max_i.get())
            target_v = float(self.entry_target_v.get())
            target_i = float(self.entry_target_i.get())
            ramp_rate_A_min = float(self.entry_ramp_rate.get())

            if target_v > self.max_v_limit or target_i > self.max_i_limit:
                messagebox.showerror(
                    "Limit Error",
                    f"Targets exceed maximum stored limits ({self.max_v_limit}V, {self.max_i_limit}A).",
                )
                return

            with self.psu_lock:
                if self.psu_protocol == "SCPI":
                    self.check_psu_Command(self.psu, f"VOLT {target_v}")
                    current_setpoint_str = self._psu_query(self.psu, "CURR?")
                else:
                    self.check_psu_Command(self.psu, f"PV {target_v}")
                    current_setpoint_str = self._psu_query(self.psu, "PC?")

            self.psu_current_setpoint = float(current_setpoint_str)

            self.psu_target_i = target_i
            self.ramp_rate_A_per_sec = ramp_rate_A_min / 60.0
            self.last_ramp_time = time.time()
            self.ramping_active = True
            self.btn_stop_ramp.config(state=tk.NORMAL)

            self.log(f"Starting Current Ramp: {self.psu_current_setpoint}A -> {target_i}A at {ramp_rate_A_min} A/min.")
        except ValueError:
            messagebox.showerror("Input Error", "Please ensure all limits and targets are valid numbers.")
        except Exception as e:
            self.log(f"Error starting ramp: {e}")

    def stop_current_ramp(self):
        if self.ramping_active:
            self.ramping_active = False
            self.btn_stop_ramp.config(state=tk.DISABLED)
            self.log(f"Ramp manually stopped. Holding at {self.psu_current_setpoint:.3f} A.")

    def safe_heater_off(self):
        if not self.psu:
            return
        self.ramping_active = False
        self.btn_stop_ramp.config(state=tk.DISABLED)
        if self.pid_active:
            self.stop_pid_hold()
        try:
            with self.psu_lock:
                if self.psu_protocol == "SCPI":
                    self.check_psu_Command(self.psu, "VOLT 0.0")
                    self.check_psu_Command(self.psu, "CURR 0.0")
                    self.check_psu_Command(self.psu, "OUTP OFF")
                else:
                    self.check_psu_Command(self.psu, "PV 0.0")
                    self.check_psu_Command(self.psu, "PC 0.0")
                    self.check_psu_Command(self.psu, "OUT 0")
            self.psu_current_setpoint = 0.0
            self.log("🛑 SAFE OFF TRIGGERED. Heater set to 0V / 0A and output disabled.")
        except Exception as e:
            self.log(f"Error during Safe Off: {e}")

    def process_ramping(self):
        if not self.ramping_active or not self.psu or self.pid_active:
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
            self.root.after(0, lambda: self.btn_stop_ramp.config(state=tk.DISABLED))
            self.log("Ramp stopped: Hit max current limit.")

        try:
            self._write_psu_current(self.psu_current_setpoint)
        except Exception as e:
            self.log(f"Ramp write error: {e}")

        if math.isclose(self.psu_current_setpoint, self.psu_target_i, abs_tol=0.001):
            self.ramping_active = False
            self.root.after(0, lambda: self.btn_stop_ramp.config(state=tk.DISABLED))
            self.log(f"Ramp complete. Reached target {self.psu_target_i} A.")

    # ------------------- PID THERMAL HOLD METHODS -------------------
    def update_pid_parameters(self):
        try:
            kp = float(self.entry_pid_kp.get())
            ki = float(self.entry_pid_ki.get())
            alpha = float(self.entry_pid_alpha.get())
            max_rate_a_min = float(self.entry_pid_max_rate.get())
            target_temp = float(self.entry_pid_target.get())
            ch = int(self.combo_pid_ch.get())

            if kp < 0 or ki < 0 or max_rate_a_min <= 0 or not (0.0 < alpha <= 1.0):
                raise ValueError("Kp/Ki >= 0, Max Rate > 0, and EMA Alpha must be in (0, 1].")

            self.pid_controller.kp = kp
            self.pid_controller.ki = ki
            self.pid_controller.kd = 0.0
            self.pid_controller.ema_alpha = alpha
            self.pid_max_rate_A_per_sec = max_rate_a_min / 60.0
            self.pid_target_temp = target_temp
            self.pid_channel = ch

            self.log(
                f"PID updated live -> CH{ch}, Target={target_temp:.1f}°C, "
                f"Kp={kp}, Ki={ki}, EMA_alpha={alpha}, MaxRate={max_rate_a_min} A/min"
            )
            return True
        except ValueError as e:
            messagebox.showerror("PID Parameter Error", f"Invalid PID setting: {e}")
            return False

    def start_pid_hold(self):
        if not self.psu or not self.daq:
            messagebox.showerror("Hardware Missing", "Both PSU and DAQ6510 must be connected for PID Hold.")
            return

        if not self.update_pid_parameters():
            return

        # Stop manual ramp if active so PID takes sole control of current
        if self.ramping_active:
            self.ramping_active = False
            self.btn_stop_ramp.config(state=tk.DISABLED)

        try:
            self.max_v_limit = float(self.entry_max_v.get())
            self.max_i_limit = float(self.entry_max_i.get())
            target_v = float(self.entry_target_v.get())

            if target_v <= 0.0 or target_v > self.max_v_limit:
                messagebox.showerror(
                    "Voltage Ceiling Error",
                    f"Set a valid Target Voltage (0 < V <= {self.max_v_limit}V) in the PSU panel before starting PID Hold.",
                )
                return

            with self.psu_lock:
                if self.psu_protocol == "SCPI":
                    self.check_psu_Command(self.psu, f"VOLT {target_v}")
                    curr_str = self._psu_query(self.psu, "CURR?")
                    volt_str = self._psu_query(self.psu, "MEAS:VOLT?")
                else:
                    self.check_psu_Command(self.psu, f"PV {target_v}")
                    curr_str = self._psu_query(self.psu, "PC?")
                    volt_str = self._psu_query(self.psu, "MV?")

            self.psu_current_setpoint = float(curr_str)
            meas_v = float(volt_str)
            initial_power = max(0.0, meas_v * self.psu_current_setpoint)

            # Bumpless initialization seeded with current power & existing EMA temperature
            current_ema = self.ema_temps.get(self.pid_channel)
            self.pid_controller.reset(initial_power=initial_power, initial_temp=current_ema)
            self.pid_active = True

            self.btn_pid_hold.config(state=tk.DISABLED)
            self.btn_pid_stop.config(state=tk.NORMAL)
            self.log(
                f"🔥 PID Hold ENGAGED on CH{self.pid_channel} -> Target: {self.pid_target_temp:.1f}°C "
                f"(Bumpless start at {initial_power:.1f}W / {self.psu_current_setpoint:.3f}A)"
            )
        except Exception as e:
            self.log(f"Failed to start PID Hold: {e}")

    def stop_pid_hold(self):
        if self.pid_active:
            self.pid_active = False
            self.btn_pid_hold.config(state=tk.NORMAL if (self.psu and self.daq) else tk.DISABLED)
            self.btn_pid_stop.config(state=tk.DISABLED)
            self.lbl_pid_status.config(
                text=f"PID: Stopped (Holding {self.psu_current_setpoint:.3f} A)"
            )
            self.log(f"PID Hold stopped. Output held at {self.psu_current_setpoint:.3f} A.")

    # ------------------- DAQ METHODS -------------------
    def connect_daq_thread(self):
        selected_port = self.combo_daq_port.get().strip()
        if not selected_port:
            messagebox.showerror("Error", "Please select a VISA port for the DAQ.")
            return

        self.btn_daq_conn.config(state=tk.DISABLED)
        self.combo_daq_port.config(state=tk.DISABLED)
        self.lbl_daq_status.config(text="Status: Connecting...")
        threading.Thread(target=self._connect_daq, args=(selected_port,), daemon=True).start()

    def _connect_daq(self, port):
        try:
            with self.daq_lock:
                self.daq = self.rm.open_resource(port)
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
            self.root.after(0, lambda: [
                self.btn_daq_conn.config(state=tk.NORMAL),
                self.combo_daq_port.config(state=tk.NORMAL),
            ])

    def configure_daq(self):
        with self.daq_lock:
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
        with self.daq_lock:
            for _ in range(maximum_errors):
                error_message = self.daq.query(":SYST:ERR?").strip()
                if error_message.startswith("0,") or error_message.startswith("+0,") or "No error" in error_message:
                    break
                errors.append(error_message)
        return errors

    def read_all_channels(self):
        with self.daq_lock:
            self.daq.write(':TRAC:CLEAR "defbuffer1"')
            self.daq.write(":INIT")
            self.daq.query("*OPC?")

            raw_data = self.daq.query_ascii_values(f':TRAC:DATA? 1, {len(CHANNELS)}, "defbuffer1", READ, REL')

        if len(raw_data) != len(CHANNELS) * 2:
            raise RuntimeError(f"Expected {len(CHANNELS) * 2} data points, received {len(raw_data)}")

        readings = []
        timestamps = []
        for i in range(0, len(raw_data), 2):
            readings.append(float(raw_data[i]))
            timestamps.append(float(raw_data[i + 1]))

        return readings, timestamps

    def voltage_to_temperature(self, channel, voltage):
        cal = TEMPERATURE_CALIBRATION.get(channel)
        if not cal:
            return 0.0
        voltage_mv = voltage * 1000
        return voltage_mv * cal["gain"] + cal["offset"]

    # ------------------- LOGGING & TEST CONTROL -------------------
    def check_enable_test(self):
        if self.psu or self.daq:
            self.btn_start_test.config(state=tk.NORMAL)
        if self.psu and self.daq and not self.pid_active:
            self.btn_pid_hold.config(state=tk.NORMAL)

    def start_test(self):
        try:
            self.poll_interval = float(self.entry_poll_interval.get())
            if self.poll_interval <= 0:
                raise ValueError("Interval must be greater than zero.")
        except ValueError as e:
            messagebox.showerror("Input Error", f"Invalid poll interval: {e}")
            return

        self.test_running = True
        self.btn_start_test.config(state=tk.DISABLED)
        self.entry_poll_interval.config(state=tk.DISABLED)
        self.btn_stop_test.config(state=tk.NORMAL)

        self.loop_iteration = 0
        self.t_data.clear()
        self.v_data.clear()
        self.i_data.clear()
        for ch in CHANNELS:
            self.temp_data[ch].clear()
            self.ema_temp_data[ch].clear()
            self.ema_temps[ch] = None

        self.start_time = time.time()

        filename = f"daq_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.csv_file = open(filename, mode="w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        headers = [
            "PC_Timestamp",
            "PC_Elapsed_s",
            "PSU_Voltage_V",
            "PSU_Current_A",
            "PID_Active",
            "PID_Target_C",
            "PID_EMA_Temp_C",
            "PID_Cmd_Power_W",
            "PSU_Latency_ms",
            "DAQ_Latency_ms",
        ]
        for ch in CHANNELS:
            headers.extend([
                f"CH{ch}_Voltage_V",
                f"CH{ch}_Temp_C",
                f"CH{ch}_EMA_Temp_C",
                f"CH{ch}_HW_Time_s",
            ])

        self.csv_writer.writerow(headers)
        self.log(f"Started synchronized logging to {filename} (Interval: {self.poll_interval}s)")

    def stop_test(self):
        self.test_running = False
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
        self.btn_start_test.config(state=tk.NORMAL)
        self.entry_poll_interval.config(state=tk.NORMAL)
        self.btn_stop_test.config(state=tk.DISABLED)
        self.log("Data logging stopped.")

    def _set_all_temp_channels(self, visible):
        for ch in CHANNELS:
            self.temp_channel_visible[ch].set(bool(visible))
        self._on_temp_channel_visibility_changed()

    def _on_temp_channel_visibility_changed(self):
        if not hasattr(self, "line_temps") or not hasattr(self, "line_ema_temps"):
            return

        for ch in CHANNELS:
            is_vis = self.temp_channel_visible[ch].get()
            self.line_temps[ch].set_visible(is_vis)
            self.line_ema_temps[ch].set_visible(is_vis)

        self._refresh_temp_legends_and_scales()
        if hasattr(self, "canvas"):
            self.canvas.draw_idle()

    def _refresh_temp_legends_and_scales(self):
        for ax, line_dict in (
            (self.ax_temp, self.line_temps),
            (self.ax_ema_temp, self.line_ema_temps),
        ):
            visible_lines = [
                line_dict[ch]
                for ch in CHANNELS
                if self.temp_channel_visible[ch].get()
            ]

            old_legend = ax.get_legend()
            if old_legend is not None:
                old_legend.remove()

            if visible_lines:
                ax.legend(
                    visible_lines,
                    [line.get_label() for line in visible_lines],
                    fontsize=8,
                    loc="upper left",
                )
                ax.relim(visible_only=True)
                ax.autoscale_view()
            else:
                ax.set_ylim(auto=True)

    def update_plots(self):
        if not self.t_data:
            return
        self.line_v.set_data(self.t_data, self.v_data)
        self.line_i.set_data(self.t_data, self.i_data)

        for ch in CHANNELS:
            is_vis = self.temp_channel_visible[ch].get()
            self.line_temps[ch].set_data(self.t_data, self.temp_data[ch])
            self.line_temps[ch].set_visible(is_vis)

            self.line_ema_temps[ch].set_data(self.t_data, self.ema_temp_data[ch])
            self.line_ema_temps[ch].set_visible(is_vis)

        self.ax_pwr_v.relim()
        self.ax_pwr_v.autoscale_view()
        self.ax_pwr_i.relim()
        self.ax_pwr_i.autoscale_view()

        self._refresh_temp_legends_and_scales()

        self.canvas.draw()

    # ------------------- CORE POLLING LOOP -------------------
    def hardware_poll_loop(self):
        while self.polling_active:
            loop_start = time.time()
            self.process_ramping()

            current_time_str = datetime.now().isoformat()
            elapsed = loop_start - self.start_time if (self.test_running and self.start_time) else 0.0

            v_meas, i_meas = None, None
            psu_latency_ms, daq_latency_ms = None, None
            daq_voltages = [None] * len(CHANNELS)
            daq_temps = [None] * len(CHANNELS)
            daq_ema_temps = [None] * len(CHANNELS)
            daq_times = [None] * len(CHANNELS)

            # 1. Query PSU via PySerial
            if self.psu and self.psu.is_open:
                try:
                    t_start_psu = time.perf_counter()

                    with self.psu_lock:
                        if self.psu_protocol == "SCPI":
                            v_str = self._psu_query(self.psu, "MEAS:VOLT?", wait_s=0.05)
                            i_str = self._psu_query(self.psu, "MEAS:CURR?", wait_s=0.05)
                        else:
                            v_str = self._psu_query(self.psu, "MV?", wait_s=0.05)
                            i_str = self._psu_query(self.psu, "MC?", wait_s=0.05)

                        v_meas = float(v_str)
                        i_meas = float(i_str)

                    psu_latency_ms = (time.perf_counter() - t_start_psu) * 1000

                    self.root.after(0, lambda v=v_meas, i=i_meas: [
                        self.lbl_v_meas.config(text=f"Meas V: {v:.3f} V"),
                        self.lbl_i_meas.config(text=f"Meas I: {i:.3f} A"),
                    ])
                except Exception as e:
                    self.log(f"PSU Poll Error: {e}")

            # 2. Query DAQ & Update EMA Filter for all channels
            if self.daq and (self.test_running or self.pid_active):
                try:
                    t_start_daq = time.perf_counter()
                    daq_voltages, daq_times = self.read_all_channels()
                    daq_latency_ms = (time.perf_counter() - t_start_daq) * 1000

                    daq_temps = [self.voltage_to_temperature(ch, v) for ch, v in zip(CHANNELS, daq_voltages)]

                    alpha = self.pid_controller.ema_alpha
                    for idx, (ch, raw_t) in enumerate(zip(CHANNELS, daq_temps)):
                        if raw_t is not None:
                            if self.ema_temps[ch] is None:
                                self.ema_temps[ch] = raw_t
                            else:
                                self.ema_temps[ch] = alpha * raw_t + (1.0 - alpha) * self.ema_temps[ch]
                            daq_ema_temps[idx] = self.ema_temps[ch]

                    temp_str = " | ".join([f"CH{ch}: {t:.1f}°C" for ch, t in zip(CHANNELS, daq_temps)])
                    self.root.after(0, lambda ts=temp_str: self.lbl_daq_temps.config(text=ts))
                except Exception as e:
                    self.log(f"DAQ Poll Error: {e}")

            # 3. Execute PID Thermal Hold Step
            if self.pid_active and self.psu and self.daq:
                try:
                    ch_idx = CHANNELS.index(self.pid_channel)
                    raw_temp = daq_temps[ch_idx]
                    pre_filtered = daq_ema_temps[ch_idx]

                    if raw_temp is not None:
                        try:
                            target_v = float(self.entry_target_v.get())
                        except ValueError:
                            target_v = self.max_v_limit

                        new_i, cmd_power_w, filt_temp = self.pid_controller.compute(
                            raw_temp=raw_temp,
                            setpoint_temp=self.pid_target_temp,
                            v_meas=v_meas,
                            i_meas=i_meas,
                            v_target=target_v,
                            current_i_setpoint=self.psu_current_setpoint,
                            max_v_limit=self.max_v_limit,
                            max_i_limit=self.max_i_limit,
                            max_rate_a_per_sec=self.pid_max_rate_A_per_sec,
                            pre_filtered_temp=pre_filtered,
                        )

                        self.psu_current_setpoint = new_i
                        self.last_pid_power_w = cmd_power_w
                        self.last_pid_filt_temp = filt_temp

                        self._write_psu_current(new_i)

                        status_txt = (
                            f"PID: CH{self.pid_channel} -> {self.pid_target_temp:.1f}°C | "
                            f"EMA: {filt_temp:.2f}°C | Out: {cmd_power_w:.1f}W ({new_i:.3f}A)"
                        )
                        self.root.after(0, lambda s=status_txt: self.lbl_pid_status.config(text=s))
                except Exception as e:
                    self.log(f"PID Execution Error: {e}")

            # 4. Log & Plot
            if self.test_running:
                if self.csv_writer:
                    row = [
                        current_time_str,
                        round(elapsed, 3),
                        v_meas if v_meas is not None else "",
                        i_meas if i_meas is not None else "",
                        int(self.pid_active),
                        self.pid_target_temp if self.pid_active else "",
                        round(self.last_pid_filt_temp, 3) if (self.pid_active and self.last_pid_filt_temp is not None) else "",
                        round(self.last_pid_power_w, 2) if self.pid_active else "",
                        round(psu_latency_ms, 2) if psu_latency_ms is not None else "",
                        round(daq_latency_ms, 2) if daq_latency_ms is not None else "",
                    ]

                    for v, t, ema_t, hw_time in zip(daq_voltages, daq_temps, daq_ema_temps, daq_times):
                        row.extend([
                            v if v is not None else "",
                            t if t is not None else "",
                            round(ema_t, 3) if ema_t is not None else "",
                            hw_time if hw_time is not None else "",
                        ])

                    self.csv_writer.writerow(row)
                    self.csv_file.flush()

                self.t_data.append(elapsed)
                self.v_data.append(v_meas if v_meas is not None else float("nan"))
                self.i_data.append(i_meas if i_meas is not None else float("nan"))

                for i, ch in enumerate(CHANNELS):
                    self.temp_data[ch].append(daq_temps[i] if daq_temps[i] is not None else float("nan"))
                    self.ema_temp_data[ch].append(daq_ema_temps[i] if daq_ema_temps[i] is not None else float("nan"))

                self.root.after(0, self.update_plots)

            # 5. Enforce Exact Absolute Polling Frequency
            if self.test_running and self.start_time:
                self.loop_iteration += 1
                next_target_time = self.start_time + (self.loop_iteration * self.poll_interval)
                sleep_time = next_target_time - time.time()
                if sleep_time > 0:
                    time.sleep(sleep_time)
            else:
                sleep_time = DEFAULT_POLL_INTERVAL - (time.time() - loop_start)
                if sleep_time > 0:
                    time.sleep(sleep_time)

    def on_closing(self):
        self.polling_active = False
        if self.test_running:
            self.stop_test()

        with self.psu_lock:
            if self.psu and self.psu.is_open:
                try:
                    if self.psu_protocol == "SCPI":
                        self._psu_write(self.psu, "OUTP OFF")
                        self._psu_write(self.psu, "SYST:SET LOC")
                    else:
                        self._psu_query(self.psu, "OUT 0")
                        self._psu_query(self.psu, "RMT 0")
                    self.psu.close()
                except Exception:
                    pass
                self.psu = None

        with self.daq_lock:
            if self.daq:
                try:
                    self.daq.write(":ABOR")
                    self.daq.close()
                except Exception:
                    pass

        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = UnifiedHardwareApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
