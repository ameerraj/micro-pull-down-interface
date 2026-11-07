import tkinter as tk
from tkinter import ttk, messagebox
import csv
import math
import threading
import time
from datetime import datetime

import pyvisa
import serial
import serial.tools.list_ports
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


# ============================================================
# DISCRETE PID CONTROLLER (WITH ANTI-WINDUP & EMA FILTER)
# ============================================================
class DiscretePID:
    def __init__(self, kp, ki, kd, out_min, out_max, ema_alpha=0.2):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max
        self.ema_alpha = ema_alpha
        
        self.integral = 0.0
        self.last_temp = None
        self.filtered_derivative = 0.0

    def reset(self):
        self.integral = 0.0
        self.last_temp = None
        self.filtered_derivative = 0.0

    def update_gains(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd

    def compute(self, setpoint, current_temp, dt):
        if dt <= 0:
            return 0.0

        error = setpoint - current_temp

        # 1. Proportional Term
        p_out = self.kp * error

        # 2. Integral Term with Anti-Windup (Dynamic Clamping)
        self.integral += error * dt
        i_out = self.ki * self.integral

        # 3. Derivative Term with EMA Filter (calculated on measurement, not error)
        d_out = 0.0
        if self.last_temp is not None:
            raw_derivative = (current_temp - self.last_temp) / dt
            self.filtered_derivative = (
                self.ema_alpha * raw_derivative 
                + (1 - self.ema_alpha) * self.filtered_derivative
            )
            d_out = -self.kd * self.filtered_derivative 
        self.last_temp = current_temp

        # 4. Sum Outputs & Apply Hard Clamping
        output = p_out + i_out + d_out

        if output > self.out_max:
            output = self.out_max
            # Anti-windup: Recalculate integral to exactly match the ceiling
            if self.ki != 0:
                self.integral = (self.out_max - p_out - d_out) / self.ki
        elif output < self.out_min:
            output = self.out_min
            if self.ki != 0:
                self.integral = (self.out_min - p_out - d_out) / self.ki

        return output


# ============================================================
# POWER SUPPLY CONFIGURATION
# ============================================================
HARDWARE_OVP = 10.5       # Safe OVP ceiling for 0-10 V TDK-Lambda GSP
PSU_BAUD_RATE = 115200

# ============================================================
# GENERAL POLLING CONFIGURATION
# ============================================================
DEFAULT_POLL_INTERVAL = 1.0

# ============================================================
# KEITHLEY DAQ6510 CONFIGURATION & CALIBRATION
# ============================================================
CHANNELS = [101, 102, 103, 104, 105]
CHANNEL_LIST = "(@101:105)"
NPLC = 1.0
VISA_TIMEOUT_MS = 5000

# temperature = gain * measured voltage (mV) + offset
TEMPERATURE_CALIBRATION = {
    101: {"gain": 84.33, "offset": 10.78},
    102: {"gain": 84.33, "offset": 10.78},
    103: {"gain": 84.33, "offset": 10.78},
    104: {"gain": 84.33, "offset": 10.78},
    105: {"gain": 84.33, "offset": 10.78},  # crucible thermocouple
}


class PSUAndThermocoupleApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Power Supply & Thermocouple Controller")
        self.root.geometry("1400x900")
        self.root.minsize(1200, 800)

        # Hardware locks
        self.psu_lock = threading.Lock()
        self.daq_lock = threading.Lock()

        # VISA backend
        self.rm = pyvisa.ResourceManager()

        # Hardware handles
        self.psu = None
        self.psu_protocol = "SCPI"
        self.daq = None

        # Logging state
        self.polling_active = True
        self.test_running = False
        self.csv_file = None
        self.csv_writer = None
        self.loop_iteration = 0
        self.poll_interval = DEFAULT_POLL_INTERVAL
        self.start_time = None

        # Manual PSU Ramp State
        self.max_v_limit = 10.0
        self.max_i_limit = 360.0
        self.manual_ramping_active = False
        self.psu_target_i = 0.0
        self.psu_current_setpoint = 0.0
        self.ramp_rate_A_per_sec = 0.0
        self.last_manual_ramp_time = 0.0

        # PID Thermal Control State
        self.pid_active = False
        self.pid_controller = DiscretePID(kp=0.5, ki=0.01, kd=0.1, out_min=0.0, out_max=360.0)
        self.pid_target_temp = 0.0
        self.pid_moving_setpoint = 0.0
        self.temp_ramp_rate_C_per_sec = 0.0
        self.last_pid_time = 0.0
        self.control_channel = 105

        # Ports
        self.available_psu_com = []
        self.available_usb_visa = []

        # Plot data
        self.t_data = []
        self.v_data = []
        self.i_data = []
        self.temp_data = {ch: [] for ch in CHANNELS}

        self.temp_channel_visible = {
            ch: tk.BooleanVar(value=True) for ch in CHANNELS
        }
        self.log_history = []

        self.scan_ports()
        self.setup_ui()

        self.poll_thread = threading.Thread(
            target=self.hardware_poll_loop,
            daemon=True,
        )
        self.poll_thread.start()

    # ============================================================
    # PORT SCANNING
    # ============================================================
    def scan_ports(self):
        try:
            resources = self.rm.list_resources()
            self.available_usb_visa = [
                resource for resource in resources
                if "USB" in resource or "TCPIP" in resource
            ]
        except Exception:
            self.available_usb_visa = []

        try:
            com_ports = serial.tools.list_ports.comports()
            self.available_psu_com = []
            for port in com_ports:
                if "2504:0002" in port.hwid or port.device == "COM4":
                    self.available_psu_com.insert(0, port.device)
                else:
                    self.available_psu_com.append(port.device)
            if not self.available_psu_com:
                self.available_psu_com = ["COM4"]
        except Exception:
            self.available_psu_com = ["COM4"]

    def refresh_ports(self):
        self.scan_ports()
        self.combo_psu_port["values"] = self.available_psu_com
        if not self.combo_psu_port.get() and self.available_psu_com:
            self.combo_psu_port.set(self.available_psu_com[0])

        self.combo_daq_port["values"] = self.available_usb_visa
        if not self.combo_daq_port.get() and self.available_usb_visa:
            self.combo_daq_port.set(self.available_usb_visa[0])
        self.log("Hardware port lists refreshed.")

    # ============================================================
    # USER INTERFACE
    # ============================================================
    def setup_ui(self):
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # 1. Logging Frame
        logging_frame = ttk.LabelFrame(main_frame, text="Synchronized Data Logging", padding=8)
        logging_frame.pack(fill=tk.X, pady=(0, 8))
        
        ttk.Label(logging_frame, text="Poll Interval (s):").grid(row=0, column=0, padx=(0, 6))
        self.entry_poll_interval = ttk.Entry(logging_frame, width=10)
        self.entry_poll_interval.grid(row=0, column=1, padx=(0, 12))
        self.entry_poll_interval.insert(0, "1.0")

        self.btn_start_test = ttk.Button(logging_frame, text="Start Log", command=self.start_test, state=tk.DISABLED)
        self.btn_start_test.grid(row=0, column=2, padx=2)

        self.btn_stop_test = ttk.Button(logging_frame, text="Stop Log", command=self.stop_test, state=tk.DISABLED)
        self.btn_stop_test.grid(row=0, column=3, padx=2)

        ttk.Button(logging_frame, text="Refresh Ports", command=self.refresh_ports).grid(row=0, column=4, padx=(8, 0))

        # 2. Hardware Controls Frame
        control_frame = ttk.Frame(main_frame)
        control_frame.pack(fill=tk.X, pady=(0, 8))
        control_frame.columnconfigure(0, weight=1)
        control_frame.columnconfigure(1, weight=1)
        control_frame.columnconfigure(2, weight=1)

        self._build_psu_interface(control_frame)
        self._build_pid_interface(control_frame)
        self._build_daq_interface(control_frame)

        # 3. System Log
        log_frame = ttk.LabelFrame(main_frame, text="System Log", padding=5)
        log_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=(8, 0))

        self.log_text = tk.Text(log_frame, height=5, state=tk.DISABLED, wrap=tk.WORD)
        log_scroll = ttk.Scrollbar(log_frame, orient=tk.VERTICAL, command=self.log_text.yview)
        self.log_text.configure(yscrollcommand=log_scroll.set)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        log_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        # 4. Graphs
        graph_frame = ttk.Frame(main_frame)
        graph_frame.pack(fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(11, 4), dpi=100)
        self.fig.patch.set_facecolor("#f0f0f0")

        self.ax_pwr_v = self.fig.add_subplot(121)
        self.ax_pwr_i = self.ax_pwr_v.twinx()
        self.ax_pwr_v.set_title("Power Supply")
        self.ax_pwr_v.set_xlabel("Time (s)")
        self.ax_pwr_v.set_ylabel("Voltage (V)")
        self.ax_pwr_i.set_ylabel("Current (A)")
        self.line_v, = self.ax_pwr_v.plot([], [], label="Voltage", color='blue')
        self.line_i, = self.ax_pwr_i.plot([], [], label="Current", color='orange')
        
        self.ax_temp = self.fig.add_subplot(122)
        self.ax_temp.set_title("Thermocouples & PID Setpoint")
        self.ax_temp.set_xlabel("Time (s)")
        self.ax_temp.set_ylabel("Temperature (°C)")

        self.line_temps = {}
        for ch in CHANNELS:
            self.line_temps[ch], = self.ax_temp.plot([], [], label=f"CH {ch}")
            
        self.line_setpoint, = self.ax_temp.plot([], [], 'k--', label="Target Setpoint", linewidth=2)
        
        self.ax_temp.legend(fontsize=8, loc="upper left")
        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _build_psu_interface(self, parent):
        psu_frame = ttk.LabelFrame(parent, text="Manual PSU Control", padding=10)
        psu_frame.grid(row=0, column=0, sticky="nsew", padx=(0, 5))

        ttk.Label(psu_frame, text="Port:").grid(row=0, column=0, sticky="w")
        self.combo_psu_port = ttk.Combobox(psu_frame, values=self.available_psu_com, width=10)
        if self.available_psu_com:
            self.combo_psu_port.set(self.available_psu_com[0])
        self.combo_psu_port.grid(row=0, column=1, padx=5, pady=2)

        self.btn_psu_conn = ttk.Button(psu_frame, text="Connect", command=self.connect_psu_thread)
        self.btn_psu_conn.grid(row=0, column=2, pady=2, sticky="ew")

        ttk.Label(psu_frame, text="Max Allow V:").grid(row=1, column=0, sticky="w")
        self.entry_max_v = ttk.Entry(psu_frame, width=8)
        self.entry_max_v.grid(row=1, column=1, padx=5, pady=2)
        self.entry_max_v.insert(0, "10.0")

        ttk.Label(psu_frame, text="Max Allow I:").grid(row=1, column=2, sticky="w")
        self.entry_max_i = ttk.Entry(psu_frame, width=8)
        self.entry_max_i.grid(row=1, column=3, padx=5, pady=2)
        self.entry_max_i.insert(0, "360.0")

        ttk.Label(psu_frame, text="Target I (A):").grid(row=2, column=0, sticky="w")
        self.entry_target_i = ttk.Entry(psu_frame, width=8)
        self.entry_target_i.grid(row=2, column=1, padx=5, pady=2)
        self.entry_target_i.insert(0, "0.0")

        ttk.Label(psu_frame, text="Ramp (A/min):").grid(row=2, column=2, sticky="w")
        self.entry_ramp_rate = ttk.Entry(psu_frame, width=8)
        self.entry_ramp_rate.grid(row=2, column=3, padx=5, pady=2)
        self.entry_ramp_rate.insert(0, "10.0")

        self.btn_apply = ttk.Button(psu_frame, text="Start I-Ramp", command=self.start_manual_ramp, state=tk.DISABLED)
        self.btn_apply.grid(row=3, column=0, columnspan=2, pady=5, sticky="ew", padx=2)

        self.btn_stop_ramp = ttk.Button(psu_frame, text="Hold Current", command=self.stop_manual_ramp, state=tk.DISABLED)
        self.btn_stop_ramp.grid(row=3, column=2, columnspan=2, pady=5, sticky="ew", padx=2)

        self.btn_safe_off = ttk.Button(psu_frame, text="🛑 SAFELY TURN OFF HEATER", command=self.safe_heater_off, state=tk.DISABLED)
        self.btn_safe_off.grid(row=4, column=0, columnspan=4, pady=5, sticky="ew")

        self.lbl_v_meas = ttk.Label(psu_frame, text="Meas V: -- V", font=("Arial", 11, "bold"))
        self.lbl_v_meas.grid(row=5, column=0, columnspan=2, pady=5)
        self.lbl_i_meas = ttk.Label(psu_frame, text="Meas I: -- A", font=("Arial", 11, "bold"))
        self.lbl_i_meas.grid(row=5, column=2, columnspan=2, pady=5)

    def _build_pid_interface(self, parent):
        pid_frame = ttk.LabelFrame(parent, text="PID Thermal Profile Control", padding=10)
        pid_frame.grid(row=0, column=1, sticky="nsew", padx=5)

        ttk.Label(pid_frame, text="Control CH:").grid(row=0, column=0, sticky="w")
        self.combo_pid_ch = ttk.Combobox(pid_frame, values=CHANNELS, width=8)
        self.combo_pid_ch.set(105)
        self.combo_pid_ch.grid(row=0, column=1, padx=5, pady=2)

        ttk.Label(pid_frame, text="Kp:").grid(row=1, column=0, sticky="w")
        self.entry_kp = ttk.Entry(pid_frame, width=8)
        self.entry_kp.grid(row=1, column=1, padx=5, pady=2)
        self.entry_kp.insert(0, "0.5")

        ttk.Label(pid_frame, text="Ki:").grid(row=1, column=2, sticky="w")
        self.entry_ki = ttk.Entry(pid_frame, width=8)
        self.entry_ki.grid(row=1, column=3, padx=5, pady=2)
        self.entry_ki.insert(0, "0.01")
        
        ttk.Label(pid_frame, text="Kd:").grid(row=2, column=0, sticky="w")
        self.entry_kd = ttk.Entry(pid_frame, width=8)
        self.entry_kd.grid(row=2, column=1, padx=5, pady=2)
        self.entry_kd.insert(0, "0.1")

        ttk.Label(pid_frame, text="Target Temp (°C):").grid(row=3, column=0, columnspan=2, sticky="w")
        self.entry_target_temp = ttk.Entry(pid_frame, width=8)
        self.entry_target_temp.grid(row=3, column=2, columnspan=2, padx=5, pady=2, sticky="w")
        self.entry_target_temp.insert(0, "600.0")
        
        ttk.Label(pid_frame, text="Temp Ramp (°C/min):").grid(row=4, column=0, columnspan=2, sticky="w")
        self.entry_temp_ramp = ttk.Entry(pid_frame, width=8)
        self.entry_temp_ramp.grid(row=4, column=2, columnspan=2, padx=5, pady=2, sticky="w")
        self.entry_temp_ramp.insert(0, "60.0")

        self.btn_start_pid = ttk.Button(pid_frame, text="Start Auto Profile", command=self.start_thermal_profile, state=tk.DISABLED)
        self.btn_start_pid.grid(row=5, column=0, columnspan=2, pady=5, sticky="ew", padx=2)

        self.btn_stop_pid = ttk.Button(pid_frame, text="Stop PID", command=self.stop_thermal_profile, state=tk.DISABLED)
        self.btn_stop_pid.grid(row=5, column=2, columnspan=2, pady=5, sticky="ew", padx=2)

        self.lbl_pid_status = ttk.Label(pid_frame, text="Status: OFF | Setpoint: -- °C", font=("Arial", 9))
        self.lbl_pid_status.grid(row=6, column=0, columnspan=4, pady=5)

    def _build_daq_interface(self, parent):
        daq_frame = ttk.LabelFrame(parent, text="Keithley DAQ6510", padding=10)
        daq_frame.grid(row=0, column=2, sticky="nsew", padx=(5, 0))

        self.combo_daq_port = ttk.Combobox(daq_frame, values=self.available_usb_visa)
        if self.available_usb_visa:
            self.combo_daq_port.set(self.available_usb_visa[0])
        self.combo_daq_port.pack(fill=tk.X, pady=2)

        self.btn_daq_conn = ttk.Button(daq_frame, text="Connect DAQ", command=self.connect_daq_thread)
        self.btn_daq_conn.pack(fill=tk.X, pady=2)

        self.lbl_daq_status = ttk.Label(daq_frame, text="Status: Disconnected")
        self.lbl_daq_status.pack(pady=2)

        self.lbl_daq_temps = ttk.Label(daq_frame, text="Temps: -- °C", font=("Arial", 11), wraplength=300, justify=tk.LEFT)
        self.lbl_daq_temps.pack(fill=tk.X, pady=(5, 5))

        channel_filter = ttk.LabelFrame(daq_frame, text="Graph Channels", padding=5)
        channel_filter.pack(fill=tk.X, pady=(2, 2))

        for index, ch in enumerate(CHANNELS):
            ttk.Checkbutton(
                channel_filter, text=f"CH {ch}", variable=self.temp_channel_visible[ch],
                command=self._on_temp_channel_visibility_changed
            ).grid(row=index // 3, column=index % 3, sticky="w", padx=3, pady=2)

    # ============================================================
    # SYSTEM LOG
    # ============================================================
    def log(self, message):
        line = f"[{time.strftime('%H:%M:%S')}] {message}\n"
        self.log_history.append(line)
        if len(self.log_history) > 1000:
            self.log_history = self.log_history[-1000:]
        def append():
            try:
                self.log_text.config(state=tk.NORMAL)
                self.log_text.insert(tk.END, line)
                self.log_text.see(tk.END)
                self.log_text.config(state=tk.DISABLED)
            except tk.TclError:
                pass
        self.root.after(0, append)

    # ============================================================
    # PSU SERIAL METHODS
    # ============================================================
    @staticmethod
    def _psu_write(ser_inst, cmd):
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

    def check_psu_command(self, ser_inst, cmd):
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

    def connect_psu_thread(self):
        selected_port = self.combo_psu_port.get().strip()
        if not selected_port:
            messagebox.showerror("Error", "Select a COM port.")
            return
        if selected_port.upper().startswith("ASRL"):
            num = "".join(filter(str.isdigit, selected_port))
            selected_port = f"COM{num}"

        self.btn_psu_conn.config(state=tk.DISABLED)
        self.combo_psu_port.config(state=tk.DISABLED)
        self.log(f"Connecting PSU on {selected_port}...")
        threading.Thread(target=self._connect_psu, args=(selected_port,), daemon=True).start()

    def _connect_psu(self, port):
        temp_ser = None
        try:
            with self.psu_lock:
                temp_ser = serial.Serial(
                    port=port, baudrate=PSU_BAUD_RATE, bytesize=serial.EIGHTBITS,
                    parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                    timeout=0.6, write_timeout=1.0, rtscts=False, dsrdtr=False
                )
                temp_ser.dtr = True
                temp_ser.rts = True
                time.sleep(0.15)
                temp_ser.reset_input_buffer()
                temp_ser.reset_output_buffer()

                connected_idn = None
                self.psu_protocol = None
                for address in [6, 1, 0]:
                    self._psu_write(temp_ser, f"INST:NSEL {address}")
                    time.sleep(0.1)
                    temp_ser.reset_input_buffer()
                    idn = self._psu_query(temp_ser, "*IDN?", wait_s=0.15)
                    if idn:
                        connected_idn = idn
                        self.psu_protocol = "SCPI"
                        self.log(f"✅ PSU SCPI Addr {address}: {connected_idn}")
                        break
                    
                    response = self._psu_query(temp_ser, f"ADR {address:02d}", wait_s=0.15)
                    if "OK" in response:
                        connected_idn = self._psu_query(temp_ser, "IDN?", wait_s=0.15)
                        self.psu_protocol = "GEN"
                        self.log(f"✅ PSU GEN Addr {address:02d}: {connected_idn}")
                        break

                if not connected_idn:
                    raise RuntimeError(f"No response on {port}.")

                if self.psu_protocol == "SCPI":
                    cmds = ["SYST:SET REM", f"VOLT:PROT:LEV {HARDWARE_OVP}", "VOLT 10.0", "CURR 0.0", "OUTP ON"]
                else:
                    cmds = ["RMT 1", f"OVP {HARDWARE_OVP}", "PV 10.0", "PC 0.0", "OUT 1"]
                for cmd in cmds:
                    self.check_psu_command(temp_ser, cmd)
                self.psu = temp_ser

            def update_ui():
                self.btn_psu_conn.config(text="PSU Connected", state=tk.DISABLED)
                self.btn_apply.config(state=tk.NORMAL)
                self.btn_safe_off.config(state=tk.NORMAL)
                self.check_enable_test()
                # Enable PID start if DAQ is also connected
                if self.daq is not None:
                    self.btn_start_pid.config(state=tk.NORMAL)
            self.root.after(0, update_ui)

        except Exception as exc:
            self.log(f"PSU Connect failed: {exc}")
            if temp_ser and temp_ser.is_open:
                temp_ser.close()
            def reset_ui():
                self.btn_psu_conn.config(state=tk.NORMAL, text="Connect PSU")
                self.combo_psu_port.config(state=tk.NORMAL)
            self.root.after(0, reset_ui)

    # ============================================================
    # MANUAL RAMP CONTROL
    # ============================================================
    def start_manual_ramp(self):
        if not self.psu or self.pid_active:
            messagebox.showwarning("Warning", "Cannot start manual ramp while PID is active.")
            return

        try:
            self.max_v_limit = float(self.entry_max_v.get())
            self.max_i_limit = min(float(self.entry_max_i.get()), 360.0)
            target_i = float(self.entry_target_i.get())
            ramp_rate_a_min = float(self.entry_ramp_rate.get())

            if self.max_v_limit > HARDWARE_OVP:
                messagebox.showerror("Limit Error", f"V cannot exceed {HARDWARE_OVP} V.")
                return

            with self.psu_lock:
                if self.psu_protocol == "SCPI":
                    self.check_psu_command(self.psu, f"VOLT {self.max_v_limit}")
                    current_setpoint_str = self._psu_query(self.psu, "CURR?")
                else:
                    self.check_psu_command(self.psu, f"PV {self.max_v_limit}")
                    current_setpoint_str = self._psu_query(self.psu, "PC?")

            self.psu_current_setpoint = float(current_setpoint_str)
            self.psu_target_i = target_i
            self.ramp_rate_A_per_sec = ramp_rate_a_min / 60.0
            self.last_manual_ramp_time = time.time()
            self.manual_ramping_active = True
            self.btn_stop_ramp.config(state=tk.NORMAL)
            self.log(f"Manual Ramp: -> {target_i} A @ {ramp_rate_a_min} A/min.")

        except Exception as exc:
            self.log(f"Error starting manual ramp: {exc}")

    def stop_manual_ramp(self):
        if self.manual_ramping_active:
            self.manual_ramping_active = False
            self.btn_stop_ramp.config(state=tk.DISABLED)
            self.log(f"Manual ramp stopped. Holding at {self.psu_current_setpoint:.3f} A.")

    # ============================================================
    # PID THERMAL CONTROL
    # ============================================================
    def start_thermal_profile(self):
        if not self.psu or not self.daq:
            messagebox.showerror("Error", "Both PSU and DAQ must be connected for PID control.")
            return

        try:
            self.control_channel = int(self.combo_pid_ch.get())
            kp = float(self.entry_kp.get())
            ki = float(self.entry_ki.get())
            kd = float(self.entry_kd.get())
            self.pid_target_temp = float(self.entry_target_temp.get())
            ramp_c_min = float(self.entry_temp_ramp.get())
            
            self.max_v_limit = float(self.entry_max_v.get())
            self.max_i_limit = min(float(self.entry_max_i.get()), 360.0)

            self.pid_controller.update_gains(kp, ki, kd)
            self.pid_controller.out_max = self.max_i_limit
            self.temp_ramp_rate_C_per_sec = ramp_c_min / 60.0

            # Stop manual ramping if active
            self.manual_ramping_active = False
            self.btn_stop_ramp.config(state=tk.DISABLED)

            # Initialize moving setpoint to current actual temperature
            current_temp = self._get_latest_temp(self.control_channel)
            if current_temp is None:
                raise ValueError("Could not read current temperature. Check DAQ.")
                
            self.pid_moving_setpoint = current_temp
            self.pid_controller.reset()
            self.last_pid_time = time.time()
            
            # Ensure voltage limit is set
            with self.psu_lock:
                if self.psu_protocol == "SCPI":
                    self.check_psu_command(self.psu, f"VOLT {self.max_v_limit}")
                else:
                    self.check_psu_command(self.psu, f"PV {self.max_v_limit}")

            self.pid_active = True
            self.btn_start_pid.config(state=tk.DISABLED)
            self.btn_stop_pid.config(state=tk.NORMAL)
            self.btn_apply.config(state=tk.DISABLED) # Disable manual ramp
            
            # Reset setpoint plotting data
            self.setpoint_data = []

            self.log(f"PID Profile Started: Ramping CH{self.control_channel} to {self.pid_target_temp}°C at {ramp_c_min}°C/min.")

        except Exception as exc:
            messagebox.showerror("PID Error", str(exc))

    def stop_thermal_profile(self):
        if self.pid_active:
            self.pid_active = False
            self.btn_stop_pid.config(state=tk.DISABLED)
            self.btn_start_pid.config(state=tk.NORMAL)
            self.btn_apply.config(state=tk.NORMAL)
            self.lbl_pid_status.config(text="Status: OFF | Holding current A")
            self.log(f"PID Stopped. Holding PSU at last commanded current: {self.psu_current_setpoint:.2f} A.")

    def _get_latest_temp(self, channel):
        if not self.temp_data[channel]:
            return None
        return self.temp_data[channel][-1]

    # ============================================================
    # SAFE SHUTDOWN
    # ============================================================
    def safe_heater_off(self):
        if not self.psu: return
        self.manual_ramping_active = False
        self.pid_active = False
        self.btn_stop_ramp.config(state=tk.DISABLED)
        self.btn_stop_pid.config(state=tk.DISABLED)
        self.btn_start_pid.config(state=tk.NORMAL if self.daq else tk.DISABLED)
        self.btn_apply.config(state=tk.NORMAL)

        try:
            with self.psu_lock:
                if self.psu_protocol == "SCPI":
                    self.check_psu_command(self.psu, "CURR 0.0")
                    self.check_psu_command(self.psu, "OUTP OFF")
                else:
                    self.check_psu_command(self.psu, "PC 0.0")
                    self.check_psu_command(self.psu, "OUT 0")
            self.psu_current_setpoint = 0.0
            self.psu_target_i = 0.0
            self.log("🛑 SAFE OFF TRIGGERED. Heater set to 0 A and output disabled.")
            self.lbl_pid_status.config(text="Status: SAFE OFF")
        except Exception as exc:
            self.log(f"Error during Safe Off: {exc}")

    # ============================================================
    # DAQ METHODS
    # ============================================================
    def connect_daq_thread(self):
        selected_port = self.combo_daq_port.get().strip()
        if not selected_port: return
        self.btn_daq_conn.config(state=tk.DISABLED)
        self.combo_daq_port.config(state=tk.DISABLED)
        self.lbl_daq_status.config(text="Status: Connecting...")
        threading.Thread(target=self._connect_daq, args=(selected_port,), daemon=True).start()

    def _connect_daq(self, port):
        temp_daq = None
        try:
            with self.daq_lock:
                temp_daq = self.rm.open_resource(port)
                temp_daq.timeout = VISA_TIMEOUT_MS
                temp_daq.write_termination = "\n"
                temp_daq.read_termination = "\n"
                identification = temp_daq.query("*IDN?").strip()
                self.daq = temp_daq
            self.log(f"DAQ Connected: {identification}")
            self.configure_daq()

            def update_ui():
                self.btn_daq_conn.config(text="DAQ Connected", state=tk.DISABLED)
                self.lbl_daq_status.config(text="Status: Configured")
                self.check_enable_test()
                if self.psu is not None:
                    self.btn_start_pid.config(state=tk.NORMAL)
            self.root.after(0, update_ui)
        except Exception as exc:
            self.log(f"DAQ Connect failed: {exc}")
            if temp_daq: temp_daq.close()
            self.daq = None
            def reset_ui():
                self.btn_daq_conn.config(state=tk.NORMAL, text="Connect DAQ")
                self.combo_daq_port.config(state=tk.NORMAL)
                self.lbl_daq_status.config(text="Status: Disconnected")
            self.root.after(0, reset_ui)

    def configure_daq(self):
        with self.daq_lock:
            self.daq.write(":ABOR")
            self.daq.write("*RST")
            self.daq.query("*OPC?")
            self.daq.write("*CLS")
            self.daq.write(f':SENS:FUNC "VOLT:DC", {CHANNEL_LIST}')
            self.daq.write(f":SENS:VOLT:DC:RANG:AUTO ON, {CHANNEL_LIST}")
            self.daq.write(f":SENS:VOLT:DC:NPLC {NPLC}, {CHANNEL_LIST}")
            self.daq.write(f":ROUT:SCAN:CRE {CHANNEL_LIST}")
            self.daq.write(":ROUT:SCAN:COUNT:SCAN 1")
            self.daq.query("*OPC?")

    def read_all_channels(self):
        with self.daq_lock:
            self.daq.write(':TRAC:CLEAR "defbuffer1"')
            self.daq.write(":INIT")
            self.daq.query("*OPC?")
            raw_data = self.daq.query_ascii_values(f':TRAC:DATA? 1, {len(CHANNELS)}, "defbuffer1", READ, REL')
        readings = [float(raw_data[i]) for i in range(0, len(raw_data), 2)]
        timestamps = [float(raw_data[i+1]) for i in range(0, len(raw_data), 2)]
        return readings, timestamps

    @staticmethod
    def voltage_to_temperature(channel, voltage):
        cal = TEMPERATURE_CALIBRATION.get(channel)
        if not cal: return 0.0
        return (voltage * 1000.0 * cal["gain"]) + cal["offset"]

    # ============================================================
    # LOGGING / PLOTTING
    # ============================================================
    def check_enable_test(self):
        if self.psu or self.daq:
            self.btn_start_test.config(state=tk.NORMAL)

    def start_test(self):
        try:
            self.poll_interval = float(self.entry_poll_interval.get())
        except ValueError: return
        self.test_running = True
        self.btn_start_test.config(state=tk.DISABLED)
        self.btn_stop_test.config(state=tk.NORMAL)
        self.loop_iteration = 0
        self.t_data.clear()
        self.v_data.clear()
        self.i_data.clear()
        self.setpoint_data = []
        for ch in CHANNELS: self.temp_data[ch].clear()
        self.start_time = time.time()
        
        filename = f"psu_thermocouple_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        try:
            self.csv_file = open(filename, mode="w", newline="")
            self.csv_writer = csv.writer(self.csv_file)
            headers = ["PC_Timestamp", "PC_Elapsed_s", "PSU_Voltage_V", "PSU_Current_A", "PID_Target_Temp", "PID_Active", "PSU_Latency_ms", "DAQ_Latency_ms"]
            for ch in CHANNELS:
                headers.extend([f"CH{ch}_Voltage_V", f"CH{ch}_Temp_C", f"CH{ch}_HW_Time_s"])
            self.csv_writer.writerow(headers)
            self.csv_file.flush()
        except Exception:
            self.test_running = False
            return
        self.log(f"Started logging to {filename}")

    def stop_test(self):
        self.test_running = False
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
        self.btn_start_test.config(state=tk.NORMAL if (self.psu or self.daq) else tk.DISABLED)
        self.btn_stop_test.config(state=tk.DISABLED)
        self.log("Logging stopped.")

    def _on_temp_channel_visibility_changed(self):
        for ch in CHANNELS:
            self.line_temps[ch].set_visible(self.temp_channel_visible[ch].get())
        visible_lines = [self.line_temps[ch] for ch in CHANNELS if self.temp_channel_visible[ch].get()]
        visible_lines.append(self.line_setpoint)
        
        old_legend = self.ax_temp.get_legend()
        if old_legend: old_legend.remove()
        self.ax_temp.legend(visible_lines, [line.get_label() for line in visible_lines], fontsize=8, loc="upper left")
        self.ax_temp.relim(visible_only=True)
        self.ax_temp.autoscale_view()
        self.canvas.draw_idle()

    def update_plots(self):
        if not self.t_data: return
        self.line_v.set_data(self.t_data, self.v_data)
        self.line_i.set_data(self.t_data, self.i_data)
        for ch in CHANNELS:
            self.line_temps[ch].set_data(self.t_data, self.temp_data[ch])
            
        if hasattr(self, 'setpoint_data') and len(self.setpoint_data) == len(self.t_data):
            self.line_setpoint.set_data(self.t_data, self.setpoint_data)
            
        self.ax_pwr_v.relim()
        self.ax_pwr_v.autoscale_view()
        self.ax_pwr_i.relim()
        self.ax_pwr_i.autoscale_view()
        self.ax_temp.relim(visible_only=True)
        self.ax_temp.autoscale_view()
        self.canvas.draw_idle()

    # ============================================================
    # CORE POLLING LOOP (INCLUDES PID CALCULATION)
    # ============================================================
    def hardware_poll_loop(self):
        while self.polling_active:
            loop_start = time.time()
            elapsed = (loop_start - self.start_time if self.test_running and self.start_time else 0.0)

            v_meas, i_meas = None, None
            psu_latency, daq_latency = None, None
            daq_voltages, daq_temps, daq_times = [None]*5, [None]*5, [None]*5

            # 1. READ DAQ
            if self.daq is not None:
                try:
                    t_daq = time.perf_counter()
                    daq_voltages, daq_times = self.read_all_channels()
                    daq_latency = (time.perf_counter() - t_daq) * 1000.0
                    daq_temps = [self.voltage_to_temperature(ch, v) for ch, v in zip(CHANNELS, daq_voltages)]
                    temp_str = " | ".join(f"CH{ch}: {t:.1f}°C" for ch, t in zip(CHANNELS, daq_temps))
                    self.root.after(0, lambda text=temp_str: self.lbl_daq_temps.config(text=text))
                except Exception as exc:
                    self.log(f"DAQ Poll Error: {exc}")

            # 2. COMPUTE PID & COMMAND PSU
            if self.pid_active and self.psu and self.psu.is_open:
                now = time.time()
                dt = now - self.last_pid_time
                self.last_pid_time = now
                
                if dt > 0:
                    # Update Moving Setpoint
                    delta_temp = self.temp_ramp_rate_C_per_sec * dt
                    if self.pid_moving_setpoint < self.pid_target_temp:
                        self.pid_moving_setpoint = min(self.pid_moving_setpoint + delta_temp, self.pid_target_temp)
                    elif self.pid_moving_setpoint > self.pid_target_temp:
                        self.pid_moving_setpoint = max(self.pid_moving_setpoint - delta_temp, self.pid_target_temp)
                    
                    # Read Control Channel Temp
                    try:
                        idx = CHANNELS.index(self.control_channel)
                        current_temp = daq_temps[idx]
                    except ValueError:
                        current_temp = None

                    if current_temp is not None:
                        # Calculate PID
                        commanded_current = self.pid_controller.compute(self.pid_moving_setpoint, current_temp, dt)
                        
                        # Command PSU
                        try:
                            with self.psu_lock:
                                if self.psu_protocol == "SCPI":
                                    self._psu_write(self.psu, f"CURR {commanded_current:.3f}")
                                else:
                                    self._psu_query(self.psu, f"PC {commanded_current:.3f}")
                            self.psu_current_setpoint = commanded_current
                            
                            status_text = f"Status: PID Active | Target: {self.pid_moving_setpoint:.1f} °C -> {commanded_current:.2f} A"
                            self.root.after(0, lambda text=status_text: self.lbl_pid_status.config(text=text))
                        except Exception as exc:
                            self.log(f"PID Write Error: {exc}")
                            
            # OR 3. MANUAL RAMPING
            elif self.manual_ramping_active and self.psu and self.psu.is_open:
                now = time.time()
                dt = now - self.last_manual_ramp_time
                self.last_manual_ramp_time = now
                delta_i = self.ramp_rate_A_per_sec * dt

                if self.psu_current_setpoint < self.psu_target_i:
                    self.psu_current_setpoint = min(self.psu_current_setpoint + delta_i, self.psu_target_i)
                elif self.psu_current_setpoint > self.psu_target_i:
                    self.psu_current_setpoint = max(self.psu_current_setpoint - delta_i, self.psu_target_i)

                if self.psu_current_setpoint > self.max_i_limit:
                    self.psu_current_setpoint = self.max_i_limit
                    self.manual_ramping_active = False
                    self.root.after(0, lambda: self.btn_stop_ramp.config(state=tk.DISABLED))
                    
                try:
                    with self.psu_lock:
                        if self.psu_protocol == "SCPI":
                            self._psu_write(self.psu, f"CURR {self.psu_current_setpoint:.3f}")
                        else:
                            self._psu_query(self.psu, f"PC {self.psu_current_setpoint:.3f}")
                except Exception as exc:
                    self.log(f"Ramp write error: {exc}")

                if math.isclose(self.psu_current_setpoint, self.psu_target_i, abs_tol=0.001):
                    self.manual_ramping_active = False
                    self.root.after(0, lambda: self.btn_stop_ramp.config(state=tk.DISABLED))

            # 4. READ PSU VOLTAGE/CURRENT
            if self.psu and self.psu.is_open:
                try:
                    t_psu = time.perf_counter()
                    with self.psu_lock:
                        if self.psu_protocol == "SCPI":
                            v_str = self._psu_query(self.psu, "MEAS:VOLT?", wait_s=0.05)
                            i_str = self._psu_query(self.psu, "MEAS:CURR?", wait_s=0.05)
                        else:
                            v_str = self._psu_query(self.psu, "MV?", wait_s=0.05)
                            i_str = self._psu_query(self.psu, "MC?", wait_s=0.05)
                    v_meas, i_meas = float(v_str), float(i_str)
                    psu_latency = (time.perf_counter() - t_psu) * 1000.0
                    self.root.after(0, lambda v=v_meas, i=i_meas: (self.lbl_v_meas.config(text=f"Meas V: {v:.3f} V"), self.lbl_i_meas.config(text=f"Meas I: {i:.3f} A")))
                except Exception:
                    pass

            # 5. LOGGING
            if self.test_running:
                if self.csv_writer:
                    row = [
                        datetime.now().isoformat(), round(elapsed, 3),
                        v_meas if v_meas is not None else "", i_meas if i_meas is not None else "",
                        self.pid_moving_setpoint if self.pid_active else "", int(self.pid_active),
                        round(psu_latency, 2) if psu_latency is not None else "",
                        round(daq_latency, 2) if daq_latency is not None else ""
                    ]
                    for v, t, hwt in zip(daq_voltages, daq_temps, daq_times):
                        row.extend([v if v is not None else "", t if t is not None else "", hwt if hwt is not None else ""])
                    try:
                        self.csv_writer.writerow(row)
                        self.csv_file.flush()
                    except Exception:
                        pass
                        
                self.t_data.append(elapsed)
                self.v_data.append(v_meas if v_meas is not None else float("nan"))
                self.i_data.append(i_meas if i_meas is not None else float("nan"))
                
                if hasattr(self, 'setpoint_data'):
                    self.setpoint_data.append(self.pid_moving_setpoint if self.pid_active else float("nan"))

                for i, ch in enumerate(CHANNELS):
                    self.temp_data[ch].append(daq_temps[i] if daq_temps[i] is not None else float("nan"))
                self.root.after(0, self.update_plots)

            # 6. TIMING CADENCE
            if self.test_running and self.start_time:
                self.loop_iteration += 1
                sleep_time = (self.start_time + self.loop_iteration * self.poll_interval) - time.time()
                if sleep_time > 0: time.sleep(sleep_time)
            else:
                sleep_time = DEFAULT_POLL_INTERVAL - (time.time() - loop_start)
                if sleep_time > 0: time.sleep(sleep_time)

    def on_closing(self):
        self.polling_active = False
        self.manual_ramping_active = False
        self.pid_active = False
        if self.test_running: self.stop_test()
        with self.psu_lock:
            if self.psu and self.psu.is_open:
                try:
                    if self.psu_protocol == "SCPI":
                        self._psu_write(self.psu, "CURR 0.0")
                        self._psu_write(self.psu, "OUTP OFF")
                        self._psu_write(self.psu, "SYST:SET LOC")
                    else:
                        self._psu_query(self.psu, "PC 0.0")
                        self._psu_query(self.psu, "OUT 0")
                        self._psu_query(self.psu, "RMT 0")
                    self.psu.close()
                except Exception: pass
        with self.daq_lock:
            if self.daq:
                try:
                    self.daq.write(":ABOR")
                    self.daq.close()
                except Exception: pass
        try: self.rm.close()
        except Exception: pass
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = PSUAndThermocoupleApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
