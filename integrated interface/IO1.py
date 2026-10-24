import tkinter as tk
from tkinter import ttk, messagebox
import pyvisa
import serial
import serial.tools.list_ports
import threading
import time
import csv
import math
import re
from datetime import datetime
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

# ============================================================
# HARDCODED SAFETY LIMITS & CONFIG 
# ============================================================
HARDWARE_OVP = 10.5         # Safe OVP ceiling for 0-10V 1000A TDK-Lambda GSP
PSU_BAUD_RATE = 115200

BAUD_RATE = 9600            # Arduino Actuator Baud Rate
SERIAL_RESPONSE_TIMEOUT_SECONDS = 0.8
DEFAULT_POLL_INTERVAL = 1.0
QUINTIC_SAMPLE_INTERVAL_SECONDS = 0.5
GRAPH_REFRESH_MS = 150
GRAPH_HISTORY_SECONDS = 300.0
HANDSHAKE_RESPONSE_SECONDS = 8.0
EXPECTED_DEVICE_ID = "COM_Port_verbunden_HTMREETZ_R1.33"

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


class UnifiedHardwareApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Advanced Hardware Controller & DAQ")
        self.root.geometry("1400x850")
        
        # INDIVIDUAL HARDWARE LOCKS
        self.psu_lock = threading.Lock()
        self.actuator_lock = threading.Lock()
        self.daq_lock = threading.Lock()
        
        # VISA BACKEND FOR USBTMC DAQ6510
        self.rm = pyvisa.ResourceManager()
        
        self.psu = None           # serial.Serial instance (TDK-Lambda GSP)
        self.psu_protocol = "SCPI"
        self.actuator = None      # serial.Serial instance (Arduino)
        self.daq = None           # pyvisa Resource (Keithley DAQ6510)
        
        self.test_running = False
        self.csv_file = None
        self.csv_writer = None
        self.loop_iteration = 0   # Absolute timeline tracker
        self.poll_interval = DEFAULT_POLL_INTERVAL

        # User-defined safety ceilings
        self.max_v_limit = 10.0
        self.max_i_limit = 360.0

        # Ramping States
        self.ramping_active = False
        self.psu_target_i = 0.0
        self.psu_current_setpoint = 0.0
        self.ramp_rate_A_per_sec = 0.0
        self.last_ramp_time = 0.0

        # Actuator state
        self.selected_range = tk.StringVar(value="S1")
        self.current_commanded_speed_um_min = None
        self.motion_state = "PAUSED"          # UP / DOWN / PAUSED
        self.current_position_mm = None
        self.up_locked_until_pause = False
        self.down_locked_until_pause = False

        # Quintic actuator state
        self.quintic_popup = None
        self.quintic_mode_active = False
        self.quintic_running = False
        self.quintic_trajectory = []
        self.quintic_duration_seconds = 0.0
        self.quintic_profile_name = ""
        self.quintic_requested_target_um_min = None
        self.quintic_effective_target_um_min = None
        self.quintic_stop_event = threading.Event()
        self.quintic_thread = None
        self.ramp_start_monotonic = None
        self.quintic_target_var = tk.StringVar()
        self.quintic_unit_var = tk.StringVar(value="mm/min")
        self.quintic_duration_var = tk.StringVar(value="120")

        # Quintic popup graph state (kept separate from integrated main graph)
        self.graph_start_time = time.monotonic()
        self.graph_x = [0.0]
        self.graph_y = [0.0]
        self.position_graph_x = [0.0]
        self.position_graph_y = [0.0]
        self.log_history = []

        # Port lists
        self.available_psu_com = []
        self.available_act_com = []
        self.available_usb_visa = []

        # Plot Data Arrays
        self.t_data = []
        self.v_data = []
        self.i_data = []
        self.pos_data = []
        self.temp_data = {ch: [] for ch in CHANNELS}

        # Thermocouple graph visibility only. Acquisition and CSV logging always
        # continue for all five DAQ channels regardless of these selections.
        self.temp_channel_visible = {
            ch: tk.BooleanVar(value=True) for ch in CHANNELS
        }
        self.start_time = None

        self.scan_ports()
        self.setup_ui()
        self.bind_shortcuts()

        self.quintic_target_var.trace_add("write", lambda *_: self._update_quintic_calculate_state())
        self.quintic_unit_var.trace_add("write", lambda *_: self._update_quintic_calculate_state())
        self.quintic_duration_var.trace_add("write", lambda *_: self._update_quintic_calculate_state())
        self.root.after(GRAPH_REFRESH_MS, self._graph_refresh_tick)
        
        # Start unified polling thread
        self.polling_active = True
        self.poll_thread = threading.Thread(target=self.hardware_poll_loop, daemon=True)
        self.poll_thread.start()

    def scan_ports(self):
        # 1. Scan VISA for USB/TCPIP instruments (Keithley DAQ6510)
        try:
            resources = self.rm.list_resources()
            self.available_usb_visa = [r for r in resources if "USB" in r or "TCPIP" in r]
            print(f"DEBUG VISA Resources: {resources}")
        except Exception as e:
            print(f"VISA Scan Error: {e}")
            self.available_usb_visa = []

        # 2. Scan OS COM ports for PSU (VID:PID 2504:0002 / COM4) and Actuator (Arduino / FTDI / CH340)
        try:
            com_ports = serial.tools.list_ports.comports()
            self.available_psu_com = []
            self.available_act_com = []
            for p in com_ports:
                # Prioritize TDK-Lambda USB-CDC (2504:0002) for PSU list
                if "2504:0002" in p.hwid or p.device == "COM4":
                    self.available_psu_com.insert(0, p.device)
                else:
                    self.available_psu_com.append(p.device)

                # Prioritize Arduino/FTDI/CH340 (and non-PSU ports) for Actuator list
                if "2504:0002" not in p.hwid and (
                    "Arduino" in p.description or "CH340" in p.description
                    or "CP210" in p.description or "0403:6001" in p.hwid
                ):
                    self.available_act_com.insert(0, p.device)
                else:
                    self.available_act_com.append(p.device)

            if not self.available_psu_com:
                self.available_psu_com = ["COM4"]
        except Exception as e:
            print(f"COM Port Scan Error: {e}")
            self.available_psu_com = ["COM4"]
            self.available_act_com = []

    def setup_ui(self):
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # ==================== GENERAL DATA LOGGING ====================
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

        # ==================== HARDWARE CONTROLS ====================
        control_frame = ttk.Frame(main_frame)
        control_frame.pack(fill=tk.X, pady=(0, 10))

        # --- PSU CONTROLS ---
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


        # --- ACTUATOR CONTROLS ---
        act_frame = ttk.LabelFrame(control_frame, text="Arduino Motor Control", padding=10)
        act_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(5, 5))
        for i in range(4):
            act_frame.columnconfigure(i, weight=1)

        ttk.Label(act_frame, text="Port:").grid(row=0, column=0, sticky="w", pady=5)
        self.combo_act_port = ttk.Combobox(act_frame, values=self.available_act_com, width=15)
        if self.available_act_com:
            self.combo_act_port.set(self.available_act_com[0])
        self.combo_act_port.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        self.btn_act_conn = ttk.Button(act_frame, text="Connect Actuator", command=self.connect_actuator_thread)
        self.btn_act_conn.grid(row=0, column=2, columnspan=2, pady=5, sticky="ew")

        self.btn_e1 = ttk.Button(act_frame, text="Enable (E1)", state=tk.DISABLED,
                                 command=lambda: self.send_actuator_command("E1"))
        self.btn_e1.grid(row=1, column=0, columnspan=2, pady=2, sticky="ew", padx=1)
        self.btn_e0 = ttk.Button(act_frame, text="Disable (E0)", state=tk.DISABLED,
                                 command=lambda: self.send_actuator_command("E0"))
        self.btn_e0.grid(row=1, column=2, columnspan=2, pady=2, sticky="ew", padx=1)

        speed_radio_frame = ttk.Frame(act_frame)
        speed_radio_frame.grid(row=2, column=0, columnspan=4, pady=5, sticky="ew")
        self.rb_s1 = ttk.Radiobutton(speed_radio_frame, text="S1 (1-100 µm/min)", variable=self.selected_range, value="S1", state=tk.DISABLED, command=self._update_speed_placeholder)
        self.rb_s1.pack(side=tk.LEFT, expand=True)
        self.rb_s2 = ttk.Radiobutton(speed_radio_frame, text="S2 (100-1000 µm/min)", variable=self.selected_range, value="S2", state=tk.DISABLED, command=self._update_speed_placeholder)
        self.rb_s2.pack(side=tk.LEFT, expand=True)
        self.rb_s3 = ttk.Radiobutton(speed_radio_frame, text="S3 (1-45 mm/min)", variable=self.selected_range, value="S3", state=tk.DISABLED, command=self._update_speed_placeholder)
        self.rb_s3.pack(side=tk.LEFT, expand=True)

        ttk.Label(act_frame, text="Speed:").grid(row=3, column=0, sticky="w", pady=5)
        self.speed_entry = ttk.Entry(act_frame, width=10, state=tk.DISABLED)
        self.speed_entry.grid(row=3, column=1, sticky="ew", pady=5)
        self.speed_entry.insert(0, "50")
        self.btn_send_speed = ttk.Button(act_frame, text="Set Speed", state=tk.DISABLED, command=self.send_speed_command)
        self.btn_send_speed.grid(row=3, column=2, columnspan=2, sticky="ew", padx=1, pady=5)

        self.btn_up = ttk.Button(act_frame, text="▲ UP (U1)", state=tk.DISABLED, command=lambda: self.send_direction_command("U1"))
        self.btn_up.grid(row=4, column=0, sticky="ew", padx=1, pady=5)
        self.btn_pause = ttk.Button(act_frame, text="⏹ PAUSE (N)", state=tk.DISABLED, command=lambda: self.send_actuator_command("N"))
        self.btn_pause.grid(row=4, column=1, sticky="ew", padx=1, pady=5)
        self.btn_down = ttk.Button(act_frame, text="▼ DOWN (D1)", state=tk.DISABLED, command=lambda: self.send_direction_command("D1"))
        self.btn_down.grid(row=4, column=2, sticky="ew", padx=1, pady=5)
        self.btn_zero = ttk.Button(act_frame, text='Pos "0" (Z)', state=tk.DISABLED, command=lambda: self.send_actuator_command("Z"))
        self.btn_zero.grid(row=4, column=3, sticky="ew", padx=1, pady=5)

        self.lbl_act_status = ttk.Label(act_frame, text="Status: Disconnected")
        self.lbl_act_status.grid(row=5, column=0, columnspan=2, pady=5, sticky="w")
        self.lbl_pos = ttk.Label(act_frame, text="Pos: -- mm", font=("Arial", 11, "bold"), foreground="blue")
        self.lbl_pos.grid(row=5, column=2, columnspan=2, pady=5, sticky="e")

        self.btn_enable_quintic = ttk.Button(act_frame, text="Enable Quintic Mode...", state=tk.DISABLED, command=self.open_quintic_popup)
        self.btn_enable_quintic.grid(row=6, column=0, columnspan=4, sticky="ew", pady=(8, 4))

        # Current actuator state is embedded directly in the actuator section.
        main_state = ttk.LabelFrame(act_frame, text="Current Actuator State", padding=6)
        main_state.grid(row=7, column=0, columnspan=4, sticky="ew", pady=(4, 0))
        for i in range(3):
            main_state.columnconfigure(i, weight=1)

        main_speed = ttk.Frame(main_state)
        main_speed.grid(row=0, column=0, sticky="ew", padx=4)
        ttk.Label(main_speed, text="CURRENT APPLIED SPEED", font=("Arial", 8, "bold")).pack(anchor="center")
        self.lbl_main_current_speed = ttk.Label(main_speed, text="0 µm/min", font=("Arial", 13, "bold"))
        self.lbl_main_current_speed.pack(anchor="center")

        main_motion = ttk.Frame(main_state)
        main_motion.grid(row=0, column=1, sticky="ew", padx=4)
        ttk.Label(main_motion, text="MOTION", font=("Arial", 8, "bold")).pack(anchor="center")
        self.lbl_main_motion = ttk.Label(main_motion, text="PAUSED", font=("Arial", 11, "bold"))
        self.lbl_main_motion.pack(anchor="center")

        main_position = ttk.Frame(main_state)
        main_position.grid(row=0, column=2, sticky="ew", padx=4)
        ttk.Label(main_position, text="POSITION", font=("Arial", 8, "bold")).pack(anchor="center")
        self.lbl_main_position = ttk.Label(main_position, text="-- mm", font=("Arial", 11, "bold"))
        self.lbl_main_position.pack(anchor="center")

        # --- DAQ CONTROLS ---
        daq_frame = ttk.LabelFrame(control_frame, text="Keithley DAQ6510", padding=10)
        daq_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 0))

        self.combo_daq_port = ttk.Combobox(daq_frame, values=self.available_usb_visa)
        if self.available_usb_visa:
            self.combo_daq_port.set(self.available_usb_visa[0])
        self.combo_daq_port.pack(fill=tk.X, pady=5)

        self.btn_daq_conn = ttk.Button(daq_frame, text="Connect DAQ", command=self.connect_daq_thread)
        self.btn_daq_conn.pack(fill=tk.X, pady=5)

        self.lbl_daq_status = ttk.Label(daq_frame, text="Status: Disconnected")
        self.lbl_daq_status.pack(pady=5)

        self.lbl_daq_temps = ttk.Label(daq_frame, text="Temps: -- °C", font=("Arial", 11), wraplength=200)
        self.lbl_daq_temps.pack(pady=(8, 5))

        # Graph-only channel filter. These controls never change which channels
        # the DAQ acquires or which values are written to the CSV file.
        channel_filter = ttk.LabelFrame(daq_frame, text="Graph Channels", padding=5)
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
        # Anchor the log before creating the expanding graph.  Keeping it on the
        # bottom prevents the taller actuator section from pushing it off-screen.
        log_frame = ttk.LabelFrame(main_frame, text="System Log", padding=5)
        log_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=(10, 0))

        self.log_text = tk.Text(log_frame, height=6, state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True)

        # ==================== MIDDLE ROW: GRAPHS ====================
        # The graph receives only the space left between the controls and the
        # bottom-anchored System Log.
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

    def bind_shortcuts(self):
        self.root.bind("<Up>", self.on_key_up)
        self.root.bind("<Down>", self.on_key_down)

    def on_key_up(self, event=None):
        if str(self.btn_up["state"]) == tk.NORMAL:
            self.send_direction_command("U1")

    def on_key_down(self, event=None):
        if str(self.btn_down["state"]) == tk.NORMAL:
            self.send_direction_command("D1")

    def log(self, message):
        line = f"[{time.strftime('%H:%M:%S')}] {message}\n"
        self.log_history.append(line)
        if len(self.log_history) > 1000:
            self.log_history = self.log_history[-1000:]
        def append():
            self._append_to_text_widget(self.log_text, line)
            if self._popup_exists() and hasattr(self, "popup_log_text"):
                self._append_to_text_widget(self.popup_log_text, line)
        self.root.after(0, append)

    @staticmethod
    def _append_to_text_widget(widget, line):
        try:
            widget.config(state=tk.NORMAL); widget.insert(tk.END, line); widget.see(tk.END); widget.config(state=tk.DISABLED)
        except tk.TclError:
            pass

    # ------------------- PSU PYSERIAL METHODS -------------------
    def _psu_write(self, ser_inst, cmd):
        """Writes command with \r\n termination over PySerial."""
        ser_inst.write(f"{cmd}\r\n".encode("ascii"))
        ser_inst.flush()

    def _psu_query(self, ser_inst, cmd, wait_s=0.08):
        """Writes command and reads back until \r or timeout."""
        ser_inst.reset_input_buffer()
        self._psu_write(ser_inst, cmd)
        time.sleep(wait_s)
        raw = ser_inst.read_until(b"\r")
        if ser_inst.in_waiting > 0:
            raw += ser_inst.read(ser_inst.in_waiting)
        return raw.decode("ascii", errors="replace").strip()

    def check_psu_Command(self, ser_inst, cmd):
        """Sends a command to the PSU and checks for immediate reply or SCPI error."""
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
                    dsrdtr=False
                )
                # Assert DTR & RTS required by TDK-Lambda Genesys+ USB CDC (2504:0002)
                temp_ser.dtr = True
                temp_ser.rts = True
                time.sleep(0.15)
                temp_ser.reset_input_buffer()
                temp_ser.reset_output_buffer()

                connected_idn = None
                self.psu_protocol = None

                for addr in [6, 1, 0]:
                    # 1. Try SCPI Mode (Genesys+ GSP)
                    self._psu_write(temp_ser, f"INST:NSEL {addr}")
                    time.sleep(0.1)
                    temp_ser.reset_input_buffer()
                    idn = self._psu_query(temp_ser, "*IDN?", wait_s=0.15)
                    if idn:
                        connected_idn = idn
                        self.psu_protocol = "SCPI"
                        self.log(f"✅ PSU Connected (SCPI Addr {addr}): {connected_idn}")
                        break

                    # 2. Try GEN Mode
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
                        "OUTP ON"
                    ]:
                        self.check_psu_Command(temp_ser, cmd)
                else:
                    for cmd in [
                        "RMT 1",
                        f"OVP {HARDWARE_OVP}",
                        "PV 0.0",
                        "PC 0.0",
                        "OUT 1"
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
                self.combo_psu_port.config(state=tk.NORMAL)
            ])

    def start_current_ramp(self):
        if not self.psu:
            return
        try:
            self.max_v_limit = float(self.entry_max_v.get())
            self.max_i_limit = float(self.entry_max_i.get())
            target_v = float(self.entry_target_v.get())
            target_i = float(self.entry_target_i.get())
            ramp_rate_A_min = float(self.entry_ramp_rate.get())

            if target_v > self.max_v_limit or target_i > self.max_i_limit:
                messagebox.showerror(
                    "Limit Error",
                    f"Targets exceed maximum stored limits ({self.max_v_limit}V, {self.max_i_limit}A)."
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
            self.root.after(0, lambda: self.btn_stop_ramp.config(state=tk.DISABLED))
            self.log("Ramp stopped: Hit max current limit.")

        try:
            with self.psu_lock:
                if self.psu_protocol == "SCPI":
                    self._psu_write(self.psu, f"CURR {self.psu_current_setpoint:.3f}")
                else:
                    self._psu_query(self.psu, f"PC {self.psu_current_setpoint:.3f}")
        except Exception as e:
            self.log(f"Ramp write error: {e}")

        if math.isclose(self.psu_current_setpoint, self.psu_target_i, abs_tol=0.001):
            self.ramping_active = False
            self.root.after(0, lambda: self.btn_stop_ramp.config(state=tk.DISABLED))
            self.log(f"Ramp complete. Reached target {self.psu_target_i} A.")

    # ------------------- ACTUATOR METHODS (NEW MODULE) -------------------
    def connect_actuator_thread(self):
        selected_port = self.combo_act_port.get().strip()
        if not selected_port:
            messagebox.showerror("Error", "Please select a COM port for the actuator.")
            return

        self.btn_act_conn.config(state=tk.DISABLED)
        self.combo_act_port.config(state=tk.DISABLED)
        self.lbl_act_status.config(text="Status: Connecting & Resetting...")
        threading.Thread(target=self._connect_actuator, args=(selected_port,), daemon=True).start()

    def _connect_actuator(self, port):
        temp_serial = None
        try:
            # Open the selected COM port. On the Nano this normally triggers a
            # reset, causing setup() to run and emit EXPECTED_DEVICE_ID.
            temp_serial = serial.Serial(
                port,
                BAUD_RATE,
                timeout=0.25,
                write_timeout=1.0,
            )

            # Clear only immediately after opening the port. This removes bytes
            # left over from an earlier session while still giving the freshly
            # resetting Nano time to send its new startup identification later.
            temp_serial.reset_input_buffer()
            temp_serial.reset_output_buffer()

            self.log(
                f"Port {port} opened. Waiting for actuator identification "
                f"'{EXPECTED_DEVICE_ID}'..."
            )

            deadline = time.monotonic() + HANDSHAKE_RESPONSE_SECONDS
            identified = False
            received_lines = []

            while time.monotonic() < deadline:
                raw = temp_serial.readline()
                if not raw:
                    continue

                text = raw.decode("ascii", errors="replace").strip()
                if not text:
                    continue

                received_lines.append(text)
                self.log(f"Handshake RX: {text!r}")

                if text == EXPECTED_DEVICE_ID:
                    identified = True
                    break

            if not identified:
                seen = (
                    " | ".join(received_lines[-5:])
                    if received_lines
                    else "no identification received"
                )
                self.log(
                    f"Wrong device or unexpected firmware on {port}. "
                    f"Expected '{EXPECTED_DEVICE_ID}', received: {seen}"
                )
                temp_serial.close()
                self.root.after(0, self._connection_failed_ui)
                return

            # Do not clear the input buffer again here. Anything arriving after
            # the accepted handshake may be a legitimate firmware response.
            temp_serial.timeout = SERIAL_RESPONSE_TIMEOUT_SECONDS

            with self.actuator_lock:
                self.actuator = temp_serial

            def update_ui():
                self.btn_act_conn.config(text="Actuator Connected")
                self.lbl_act_status.config(text="Status: Ready / Verified")
                for w in (
                    self.btn_e1, self.btn_e0, self.rb_s1, self.rb_s2, self.rb_s3,
                    self.speed_entry, self.btn_send_speed, self.btn_up, self.btn_pause,
                    self.btn_down, self.btn_zero, self.btn_enable_quintic,
                ):
                    w.config(state=tk.NORMAL)
                self._sync_normal_motion_button_states()
                self._refresh_state_views()
                self.check_enable_test()

            self.root.after(0, update_ui)
            self.log(
                f"Actuator verified on {port}: received expected startup ID. "
                "Connection accepted."
            )

        except Exception as exc:
            if temp_serial is not None:
                try:
                    temp_serial.close()
                except Exception:
                    pass
            self.log(f"Actuator connection failed: {exc}")
            self.root.after(0, self._connection_failed_ui)

    def _connection_failed_ui(self):
        self.actuator = None
        self.btn_act_conn.config(state=tk.NORMAL, text="Connect Actuator")
        self.combo_act_port.config(state=tk.NORMAL)
        self.lbl_act_status.config(text="Status: Disconnected")

    def send_actuator_command(self, cmd):
        """Single serial write path used by normal controls and quintic mode."""
        if not (self.actuator and self.actuator.is_open):
            self.log(f"Command blocked; actuator is not connected: {cmd}")
            return False

        try:
            with self.actuator_lock:
                self.actuator.write(f"{cmd}\n".encode("utf-8"))
                self.actuator.flush()
            self.log(f"Sent to Actuator: {cmd}")
            self._apply_command_to_local_state(cmd)
            return True
        except Exception as exc:
            self.log(f"Actuator Write Error ({cmd}): {exc}")
            return False

    def send_direction_command(self, cmd):
        """Normal-mode direction command with a hard re-click guard until N."""
        if cmd == "U1" and self.up_locked_until_pause:
            self.log("UP command ignored: UP remains locked until Pause (N) is pressed.")
            return False
        if cmd == "D1" and self.down_locked_until_pause:
            self.log("DOWN command ignored: DOWN remains locked until Pause (N) is pressed.")
            return False
        return self.send_actuator_command(cmd)

    def _apply_command_to_local_state(self, cmd):
        if cmd == "U1":
            self.motion_state = "UP"
            self.up_locked_until_pause = True
        elif cmd == "D1":
            self.motion_state = "DOWN"
            self.down_locked_until_pause = True
        elif cmd == "N":
            self.motion_state = "PAUSED"
            self.up_locked_until_pause = False
            self.down_locked_until_pause = False
        elif cmd == "E0":
            self.motion_state = "PAUSED"
        elif cmd == "Z":
            self.current_position_mm = 0.0
        self.root.after(0, self._refresh_state_views)

    def _sync_normal_motion_button_states(self):
        """Prevent re-selecting the active direction until Pause (N) is pressed."""
        connected = bool(self.actuator and self.actuator.is_open)
        controls_available = connected and not self.quintic_mode_active
        if not controls_available:
            self.btn_up.config(state=tk.DISABLED)
            self.btn_down.config(state=tk.DISABLED)
            self.btn_pause.config(state=tk.DISABLED)
            return

        self.btn_pause.config(state=tk.NORMAL)
        self.btn_up.config(state=tk.DISABLED if self.up_locked_until_pause else tk.NORMAL)
        self.btn_down.config(state=tk.DISABLED if self.down_locked_until_pause else tk.NORMAL)

    def _refresh_state_views(self):
        applied = self._get_applied_speed_um_min()
        pos_text = "-- mm" if self.current_position_mm is None else f"{self.current_position_mm:.3f} mm"

        if self.current_position_mm is not None:
            self.lbl_pos.config(text=f"Pos: {self.current_position_mm:.3f} mm")
        self.lbl_main_current_speed.config(text=self._format_speed(applied))
        self.lbl_main_motion.config(text=self.motion_state)
        self.lbl_main_position.config(text=pos_text)
        self._sync_normal_motion_button_states()
        self._update_popup_state()

    def _update_speed_placeholder(self):
        r = self.selected_range.get()
        self.speed_entry.delete(0, tk.END)
        self.speed_entry.insert(0, {"S1": "50", "S2": "500", "S3": "20"}[r])

    def send_speed_command(self):
        val_str = self.speed_entry.get().strip()
        if not val_str.isdigit():
            messagebox.showwarning("Invalid Input", "Please enter a valid positive integer.")
            return

        val = int(val_str)
        r = self.selected_range.get()
        if r == "S1" and not (1 <= val <= 100):
            messagebox.showwarning("Out of Range", "Range S1 requires values between 1 and 100 µm/min.")
            return
        if r == "S2" and not (100 <= val <= 1000):
            messagebox.showwarning("Out of Range", "Range S2 requires values between 100 and 1000 µm/min.")
            return
        if r == "S3" and not (1 <= val <= 45):
            messagebox.showwarning("Out of Range", "Range S3 requires values between 1 and 45 mm/min.")
            return

        cmd = f"{r}{val}"
        if self.send_actuator_command(cmd):
            self.current_commanded_speed_um_min = float(val if r in ("S1", "S2") else val * 1000)
            self.log(f"Stored commanded speed: {self._format_speed(self.current_commanded_speed_um_min)}")
            self._update_popup_state()

    def _popup_exists(self):
        try:
            return self.quintic_popup is not None and self.quintic_popup.winfo_exists()
        except tk.TclError:
            return False

    def open_quintic_popup(self):
        if not (self.actuator and self.actuator.is_open):
            messagebox.showerror("Actuator Not Connected", "Connect the actuator before entering Quintic Mode.")
            return

        if self.current_commanded_speed_um_min is None:
            messagebox.showerror(
                "Current Speed Unknown",
                "Set the actuator speed once in normal mode before entering Quintic Mode."
            )
            return

        if self._popup_exists():
            self.quintic_popup.lift()
            self.quintic_popup.focus_force()
            return

        self.quintic_mode_active = True
        self.quintic_trajectory = []
        self._set_normal_motion_controls_enabled(False)
        self.btn_enable_quintic.config(state=tk.DISABLED)

        popup = tk.Toplevel(self.root)
        self.quintic_popup = popup
        popup.title("Quintic Speed Ramp Control")
        popup.transient(self.root)
        popup.protocol("WM_DELETE_WINDOW", self.close_quintic_popup)

        screen_w = popup.winfo_screenwidth()
        screen_h = popup.winfo_screenheight()
        width = min(1180, max(980, screen_w - 100))
        height = min(760, max(640, screen_h - 120))
        popup.geometry(f"{width}x{height}")
        popup.minsize(940, 620)

        outer = ttk.Frame(popup, padding=8)
        outer.pack(fill=tk.BOTH, expand=True)
        outer.columnconfigure(0, weight=5, minsize=400)
        outer.columnconfigure(1, weight=7, minsize=520)
        outer.rowconfigure(0, weight=1)

        left = ttk.Frame(outer)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 5))
        right = ttk.Frame(outer)
        right.grid(row=0, column=1, sticky="nsew", padx=(5, 0))
        right.columnconfigure(0, weight=1)
        right.columnconfigure(1, weight=1)
        right.rowconfigure(0, weight=0)
        right.rowconfigure(1, weight=5)
        right.rowconfigure(2, weight=2)
        right.rowconfigure(3, weight=2)

        self._build_quintic_controls(left)
        self._build_actuator_state_bar(right)
        self._build_popup_graphs(right)
        self._build_trajectory_table(right)
        self._build_popup_log(right)

        self._clear_trajectory_table()
        self._update_quintic_calculate_state()
        self._update_popup_state()
        self.log("Entered Quintic Mode. Normal speed/direction/pause controls disabled.")

    def _build_quintic_controls(self, parent):
        frame = ttk.LabelFrame(parent, text="Quintic Speed Ramp", padding=10)
        frame.pack(fill=tk.X, expand=False)
        for i in range(4):
            frame.columnconfigure(i, weight=1)

        ttk.Label(frame, text="Current stored speed:").grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 6))
        self.lbl_popup_start_speed = ttk.Label(frame, text=self._format_speed(self.current_commanded_speed_um_min),
                                               font=("Arial", 11, "bold"))
        self.lbl_popup_start_speed.grid(row=0, column=2, columnspan=2, sticky="e", pady=(0, 6))

        ttk.Label(frame, text="Target Speed:").grid(row=1, column=0, sticky="w", pady=3)
        self.entry_quintic_target = ttk.Entry(frame, textvariable=self.quintic_target_var)
        self.entry_quintic_target.grid(row=1, column=1, sticky="ew", padx=2, pady=3)
        self.combo_quintic_unit = ttk.Combobox(frame, textvariable=self.quintic_unit_var,
                                               values=["µm/min", "mm/min"], state="readonly", width=10)
        self.combo_quintic_unit.grid(row=1, column=2, columnspan=2, sticky="ew", padx=2, pady=3)

        ttk.Label(frame, text="Ramp Duration (s):").grid(row=2, column=0, sticky="w", pady=3)
        self.entry_quintic_duration = ttk.Entry(frame, textvariable=self.quintic_duration_var)
        self.entry_quintic_duration.grid(row=2, column=1, sticky="ew", padx=2, pady=3)
        self.btn_quintic_calculate = ttk.Button(frame, text="Set & Calculate",
                                                state=tk.DISABLED, command=self.calculate_quintic_trajectory)
        self.btn_quintic_calculate.grid(row=2, column=2, columnspan=2, sticky="ew", padx=2, pady=3)

        self.lbl_quintic_validation = ttk.Label(frame, text="Enter target speed and duration.", wraplength=390)
        self.lbl_quintic_validation.grid(row=3, column=0, columnspan=4, sticky="w", pady=(4, 3))

        self.lbl_quintic_summary = ttk.Label(
            frame,
            text=f"Start speed: {self._format_speed(self.current_commanded_speed_um_min)}",
            justify=tk.LEFT,
            wraplength=390,
        )
        self.lbl_quintic_summary.grid(row=4, column=0, columnspan=4, sticky="w", pady=(2, 6))

        self.btn_quintic_up = ttk.Button(frame, text="▲ Ramp Upward", state=tk.DISABLED,
                                         command=lambda: self.start_quintic_ramp("U1"))
        self.btn_quintic_up.grid(row=5, column=0, columnspan=2, sticky="ew", padx=2, pady=3)
        self.btn_quintic_down = ttk.Button(frame, text="▼ Ramp Downward", state=tk.DISABLED,
                                           command=lambda: self.start_quintic_ramp("D1"))
        self.btn_quintic_down.grid(row=5, column=2, columnspan=2, sticky="ew", padx=2, pady=3)

        self.btn_quintic_pause = ttk.Button(frame, text="⏹ Pause Ramp (N)", state=tk.DISABLED,
                                            command=self.pause_quintic_ramp)
        self.btn_quintic_pause.grid(row=6, column=0, columnspan=2, sticky="ew", padx=2, pady=3)
        self.btn_quintic_cancel = ttk.Button(frame, text="Cancel Ramp (Keep Moving)", state=tk.DISABLED,
                                             command=self.cancel_quintic_ramp)
        self.btn_quintic_cancel.grid(row=6, column=2, columnspan=2, sticky="ew", padx=2, pady=3)

        ttk.Separator(frame, orient=tk.HORIZONTAL).grid(row=7, column=0, columnspan=4, sticky="ew", pady=8)
        self.btn_exit_quintic = ttk.Button(frame, text="Exit Quintic Mode", command=self.close_quintic_popup)
        self.btn_exit_quintic.grid(row=8, column=0, columnspan=4, sticky="ew", pady=(0, 2))

    def _build_actuator_state_bar(self, parent):
        frame = ttk.LabelFrame(parent, text="Current Actuator State", padding=6)
        frame.grid(row=0, column=0, columnspan=2, sticky="ew", pady=(0, 5))
        for i in range(3):
            frame.columnconfigure(i, weight=1)

        speed = ttk.Frame(frame)
        speed.grid(row=0, column=0, sticky="ew", padx=6)
        ttk.Label(speed, text="CURRENT APPLIED SPEED", font=("Arial", 9, "bold")).pack(anchor="center")
        self.lbl_current_speed_large = ttk.Label(speed, text="--", font=("Arial", 18, "bold"))
        self.lbl_current_speed_large.pack(anchor="center")

        motion = ttk.Frame(frame)
        motion.grid(row=0, column=1, sticky="ew", padx=6)
        ttk.Label(motion, text="MOTION", font=("Arial", 9, "bold")).pack(anchor="center")
        self.lbl_motion_large = ttk.Label(motion, text="PAUSED", font=("Arial", 14, "bold"))
        self.lbl_motion_large.pack(anchor="center")

        position = ttk.Frame(frame)
        position.grid(row=0, column=2, sticky="ew", padx=6)
        ttk.Label(position, text="POSITION", font=("Arial", 9, "bold")).pack(anchor="center")
        self.lbl_position_large = ttk.Label(position, text="-- mm", font=("Arial", 14, "bold"))
        self.lbl_position_large.pack(anchor="center")

    def _build_popup_graphs(self, parent):
        speed_frame = ttk.LabelFrame(parent, text="Live Applied Speed", padding=4)
        speed_frame.grid(row=1, column=0, sticky="nsew", padx=(0, 3), pady=(0, 5))
        speed_toolbar = ttk.Frame(speed_frame)
        speed_toolbar.pack(fill=tk.X)
        self.lbl_graph_speed = ttk.Label(speed_toolbar, text="Applied speed: 0 mm/min")
        self.lbl_graph_speed.pack(side=tk.LEFT, padx=(2, 10), pady=1)

        self.figure = Figure(figsize=(3.4, 2.5), dpi=90)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_xlabel("Elapsed time (s)")
        self.ax.set_ylabel("Speed (mm/min)")
        self.ax.grid(True, alpha=0.3)
        self.quintic_speed_canvas = FigureCanvasTkAgg(self.figure, master=speed_frame)
        self.quintic_speed_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        position_frame = ttk.LabelFrame(parent, text="Position Over Time", padding=4)
        position_frame.grid(row=1, column=1, sticky="nsew", padx=(3, 0), pady=(0, 5))
        toolbar = ttk.Frame(position_frame)
        toolbar.pack(fill=tk.X)
        self.lbl_graph_position = ttk.Label(toolbar, text="Position: -- mm")
        self.lbl_graph_position.pack(side=tk.LEFT, padx=(2, 10), pady=1)
        ttk.Button(toolbar, text="Clear Both Graphs", command=self.clear_graph).pack(side=tk.RIGHT, padx=2, pady=1)

        self.position_figure = Figure(figsize=(3.4, 2.5), dpi=90)
        self.position_ax = self.position_figure.add_subplot(111)
        self.position_ax.set_xlabel("Elapsed time (s)")
        self.position_ax.set_ylabel("Position (mm)")
        self.position_ax.grid(True, alpha=0.3)
        self.position_canvas = FigureCanvasTkAgg(self.position_figure, master=position_frame)
        self.position_canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _build_trajectory_table(self, parent):
        frame = ttk.LabelFrame(parent, text="Calculated Quintic Trajectory", padding=4)
        frame.grid(row=2, column=0, columnspan=2, sticky="nsew", pady=(0, 5))
        holder = ttk.Frame(frame)
        holder.pack(fill=tk.BOTH, expand=True)

        self.trajectory_table = ttk.Treeview(holder, columns=("time", "command", "speed"),
                                              show="headings", height=4)
        self.trajectory_table.heading("time", text="Time (s)")
        self.trajectory_table.heading("command", text="Actuator Command")
        self.trajectory_table.heading("speed", text="Effective Speed")
        self.trajectory_table.column("time", width=80, anchor="center")
        self.trajectory_table.column("command", width=120, anchor="center")
        self.trajectory_table.column("speed", width=140, anchor="center")
        scroll = ttk.Scrollbar(holder, orient=tk.VERTICAL, command=self.trajectory_table.yview)
        self.trajectory_table.configure(yscrollcommand=scroll.set)
        self.trajectory_table.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll.pack(side=tk.RIGHT, fill=tk.Y)

    def _build_popup_log(self, parent):
        frame = ttk.LabelFrame(parent, text="System Log", padding=4)
        frame.grid(row=3, column=0, columnspan=2, sticky="nsew")
        holder = ttk.Frame(frame)
        holder.pack(fill=tk.BOTH, expand=True)
        self.popup_log_text = tk.Text(holder, height=7, state=tk.DISABLED, wrap=tk.WORD)
        scroll = ttk.Scrollbar(holder, orient=tk.VERTICAL, command=self.popup_log_text.yview)
        self.popup_log_text.configure(yscrollcommand=scroll.set)
        self.popup_log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scroll.pack(side=tk.RIGHT, fill=tk.Y)

        self.popup_log_text.config(state=tk.NORMAL)
        for line in self.log_history:
            self.popup_log_text.insert(tk.END, line)
        self.popup_log_text.see(tk.END)
        self.popup_log_text.config(state=tk.DISABLED)

    def close_quintic_popup(self):
        if self.quintic_running:
            messagebox.showwarning(
                "Ramp Running",
                "Pause or cancel the active quintic ramp before closing Quintic Mode.",
                parent=self.quintic_popup,
            )
            return

        self.quintic_mode_active = False
        self.quintic_trajectory = []
        self._set_normal_motion_controls_enabled(True)
        self.btn_enable_quintic.config(state=tk.NORMAL if self.actuator and self.actuator.is_open else tk.DISABLED)

        if self._popup_exists():
            self.quintic_popup.destroy()
        self.quintic_popup = None
        self.log("Exited Quintic Mode. Normal controls restored.")

    def _set_normal_motion_controls_enabled(self, enabled):
        connected = bool(self.actuator and self.actuator.is_open)
        state = tk.NORMAL if enabled and connected else tk.DISABLED
        for widget in (self.rb_s1, self.rb_s2, self.rb_s3, self.speed_entry, self.btn_send_speed):
            try:
                widget.config(state=state)
            except tk.TclError:
                pass

        if enabled:
            self._sync_normal_motion_button_states()
        else:
            for widget in (self.btn_up, self.btn_pause, self.btn_down):
                try:
                    widget.config(state=tk.DISABLED)
                except tk.TclError:
                    pass

    def _format_speed(speed_um_min):
        if speed_um_min is None:
            return "--"
        if speed_um_min >= 1000:
            return f"{speed_um_min / 1000.0:g} mm/min"
        return f"{speed_um_min:g} µm/min"

    def _parse_quintic_inputs(self):
        target_text = self.quintic_target_var.get().strip()
        duration_text = self.quintic_duration_var.get().strip()
        unit = self.quintic_unit_var.get().strip()

        if not target_text:
            return False, "Enter a target speed.", None
        try:
            target_value = float(target_text)
        except ValueError:
            return False, "Target speed must be numeric.", None
        if not math.isfinite(target_value):
            return False, "Target speed must be finite.", None

        if unit == "µm/min":
            if not (1 <= target_value <= 1000):
                return False, "Target must be between 1 and 1000 µm/min.", None
            target_um_min = target_value
        elif unit == "mm/min":
            if not (1 <= target_value <= 45):
                return False, "Target must be between 1 and 45 mm/min.", None
            target_um_min = target_value * 1000.0
        else:
            return False, "Select a valid speed unit.", None

        if not duration_text:
            return False, "Enter a ramp duration.", None
        try:
            duration = float(duration_text)
        except ValueError:
            return False, "Ramp duration must be numeric.", None
        if not math.isfinite(duration) or duration <= 0:
            return False, "Ramp duration must be greater than 0 seconds.", None

        return True, "", (target_um_min, duration)

    def _update_quintic_calculate_state(self):
        if not self._popup_exists() or not hasattr(self, "btn_quintic_calculate"):
            return
        if not self.quintic_mode_active or self.quintic_running:
            self.btn_quintic_calculate.config(state=tk.DISABLED)
            return

        valid, message, _ = self._parse_quintic_inputs()
        if self.current_commanded_speed_um_min is None:
            valid = False
            message = "Current commanded speed is unknown."

        inputs_locked = str(self.entry_quintic_target["state"]) == tk.DISABLED
        self.btn_quintic_calculate.config(state=tk.NORMAL if valid and not inputs_locked else tk.DISABLED)
        self.lbl_quintic_validation.config(
            text="Inputs valid. Ready to calculate trajectory." if valid and not inputs_locked else message
        )

    def _quintic_blend(u):
        return 10 * u**3 - 15 * u**4 + 6 * u**5

    def _round_half_up_positive(value):
        return int(math.floor(value + 0.5))

    def _speed_to_supported_command(self, desired_um_min):
        desired_um_min = max(1.0, min(45000.0, float(desired_um_min)))
        if desired_um_min <= 100.0:
            value = max(1, min(100, self._round_half_up_positive(desired_um_min)))
            return f"S1{value}", float(value)
        if desired_um_min <= 1000.0:
            value = max(100, min(1000, self._round_half_up_positive(desired_um_min)))
            return f"S2{value}", float(value)

        value_mm = max(1, min(45, self._round_half_up_positive(desired_um_min / 1000.0)))
        return f"S3{value_mm}", float(value_mm * 1000)

    def calculate_quintic_trajectory(self):
        valid, message, values = self._parse_quintic_inputs()
        if not valid:
            messagebox.showerror("Invalid Quintic Input", message, parent=self.quintic_popup)
            self.log(f"Quintic calculation rejected: {message}")
            return

        target_um_min, duration = values
        start_um_min = float(self.current_commanded_speed_um_min)
        if math.isclose(target_um_min, start_um_min, rel_tol=0.0, abs_tol=1e-9):
            messagebox.showinfo("No Ramp Required", "Target speed equals current commanded speed.",
                                parent=self.quintic_popup)
            return

        profile_name = "Acceleration" if target_um_min > start_um_min else "Deceleration"
        times = []
        t = 0.0
        while t < duration:
            times.append(round(t, 9))
            t += QUINTIC_SAMPLE_INTERVAL_SECONDS
        if not times or not math.isclose(times[-1], duration, rel_tol=0.0, abs_tol=1e-9):
            times.append(duration)

        trajectory = []
        last_effective_speed = None
        for t in times:
            u = min(1.0, max(0.0, t / duration))
            desired = start_um_min + (target_um_min - start_um_min) * self._quintic_blend(u)
            cmd, effective = self._speed_to_supported_command(desired)
            if last_effective_speed is None or effective != last_effective_speed:
                trajectory.append({
                    "time_s": float(t),
                    "command": cmd,
                    "effective_um_min": effective,
                    "desired_um_min": desired,
                })
                last_effective_speed = effective

        target_cmd, effective_target = self._speed_to_supported_command(target_um_min)
        if trajectory[-1]["effective_um_min"] != effective_target:
            trajectory.append({
                "time_s": duration,
                "command": target_cmd,
                "effective_um_min": effective_target,
                "desired_um_min": target_um_min,
            })

        self.quintic_trajectory = trajectory
        self.quintic_duration_seconds = duration
        self.quintic_profile_name = profile_name
        self.quintic_requested_target_um_min = target_um_min
        self.quintic_effective_target_um_min = effective_target

        self.entry_quintic_target.config(state=tk.DISABLED)
        self.combo_quintic_unit.config(state=tk.DISABLED)
        self.entry_quintic_duration.config(state=tk.DISABLED)
        self.btn_quintic_calculate.config(state=tk.DISABLED)
        self.btn_quintic_up.config(state=tk.NORMAL)
        self.btn_quintic_down.config(state=tk.NORMAL)

        self.lbl_quintic_validation.config(text="Trajectory ready. Choose upward or downward motion.")
        self.lbl_quintic_summary.config(
            text=(
                f"Start: {self._format_speed(start_um_min)} | Requested target: {self._format_speed(target_um_min)}\n"
                f"Effective target: {self._format_speed(effective_target)} | Duration: {duration:g} s | {profile_name}\n"
                f"Stored speed changes: {len(trajectory)}"
            )
        )
        self._populate_trajectory_table()
        self.log(
            f"Quintic trajectory calculated: {profile_name}, "
            f"{self._format_speed(start_um_min)} -> {self._format_speed(target_um_min)}, "
            f"duration {duration:g}s, {len(trajectory)} unique S commands."
        )

    def _clear_trajectory_table(self):
        if not self._popup_exists() or not hasattr(self, "trajectory_table"):
            return
        for item in self.trajectory_table.get_children():
            self.trajectory_table.delete(item)

    def _populate_trajectory_table(self):
        self._clear_trajectory_table()
        for point in self.quintic_trajectory:
            self.trajectory_table.insert(
                "", tk.END,
                values=(f"{point['time_s']:.3f}", point["command"], self._format_speed(point["effective_um_min"]))
            )

    def start_quintic_ramp(self, direction_cmd):
        if self.quintic_running or not self.quintic_trajectory:
            return

        self.quintic_running = True
        self.quintic_stop_event.clear()
        self.btn_quintic_up.config(state=tk.DISABLED)
        self.btn_quintic_down.config(state=tk.DISABLED)
        self.btn_quintic_pause.config(state=tk.NORMAL)
        self.btn_quintic_cancel.config(state=tk.NORMAL)
        self.btn_exit_quintic.config(state=tk.DISABLED)

        direction_name = "UP" if direction_cmd == "U1" else "DOWN"
        self.lbl_quintic_validation.config(text=f"Ramp running {direction_name}. Pause or Cancel remain available.")
        self.log(f"Quintic ramp start requested in direction {direction_name}.")

        self.quintic_thread = threading.Thread(
            target=self._quintic_execution_loop,
            args=(direction_cmd,),
            daemon=True,
        )
        self.quintic_thread.start()

    def _apply_quintic_speed(self, point):
        if self.quintic_stop_event.is_set():
            return False
        if not self.send_actuator_command(point["command"]):
            return False
        self.current_commanded_speed_um_min = point["effective_um_min"]
        self.root.after(0, self._update_popup_state)
        return True

    def _quintic_execution_loop(self, direction_cmd):
        reason = "error"
        try:
            trajectory = list(self.quintic_trajectory)
            duration = float(self.quintic_duration_seconds)

            # No preload: apply the first scheduled S command, then direction once.
            if not self._apply_quintic_speed(trajectory[0]):
                reason = "cancelled" if self.quintic_stop_event.is_set() else "error"
                return

            if not self.send_actuator_command(direction_cmd):
                reason = "error"
                return

            direction_name = "UP" if direction_cmd == "U1" else "DOWN"
            self.log(f"Quintic ramp running {direction_name}; subsequent updates send S commands only.")

            self.ramp_start_monotonic = time.monotonic()
            next_index = 1

            while not self.quintic_stop_event.is_set():
                elapsed = time.monotonic() - self.ramp_start_monotonic

                if elapsed >= duration:
                    if self.current_commanded_speed_um_min != self.quintic_effective_target_um_min:
                        final_cmd, final_eff = self._speed_to_supported_command(self.quintic_effective_target_um_min)
                        final_point = {
                            "time_s": duration,
                            "command": final_cmd,
                            "effective_um_min": final_eff,
                            "desired_um_min": self.quintic_requested_target_um_min,
                        }
                        if not self._apply_quintic_speed(final_point):
                            reason = "cancelled" if self.quintic_stop_event.is_set() else "error"
                            return
                    reason = "completed"
                    return

                # If serial polling temporarily occupied the lock, apply only the newest
                # trajectory point that is due. This prevents cumulative schedule drift.
                latest_due_index = None
                scan_index = next_index
                while scan_index < len(trajectory) and trajectory[scan_index]["time_s"] <= elapsed:
                    latest_due_index = scan_index
                    scan_index += 1

                if latest_due_index is not None:
                    point = trajectory[latest_due_index]
                    if not self._apply_quintic_speed(point):
                        reason = "cancelled" if self.quintic_stop_event.is_set() else "error"
                        return
                    next_index = latest_due_index + 1
                    continue

                next_due = trajectory[next_index]["time_s"] if next_index < len(trajectory) else duration
                self.quintic_stop_event.wait(max(0.01, min(0.05, next_due - elapsed)))

            reason = "cancelled"
        except Exception as exc:
            self.log(f"Quintic execution error: {exc}")
            reason = "error"
        finally:
            self.root.after(0, lambda r=reason: self._finish_quintic_execution(r))

    def cancel_quintic_ramp(self):
        if not self.quintic_running:
            return
        self.log("Cancel Ramp requested. Future quintic S commands will stop; motor keeps current speed/direction.")
        self.quintic_stop_event.set()

    def pause_quintic_ramp(self):
        if not self.quintic_running:
            return
        self.log("Pause Ramp requested.")
        self.quintic_stop_event.set()
        self.send_actuator_command("N")
        self.log("Quintic schedule cancelled and actuator paused with N.")

    def _finish_quintic_execution(self, reason):
        if not self.quintic_running:
            return
        self.quintic_running = False

        if reason == "completed":
            self.log(
                "Quintic trajectory completed. Final command sent; motor continues in the same "
                f"direction at {self._format_speed(self.current_commanded_speed_um_min)}."
            )
            self._refresh_state_views()
            # Successful completion exits Quintic Mode automatically.  A short
            # UI-cycle delay lets the completion message be appended first.
            self.root.after(100, self.close_quintic_popup)
            return

        if reason == "cancelled":
            self.log(
                "Quintic ramp terminated. Current commanded speed remains "
                f"{self._format_speed(self.current_commanded_speed_um_min)}."
            )
        else:
            self.log("Quintic ramp ended because of an execution error.")

        # Cancel/error leaves the popup available for inspection or another ramp.
        self.quintic_trajectory = []
        if self._popup_exists():
            self.entry_quintic_target.config(state=tk.NORMAL)
            self.combo_quintic_unit.config(state="readonly")
            self.entry_quintic_duration.config(state=tk.NORMAL)
            self.btn_quintic_up.config(state=tk.DISABLED)
            self.btn_quintic_down.config(state=tk.DISABLED)
            self.btn_quintic_pause.config(state=tk.DISABLED)
            self.btn_quintic_cancel.config(state=tk.DISABLED)
            self.btn_exit_quintic.config(state=tk.NORMAL)
            self.lbl_popup_start_speed.config(text=self._format_speed(self.current_commanded_speed_um_min))
            self.lbl_quintic_summary.config(
                text=f"Current commanded speed: {self._format_speed(self.current_commanded_speed_um_min)}"
            )
            self.lbl_quintic_validation.config(text="Ramp ended. Enter a new target or exit Quintic Mode.")
            self._clear_trajectory_table()
            self._update_quintic_calculate_state()
            self._update_popup_state()

    def _get_applied_speed_um_min(self):
        if self.motion_state == "PAUSED" or self.current_commanded_speed_um_min is None:
            return 0.0
        return float(self.current_commanded_speed_um_min)

    def _update_popup_state(self):
        if not self._popup_exists():
            return
        try:
            applied = self._get_applied_speed_um_min()
            self.lbl_current_speed_large.config(text=self._format_speed(applied))
            self.lbl_motion_large.config(text=self.motion_state)
            pos_text = "-- mm" if self.current_position_mm is None else f"{self.current_position_mm:.3f} mm"
            self.lbl_position_large.config(text=pos_text)
        except tk.TclError:
            pass

    def _graph_refresh_tick(self):
        if not self.polling_active:
            return

        if self._popup_exists() and hasattr(self, "quintic_speed_canvas"):
            try:
                elapsed = time.monotonic() - self.graph_start_time
                speed_mm_min = self._get_applied_speed_um_min() / 1000.0
                position = self.current_position_mm

                self.graph_x.append(elapsed)
                self.graph_y.append(speed_mm_min)
                self.position_graph_x.append(elapsed)
                self.position_graph_y.append(float("nan") if position is None else position)

                cutoff = max(0.0, elapsed - GRAPH_HISTORY_SECONDS)
                while len(self.graph_x) > 2 and self.graph_x[1] < cutoff:
                    self.graph_x.pop(0); self.graph_y.pop(0)
                while len(self.position_graph_x) > 2 and self.position_graph_x[1] < cutoff:
                    self.position_graph_x.pop(0); self.position_graph_y.pop(0)

                self.ax.clear()
                self.ax.set_xlabel("Elapsed time (s)")
                self.ax.set_ylabel("Speed (mm/min)")
                self.ax.grid(True, alpha=0.3)
                self.ax.plot(self.graph_x, self.graph_y, linewidth=2)

                self.position_ax.clear()
                self.position_ax.set_xlabel("Elapsed time (s)")
                self.position_ax.set_ylabel("Position (mm)")
                self.position_ax.grid(True, alpha=0.3)
                self.position_ax.plot(self.position_graph_x, self.position_graph_y, linewidth=2)

                self.lbl_graph_speed.config(text=f"Applied speed: {speed_mm_min:g} mm/min")
                self.lbl_graph_position.config(
                    text="Position: -- mm" if position is None else f"Position: {position:.3f} mm"
                )
                self._update_popup_state()
                self.figure.tight_layout()
                self.position_figure.tight_layout()
                self.quintic_speed_canvas.draw_idle()
                self.position_canvas.draw_idle()
            except tk.TclError:
                pass

        self.root.after(GRAPH_REFRESH_MS, self._graph_refresh_tick)

    def clear_graph(self):
        self.graph_start_time = time.monotonic()
        self.graph_x = [0.0]
        self.graph_y = [self._get_applied_speed_um_min() / 1000.0]
        self.position_graph_x = [0.0]
        self.position_graph_y = [float("nan") if self.current_position_mm is None else self.current_position_mm]
        self.log("Live speed and position graphs cleared.")

    def _parse_position_response(text):
        """Parse common actuator P responses without assuming one comma format.

        Supported examples:
          12.3456
          -0.1250
          12.3456,12345        (position, hardware-time)
          12,3456              (decimal comma position)
          12,3456;12345        (decimal comma position; hardware-time)
        """
        text = text.strip()
        if not text:
            return None

        # Semicolon safely separates fields when a decimal comma is used.
        if ";" in text:
            first, *rest = [part.strip() for part in text.split(";")]
            try:
                pos = float(first.replace(",", "."))
            except ValueError:
                return None
            hw_time = None
            if rest and re.fullmatch(r"[+-]?\d+", rest[0]):
                hw_time = int(rest[0])
            return pos, hw_time

        # Dot-decimal position optionally followed by comma + integer hw time.
        m = re.fullmatch(r"\s*([+-]?\d+(?:\.\d+)?)\s*(?:,\s*([+-]?\d+)\s*)?", text)
        if m:
            return float(m.group(1)), int(m.group(2)) if m.group(2) is not None else None

        # If there is one comma and no dot, treat it as a decimal separator.
        if text.count(",") == 1 and "." not in text:
            try:
                return float(text.replace(",", ".")), None
            except ValueError:
                return None

        # Last-resort: first conventional floating-point token in the reply.
        m = re.search(r"[+-]?\d+(?:\.\d+)?", text)
        if m:
            return float(m.group(0)), None
        return None

    def _position_ui_update(self, position):
        self.current_position_mm = position
        self._refresh_state_views()

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
                self.combo_daq_port.config(state=tk.NORMAL)
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
            timestamps.append(float(raw_data[i+1]))
            
        return readings, timestamps

    def voltage_to_temperature(self, channel, voltage):
        cal = TEMPERATURE_CALIBRATION.get(channel)
        if not cal:
            return 0.0
        voltage_mv = voltage * 1000
        return voltage_mv * cal["gain"] + cal["offset"]

    # ------------------- LOGGING & TEST CONTROL -------------------
    def check_enable_test(self):
        if self.psu or self.actuator or self.daq:
            self.btn_start_test.config(state=tk.NORMAL)

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
        self.pos_data.clear()
        for ch in CHANNELS:
            self.temp_data[ch].clear()
            
        self.start_time = time.time()
        
        filename = f"daq_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.csv_file = open(filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        headers = [
            "PC_Timestamp", "PC_Elapsed_s",
            "PSU_Voltage_V", "PSU_Current_A",
            "Actuator_Position_mm", "Actuator_HW_Time_ms",
            "PSU_Latency_ms", "Actuator_Latency_ms", "DAQ_Latency_ms"
        ]
        for ch in CHANNELS:
            headers.extend([f"CH{ch}_Voltage_V", f"CH{ch}_Temp_C", f"CH{ch}_HW_Time_s"])
            
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
        """Select or clear all thermocouple lines without affecting acquisition."""
        for ch in CHANNELS:
            self.temp_channel_visible[ch].set(bool(visible))
        self._on_temp_channel_visibility_changed()

    def _on_temp_channel_visibility_changed(self):
        """Apply the thermocouple display filter and refresh legend/scale."""
        if not hasattr(self, "line_temps"):
            return

        for ch in CHANNELS:
            self.line_temps[ch].set_visible(self.temp_channel_visible[ch].get())

        self._refresh_temp_legend_and_scale()
        if hasattr(self, "canvas"):
            self.canvas.draw_idle()

    def _refresh_temp_legend_and_scale(self):
        """Keep the thermocouple legend and Y-axis tied to visible channels only."""
        visible_lines = [
            self.line_temps[ch]
            for ch in CHANNELS
            if self.temp_channel_visible[ch].get()
        ]

        old_legend = self.ax_temp.get_legend()
        if old_legend is not None:
            old_legend.remove()

        if visible_lines:
            self.ax_temp.legend(
                visible_lines,
                [line.get_label() for line in visible_lines],
                fontsize=8,
                loc="upper left",
            )
            self.ax_temp.relim(visible_only=True)
            self.ax_temp.autoscale_view()
        else:
            # An empty selection is valid. Keep the graph usable rather than
            # stopping DAQ acquisition or treating it as an error.
            self.ax_temp.set_ylim(auto=True)

    def update_plots(self):
        if not self.t_data:
            return
        self.line_v.set_data(self.t_data, self.v_data)
        self.line_i.set_data(self.t_data, self.i_data)
        self.line_pos.set_data(self.t_data, self.pos_data)
        
        for ch in CHANNELS:
            self.line_temps[ch].set_data(self.t_data, self.temp_data[ch])
            self.line_temps[ch].set_visible(self.temp_channel_visible[ch].get())

        self.ax_pwr_v.relim()
        self.ax_pwr_v.autoscale_view()
        self.ax_pwr_i.relim()
        self.ax_pwr_i.autoscale_view()
        
        self.ax_pos.relim()
        self.ax_pos.autoscale_view()

        self._refresh_temp_legend_and_scale()

        self.canvas.draw()

    # ------------------- CORE POLLING LOOP -------------------
    def hardware_poll_loop(self):
        while self.polling_active:
            loop_start = time.time()
            self.process_ramping()

            current_time_str = datetime.now().isoformat()
            elapsed = loop_start - self.start_time if (self.test_running and self.start_time) else 0.0
            
            v_meas, i_meas, pos_mm, act_time_ms = None, None, None, None
            psu_latency_ms, act_latency_ms, daq_latency_ms = None, None, None
            daq_voltages = [None] * len(CHANNELS)
            daq_temps = [None] * len(CHANNELS)
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
                        self.lbl_i_meas.config(text=f"Meas I: {i:.3f} A")
                    ])
                except Exception as e:
                    self.log(f"PSU Poll Error: {e}")

            # 2. Query Actuator -- this is the ONLY P polling/read path.
            if self.actuator and self.actuator.is_open:
                try:
                    t_start_act = time.perf_counter()
                    with self.actuator_lock:
                        self.actuator.reset_input_buffer()
                        self.actuator.write(b"P\n")
                        self.actuator.flush()
                        raw = self.actuator.readline()
                    act_latency_ms = (time.perf_counter() - t_start_act) * 1000
                    if raw:
                        text = raw.decode("ascii", errors="replace").strip()
                        parsed = self._parse_position_response(text)
                        if parsed is not None:
                            pos_mm, act_time_ms = parsed
                            self.current_position_mm = pos_mm
                            self.root.after(0, lambda p=pos_mm: self._position_ui_update(p))
                        else:
                            self.log(f"Ignored unrecognized P response: {text!r}")
                except Exception as e:
                    self.log(f"Position polling error: {e}")

            # 3. Query DAQ
            if self.daq and self.test_running:
                try:
                    t_start_daq = time.perf_counter()
                    daq_voltages, daq_times = self.read_all_channels()
                    daq_latency_ms = (time.perf_counter() - t_start_daq) * 1000
                    
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
                        pos_mm if pos_mm is not None else "",
                        act_time_ms if act_time_ms is not None else "",
                        round(psu_latency_ms, 2) if psu_latency_ms is not None else "",
                        round(act_latency_ms, 2) if act_latency_ms is not None else "",
                        round(daq_latency_ms, 2) if daq_latency_ms is not None else ""
                    ]
                    
                    for v, t, hw_time in zip(daq_voltages, daq_temps, daq_times):
                        row.extend([
                            v if v is not None else "",
                            t if t is not None else "",
                            hw_time if hw_time is not None else ""
                        ])
                    
                    self.csv_writer.writerow(row)
                    self.csv_file.flush()

                self.t_data.append(elapsed)
                self.v_data.append(v_meas if v_meas is not None else float('nan'))
                self.i_data.append(i_meas if i_meas is not None else float('nan'))
                self.pos_data.append(pos_mm if pos_mm is not None else float('nan'))
                
                for i, ch in enumerate(CHANNELS):
                    self.temp_data[ch].append(daq_temps[i] if daq_temps[i] is not None else float('nan'))

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
        self.quintic_stop_event.set()
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
            
        with self.actuator_lock:
            if self.actuator and self.actuator.is_open:
                try:
                    self.actuator.write(b"N\n")
                    self.actuator.write(b"E0\n")
                    self.actuator.close()
                except Exception:
                    pass

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
