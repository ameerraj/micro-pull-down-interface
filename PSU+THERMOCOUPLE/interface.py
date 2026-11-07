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
        self.root.geometry("1200x800")
        self.root.minsize(1000, 700)

        # Individual hardware locks. The polling thread and button callbacks
        # never access the same instrument simultaneously.
        self.psu_lock = threading.Lock()
        self.daq_lock = threading.Lock()

        # VISA backend for the Keithley DAQ6510.
        self.rm = pyvisa.ResourceManager()

        # Hardware handles.
        self.psu = None
        self.psu_protocol = "SCPI"
        self.daq = None

        # Logging / polling state.
        self.polling_active = True
        self.test_running = False
        self.csv_file = None
        self.csv_writer = None
        self.loop_iteration = 0
        self.poll_interval = DEFAULT_POLL_INTERVAL
        self.start_time = None

        # PSU safety and ramp state.
        self.max_v_limit = 10.0
        self.max_i_limit = 360.0
        self.ramping_active = False
        self.psu_target_i = 0.0
        self.psu_current_setpoint = 0.0
        self.ramp_rate_A_per_sec = 0.0
        self.last_ramp_time = 0.0

        # Discovered hardware ports.
        self.available_psu_com = []
        self.available_usb_visa = []

        # Plot data.
        self.t_data = []
        self.v_data = []
        self.i_data = []
        self.temp_data = {ch: [] for ch in CHANNELS}

        # Graph visibility only. All channels are always acquired and logged.
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
        # VISA resources for the Keithley.
        try:
            resources = self.rm.list_resources()
            self.available_usb_visa = [
                resource
                for resource in resources
                if "USB" in resource or "TCPIP" in resource
            ]
            print(f"DEBUG VISA Resources: {resources}")
        except Exception as exc:
            print(f"VISA Scan Error: {exc}")
            self.available_usb_visa = []

        # COM ports for the TDK-Lambda PSU.
        try:
            com_ports = serial.tools.list_ports.comports()
            self.available_psu_com = []

            for port in com_ports:
                # Prefer the known TDK-Lambda USB CDC VID:PID / COM4.
                if "2504:0002" in port.hwid or port.device == "COM4":
                    self.available_psu_com.insert(0, port.device)
                else:
                    self.available_psu_com.append(port.device)

            if not self.available_psu_com:
                self.available_psu_com = ["COM4"]

        except Exception as exc:
            print(f"COM Port Scan Error: {exc}")
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

        # -------------------- GENERAL LOGGING --------------------
        logging_frame = ttk.LabelFrame(
            main_frame,
            text="Synchronized Data Logging",
            padding=8,
        )
        logging_frame.pack(fill=tk.X, pady=(0, 8))
        logging_frame.columnconfigure(1, weight=1)
        logging_frame.columnconfigure(2, weight=1)
        logging_frame.columnconfigure(3, weight=1)
        logging_frame.columnconfigure(4, weight=0)

        ttk.Label(logging_frame, text="Poll Interval (s):").grid(
            row=0, column=0, sticky="w", padx=(0, 6)
        )

        self.entry_poll_interval = ttk.Entry(logging_frame, width=10)
        self.entry_poll_interval.grid(
            row=0, column=1, sticky="w", padx=(0, 12)
        )
        self.entry_poll_interval.insert(0, "1.0")

        self.btn_start_test = ttk.Button(
            logging_frame,
            text="Start Log",
            command=self.start_test,
            state=tk.DISABLED,
        )
        self.btn_start_test.grid(row=0, column=2, sticky="ew", padx=2)

        self.btn_stop_test = ttk.Button(
            logging_frame,
            text="Stop Log",
            command=self.stop_test,
            state=tk.DISABLED,
        )
        self.btn_stop_test.grid(row=0, column=3, sticky="ew", padx=2)

        ttk.Button(
            logging_frame,
            text="Refresh Ports",
            command=self.refresh_ports,
        ).grid(row=0, column=4, sticky="ew", padx=(8, 0))

        # -------------------- HARDWARE CONTROLS --------------------
        control_frame = ttk.Frame(main_frame)
        control_frame.pack(fill=tk.X, pady=(0, 8))
        control_frame.columnconfigure(0, weight=1)
        control_frame.columnconfigure(1, weight=1)

        self._build_psu_interface(control_frame)
        self._build_daq_interface(control_frame)

        # -------------------- SYSTEM LOG --------------------
        # Keep log anchored at the bottom so it remains visible if the window
        # is resized vertically.
        log_frame = ttk.LabelFrame(main_frame, text="System Log", padding=5)
        log_frame.pack(side=tk.BOTTOM, fill=tk.X, pady=(8, 0))

        log_holder = ttk.Frame(log_frame)
        log_holder.pack(fill=tk.BOTH, expand=True)

        self.log_text = tk.Text(
            log_holder,
            height=6,
            state=tk.DISABLED,
            wrap=tk.WORD,
        )
        log_scroll = ttk.Scrollbar(
            log_holder,
            orient=tk.VERTICAL,
            command=self.log_text.yview,
        )
        self.log_text.configure(yscrollcommand=log_scroll.set)
        self.log_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        log_scroll.pack(side=tk.RIGHT, fill=tk.Y)

        # -------------------- GRAPHS --------------------
        graph_frame = ttk.Frame(main_frame)
        graph_frame.pack(fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(11, 4), dpi=100)
        self.fig.patch.set_facecolor("#f0f0f0")

        # Power supply graph.
        self.ax_pwr_v = self.fig.add_subplot(121)
        self.ax_pwr_i = self.ax_pwr_v.twinx()
        self.ax_pwr_v.set_title("Power Supply")
        self.ax_pwr_v.set_xlabel("Time (s)")
        self.ax_pwr_v.set_ylabel("Voltage (V)")
        self.ax_pwr_i.set_ylabel("Current (A)")
        self.line_v, = self.ax_pwr_v.plot([], [], label="Voltage")
        self.line_i, = self.ax_pwr_i.plot([], [], label="Current")

        # Thermocouple graph.
        self.ax_temp = self.fig.add_subplot(122)
        self.ax_temp.set_title("Thermocouples")
        self.ax_temp.set_xlabel("Time (s)")
        self.ax_temp.set_ylabel("Temperature (°C)")

        self.line_temps = {}
        for ch in CHANNELS:
            self.line_temps[ch], = self.ax_temp.plot(
                [], [], label=f"CH {ch}"
            )
        self.ax_temp.legend(fontsize=8, loc="upper left")

        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, master=graph_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def _build_psu_interface(self, parent):
        psu_frame = ttk.LabelFrame(
            parent,
            text="TDK-Lambda Power Supply (GSP 10-1000)",
            padding=10,
        )
        psu_frame.grid(
            row=0,
            column=0,
            sticky="nsew",
            padx=(0, 5),
        )

        for col in range(4):
            psu_frame.columnconfigure(col, weight=1)

        ttk.Label(psu_frame, text="Port:").grid(
            row=0, column=0, sticky="w", padx=2
        )
        self.combo_psu_port = ttk.Combobox(
            psu_frame,
            values=self.available_psu_com,
            width=15,
        )
        if self.available_psu_com:
            self.combo_psu_port.set(self.available_psu_com[0])
        self.combo_psu_port.grid(
            row=0, column=1, padx=5, pady=5, sticky="ew"
        )

        self.btn_psu_conn = ttk.Button(
            psu_frame,
            text="Connect PSU",
            command=self.connect_psu_thread,
        )
        self.btn_psu_conn.grid(
            row=0,
            column=2,
            columnspan=2,
            pady=5,
            sticky="ew",
        )

        ttk.Label(psu_frame, text="Max Allowable V:").grid(
            row=1, column=0, sticky="w"
        )
        self.entry_max_v = ttk.Entry(psu_frame, width=8)
        self.entry_max_v.grid(row=1, column=1, padx=5, pady=2)
        self.entry_max_v.insert(0, "10.0")

        ttk.Label(psu_frame, text="Max Allowable I:").grid(
            row=1, column=2, sticky="w"
        )
        self.entry_max_i = ttk.Entry(psu_frame, width=8)
        self.entry_max_i.grid(row=1, column=3, padx=5, pady=2)
        self.entry_max_i.insert(0, "3.0")

        ttk.Label(psu_frame, text="Target Voltage (V):").grid(
            row=2, column=0, sticky="w"
        )
        self.entry_target_v = ttk.Entry(psu_frame, width=8)
        self.entry_target_v.grid(row=2, column=1, padx=5, pady=2)
        self.entry_target_v.insert(0, "0.0")

        ttk.Label(psu_frame, text="Target Current (A):").grid(
            row=2, column=2, sticky="w"
        )
        self.entry_target_i = ttk.Entry(psu_frame, width=8)
        self.entry_target_i.grid(row=2, column=3, padx=5, pady=2)
        self.entry_target_i.insert(0, "0.0")

        ttk.Label(psu_frame, text="Ramp Rate (A/min):").grid(
            row=3, column=0, sticky="w"
        )
        self.entry_ramp_rate = ttk.Entry(psu_frame, width=8)
        self.entry_ramp_rate.grid(row=3, column=1, padx=5, pady=2)
        self.entry_ramp_rate.insert(0, "0.5")

        self.btn_apply = ttk.Button(
            psu_frame,
            text="Start Ramp",
            command=self.start_current_ramp,
            state=tk.DISABLED,
        )
        self.btn_apply.grid(
            row=3, column=2, pady=5, sticky="ew", padx=2
        )

        self.btn_stop_ramp = ttk.Button(
            psu_frame,
            text="Stop Ramp (Hold)",
            command=self.stop_current_ramp,
            state=tk.DISABLED,
        )
        self.btn_stop_ramp.grid(
            row=3, column=3, pady=5, sticky="ew", padx=2
        )

        self.btn_safe_off = ttk.Button(
            psu_frame,
            text="🛑 SAFELY TURN OFF HEATER",
            command=self.safe_heater_off,
            state=tk.DISABLED,
        )
        self.btn_safe_off.grid(
            row=4,
            column=0,
            columnspan=4,
            pady=5,
            sticky="ew",
        )

        self.lbl_v_meas = ttk.Label(
            psu_frame,
            text="Meas V: -- V",
            font=("Arial", 11, "bold"),
        )
        self.lbl_v_meas.grid(
            row=5, column=0, columnspan=2, pady=5
        )

        self.lbl_i_meas = ttk.Label(
            psu_frame,
            text="Meas I: -- A",
            font=("Arial", 11, "bold"),
        )
        self.lbl_i_meas.grid(
            row=5, column=2, columnspan=2, pady=5
        )

    def _build_daq_interface(self, parent):
        daq_frame = ttk.LabelFrame(
            parent,
            text="Keithley DAQ6510 / Thermocouples",
            padding=10,
        )
        daq_frame.grid(
            row=0,
            column=1,
            sticky="nsew",
            padx=(5, 0),
        )

        self.combo_daq_port = ttk.Combobox(
            daq_frame,
            values=self.available_usb_visa,
        )
        if self.available_usb_visa:
            self.combo_daq_port.set(self.available_usb_visa[0])
        self.combo_daq_port.pack(fill=tk.X, pady=5)

        self.btn_daq_conn = ttk.Button(
            daq_frame,
            text="Connect DAQ",
            command=self.connect_daq_thread,
        )
        self.btn_daq_conn.pack(fill=tk.X, pady=5)

        self.lbl_daq_status = ttk.Label(
            daq_frame,
            text="Status: Disconnected",
        )
        self.lbl_daq_status.pack(pady=5)

        self.lbl_daq_temps = ttk.Label(
            daq_frame,
            text="Temps: -- °C",
            font=("Arial", 11),
            wraplength=500,
            justify=tk.LEFT,
        )
        self.lbl_daq_temps.pack(fill=tk.X, pady=(8, 5))

        channel_filter = ttk.LabelFrame(
            daq_frame,
            text="Graph Channels",
            padding=5,
        )
        channel_filter.pack(fill=tk.X, pady=(3, 5))

        for col in range(3):
            channel_filter.columnconfigure(col, weight=1)

        for index, ch in enumerate(CHANNELS):
            ttk.Checkbutton(
                channel_filter,
                text=f"CH {ch}",
                variable=self.temp_channel_visible[ch],
                command=self._on_temp_channel_visibility_changed,
            ).grid(
                row=index // 3,
                column=index % 3,
                sticky="w",
                padx=3,
                pady=2,
            )

        channel_buttons = ttk.Frame(channel_filter)
        channel_buttons.grid(
            row=2,
            column=0,
            columnspan=3,
            sticky="ew",
            pady=(4, 0),
        )
        channel_buttons.columnconfigure(0, weight=1)
        channel_buttons.columnconfigure(1, weight=1)

        ttk.Button(
            channel_buttons,
            text="Select All",
            command=lambda: self._set_all_temp_channels(True),
        ).grid(row=0, column=0, sticky="ew", padx=(0, 2))

        ttk.Button(
            channel_buttons,
            text="Clear All",
            command=lambda: self._set_all_temp_channels(False),
        ).grid(row=0, column=1, sticky="ew", padx=(2, 0))

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
    # POWER SUPPLY SERIAL METHODS
    # ============================================================
    @staticmethod
    def _psu_write(ser_inst, cmd):
        """Write one PSU command with CR/LF termination."""
        ser_inst.write(f"{cmd}\r\n".encode("ascii"))
        ser_inst.flush()

    def _psu_query(self, ser_inst, cmd, wait_s=0.08):
        """Write one PSU command and read until CR or timeout."""
        ser_inst.reset_input_buffer()
        self._psu_write(ser_inst, cmd)
        time.sleep(wait_s)
        raw = ser_inst.read_until(b"\r")
        if ser_inst.in_waiting > 0:
            raw += ser_inst.read(ser_inst.in_waiting)
        return raw.decode("ascii", errors="replace").strip()

    def check_psu_command(self, ser_inst, cmd):
        """Send a PSU command and check for a reply or SCPI error."""
        ser_inst.reset_input_buffer()
        self._psu_write(ser_inst, cmd)
        time.sleep(0.08)

        if ser_inst.in_waiting > 0:
            reply = ser_inst.read(
                ser_inst.in_waiting
            ).decode("ascii", errors="replace").strip()
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
            messagebox.showerror(
                "Error",
                "Please select or type a COM port for the PSU (e.g. COM4).",
            )
            return

        if selected_port.upper().startswith("ASRL"):
            num = "".join(filter(str.isdigit, selected_port))
            selected_port = f"COM{num}"

        self.btn_psu_conn.config(state=tk.DISABLED)
        self.combo_psu_port.config(state=tk.DISABLED)
        self.log(
            f"Attempting to connect to PSU on {selected_port} via PySerial..."
        )

        threading.Thread(
            target=self._connect_psu,
            args=(selected_port,),
            daemon=True,
        ).start()

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

                # Required by the TDK-Lambda Genesys+ USB CDC interface.
                temp_ser.dtr = True
                temp_ser.rts = True
                time.sleep(0.15)
                temp_ser.reset_input_buffer()
                temp_ser.reset_output_buffer()

                connected_idn = None
                self.psu_protocol = None

                for address in [6, 1, 0]:
                    # Try SCPI protocol first.
                    self._psu_write(
                        temp_ser,
                        f"INST:NSEL {address}",
                    )
                    time.sleep(0.1)
                    temp_ser.reset_input_buffer()
                    idn = self._psu_query(
                        temp_ser,
                        "*IDN?",
                        wait_s=0.15,
                    )
                    if idn:
                        connected_idn = idn
                        self.psu_protocol = "SCPI"
                        self.log(
                            f"✅ PSU Connected (SCPI Addr {address}): "
                            f"{connected_idn}"
                        )
                        break

                    # Fallback to GEN protocol.
                    response = self._psu_query(
                        temp_ser,
                        f"ADR {address:02d}",
                        wait_s=0.15,
                    )
                    if "OK" in response:
                        connected_idn = self._psu_query(
                            temp_ser,
                            "IDN?",
                            wait_s=0.15,
                        )
                        self.psu_protocol = "GEN"
                        self.log(
                            f"✅ PSU Connected (GEN Addr {address:02d}): "
                            f"{connected_idn}"
                        )
                        break

                if not connected_idn or not self.psu_protocol:
                    raise RuntimeError(
                        f"No response from PSU on {port}. Verify Front Panel "
                        "COMM -> IntF is set to 'USB' and Addr=6."
                    )

                if self.psu_protocol == "SCPI":
                    startup_commands = [
                        "SYST:SET REM",
                        f"VOLT:PROT:LEV {HARDWARE_OVP}",
                        "VOLT 0.0",
                        "CURR 0.0",
                        "OUTP ON",
                    ]
                else:
                    startup_commands = [
                        "RMT 1",
                        f"OVP {HARDWARE_OVP}",
                        "PV 0.0",
                        "PC 0.0",
                        "OUT 1",
                    ]

                for command in startup_commands:
                    self.check_psu_command(temp_ser, command)

                self.psu = temp_ser

            def update_ui():
                self.btn_psu_conn.config(
                    text="PSU Connected",
                    state=tk.DISABLED,
                )
                self.btn_apply.config(state=tk.NORMAL)
                self.btn_safe_off.config(state=tk.NORMAL)
                self.check_enable_test()
                self.log(
                    f"PSU active in REM mode ({self.psu_protocol}). "
                    "Output enabled at 0 V / 0 A."
                )

            self.root.after(0, update_ui)

        except Exception as exc:
            self.log(f"PSU Connection failed: {exc}")
            if temp_ser and temp_ser.is_open:
                try:
                    temp_ser.close()
                except Exception:
                    pass

            def reset_ui():
                self.btn_psu_conn.config(
                    state=tk.NORMAL,
                    text="Connect PSU",
                )
                self.combo_psu_port.config(state=tk.NORMAL)

            self.root.after(0, reset_ui)

    def start_current_ramp(self):
        if not self.psu:
            return

        try:
            self.max_v_limit = float(self.entry_max_v.get())
            self.max_i_limit = float(self.entry_max_i.get())
            target_v = float(self.entry_target_v.get())
            target_i = float(self.entry_target_i.get())
            ramp_rate_a_min = float(self.entry_ramp_rate.get())

            if (
                self.max_v_limit < 0
                or self.max_i_limit < 0
                or target_v < 0
                or target_i < 0
                or ramp_rate_a_min <= 0
            ):
                raise ValueError(
                    "Limits and targets must be non-negative and ramp rate must be > 0."
                )

            if self.max_v_limit > HARDWARE_OVP:
                messagebox.showerror(
                    "Limit Error",
                    f"Max allowable voltage cannot exceed the hardware OVP "
                    f"ceiling of {HARDWARE_OVP} V.",
                )
                return

            if target_v > self.max_v_limit or target_i > self.max_i_limit:
                messagebox.showerror(
                    "Limit Error",
                    "Targets exceed maximum stored limits "
                    f"({self.max_v_limit} V, {self.max_i_limit} A).",
                )
                return

            with self.psu_lock:
                if self.psu_protocol == "SCPI":
                    self.check_psu_command(
                        self.psu,
                        f"VOLT {target_v}",
                    )
                    current_setpoint_str = self._psu_query(
                        self.psu,
                        "CURR?",
                    )
                else:
                    self.check_psu_command(
                        self.psu,
                        f"PV {target_v}",
                    )
                    current_setpoint_str = self._psu_query(
                        self.psu,
                        "PC?",
                    )

            self.psu_current_setpoint = float(current_setpoint_str)
            self.psu_target_i = target_i
            self.ramp_rate_A_per_sec = ramp_rate_a_min / 60.0
            self.last_ramp_time = time.time()
            self.ramping_active = True
            self.btn_stop_ramp.config(state=tk.NORMAL)

            self.log(
                f"Starting Current Ramp: {self.psu_current_setpoint} A -> "
                f"{target_i} A at {ramp_rate_a_min} A/min."
            )

        except ValueError as exc:
            messagebox.showerror(
                "Input Error",
                f"Please ensure all limits and targets are valid: {exc}",
            )
        except Exception as exc:
            self.log(f"Error starting ramp: {exc}")

    def stop_current_ramp(self):
        if self.ramping_active:
            self.ramping_active = False
            self.btn_stop_ramp.config(state=tk.DISABLED)
            self.log(
                "Ramp manually stopped. Holding at "
                f"{self.psu_current_setpoint:.3f} A."
            )

    def safe_heater_off(self):
        if not self.psu:
            return

        self.ramping_active = False
        self.btn_stop_ramp.config(state=tk.DISABLED)

        try:
            with self.psu_lock:
                if self.psu_protocol == "SCPI":
                    self.check_psu_command(self.psu, "VOLT 0.0")
                    self.check_psu_command(self.psu, "CURR 0.0")
                    self.check_psu_command(self.psu, "OUTP OFF")
                else:
                    self.check_psu_command(self.psu, "PV 0.0")
                    self.check_psu_command(self.psu, "PC 0.0")
                    self.check_psu_command(self.psu, "OUT 0")

            self.psu_current_setpoint = 0.0
            self.psu_target_i = 0.0
            self.log(
                "🛑 SAFE OFF TRIGGERED. Heater set to 0 V / 0 A "
                "and output disabled."
            )
        except Exception as exc:
            self.log(f"Error during Safe Off: {exc}")

    def process_ramping(self):
        if not self.ramping_active or not self.psu:
            return

        now = time.time()
        dt = now - self.last_ramp_time
        self.last_ramp_time = now
        delta_i = self.ramp_rate_A_per_sec * dt

        if self.psu_current_setpoint < self.psu_target_i:
            self.psu_current_setpoint = min(
                self.psu_current_setpoint + delta_i,
                self.psu_target_i,
            )
        elif self.psu_current_setpoint > self.psu_target_i:
            self.psu_current_setpoint = max(
                self.psu_current_setpoint - delta_i,
                self.psu_target_i,
            )

        if self.psu_current_setpoint > self.max_i_limit:
            self.psu_current_setpoint = self.max_i_limit
            self.ramping_active = False
            self.root.after(
                0,
                lambda: self.btn_stop_ramp.config(state=tk.DISABLED),
            )
            self.log("Ramp stopped: Hit max current limit.")

        try:
            with self.psu_lock:
                if self.psu_protocol == "SCPI":
                    self._psu_write(
                        self.psu,
                        f"CURR {self.psu_current_setpoint:.3f}",
                    )
                else:
                    self._psu_query(
                        self.psu,
                        f"PC {self.psu_current_setpoint:.3f}",
                    )
        except Exception as exc:
            self.log(f"Ramp write error: {exc}")

        if math.isclose(
            self.psu_current_setpoint,
            self.psu_target_i,
            abs_tol=0.001,
        ):
            self.ramping_active = False
            self.root.after(
                0,
                lambda: self.btn_stop_ramp.config(state=tk.DISABLED),
            )
            self.log(
                f"Ramp complete. Reached target {self.psu_target_i} A."
            )

    # ============================================================
    # KEITHLEY DAQ6510 METHODS
    # ============================================================
    def connect_daq_thread(self):
        selected_port = self.combo_daq_port.get().strip()
        if not selected_port:
            messagebox.showerror(
                "Error",
                "Please select a VISA port for the DAQ.",
            )
            return

        self.btn_daq_conn.config(state=tk.DISABLED)
        self.combo_daq_port.config(state=tk.DISABLED)
        self.lbl_daq_status.config(text="Status: Connecting...")

        threading.Thread(
            target=self._connect_daq,
            args=(selected_port,),
            daemon=True,
        ).start()

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
                self.btn_daq_conn.config(
                    text="DAQ Connected",
                    state=tk.DISABLED,
                )
                self.lbl_daq_status.config(text="Status: Configured")
                self.check_enable_test()

            self.root.after(0, update_ui)

        except Exception as exc:
            self.log(f"DAQ Connection failed: {exc}")

            if temp_daq is not None:
                try:
                    temp_daq.close()
                except Exception:
                    pass

            self.daq = None

            def reset_ui():
                self.btn_daq_conn.config(
                    state=tk.NORMAL,
                    text="Connect DAQ",
                )
                self.combo_daq_port.config(state=tk.NORMAL)
                self.lbl_daq_status.config(text="Status: Disconnected")

            self.root.after(0, reset_ui)

    def configure_daq(self):
        with self.daq_lock:
            self.daq.write(":ABOR")
            self.daq.write("*RST")
            self.daq.query("*OPC?")
            self.daq.write("*CLS")

            terminal_position = self.daq.query(":ROUT:TERM?").strip()
            if "REAR" not in terminal_position.upper():
                raise RuntimeError(
                    "The DAQ6510 physical TERMINALS switch is not set to REAR."
                )

            self.daq.write(
                f':SENS:FUNC "VOLT:DC", {CHANNEL_LIST}'
            )
            self.daq.write(
                f":SENS:VOLT:DC:RANG:AUTO ON, {CHANNEL_LIST}"
            )
            self.daq.write(
                f":SENS:VOLT:DC:NPLC {NPLC}, {CHANNEL_LIST}"
            )
            self.daq.write(f":ROUT:SCAN:CRE {CHANNEL_LIST}")
            self.daq.write(":ROUT:SCAN:COUNT:SCAN 1")
            self.daq.query("*OPC?")

        errors = self.get_instrument_errors()
        if errors:
            raise RuntimeError(
                "DAQ configuration error(s): " + " | ".join(errors)
            )

    def get_instrument_errors(self, maximum_errors=10):
        errors = []
        with self.daq_lock:
            for _ in range(maximum_errors):
                error_message = self.daq.query(":SYST:ERR?").strip()
                if (
                    error_message.startswith("0,")
                    or error_message.startswith("+0,")
                    or "No error" in error_message
                ):
                    break
                errors.append(error_message)
        return errors

    def read_all_channels(self):
        with self.daq_lock:
            self.daq.write(':TRAC:CLEAR "defbuffer1"')
            self.daq.write(":INIT")
            self.daq.query("*OPC?")
            raw_data = self.daq.query_ascii_values(
                f':TRAC:DATA? 1, {len(CHANNELS)}, '
                '"defbuffer1", READ, REL'
            )

        if len(raw_data) != len(CHANNELS) * 2:
            raise RuntimeError(
                f"Expected {len(CHANNELS) * 2} data points, "
                f"received {len(raw_data)}"
            )

        readings = []
        timestamps = []
        for index in range(0, len(raw_data), 2):
            readings.append(float(raw_data[index]))
            timestamps.append(float(raw_data[index + 1]))

        return readings, timestamps

    @staticmethod
    def voltage_to_temperature(channel, voltage):
        calibration = TEMPERATURE_CALIBRATION.get(channel)
        if not calibration:
            return 0.0

        voltage_mv = voltage * 1000.0
        return (
            voltage_mv * calibration["gain"]
            + calibration["offset"]
        )

    # ============================================================
    # LOGGING / GRAPH CONTROL
    # ============================================================
    def check_enable_test(self):
        if self.psu is not None or self.daq is not None:
            self.btn_start_test.config(state=tk.NORMAL)

    def start_test(self):
        try:
            self.poll_interval = float(self.entry_poll_interval.get())
            if self.poll_interval <= 0:
                raise ValueError("Interval must be greater than zero.")
        except ValueError as exc:
            messagebox.showerror(
                "Input Error",
                f"Invalid poll interval: {exc}",
            )
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

        self.start_time = time.time()

        filename = (
            "psu_thermocouple_log_"
            f"{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )

        try:
            self.csv_file = open(filename, mode="w", newline="")
            self.csv_writer = csv.writer(self.csv_file)

            headers = [
                "PC_Timestamp",
                "PC_Elapsed_s",
                "PSU_Voltage_V",
                "PSU_Current_A",
                "PSU_Latency_ms",
                "DAQ_Latency_ms",
            ]

            for ch in CHANNELS:
                headers.extend(
                    [
                        f"CH{ch}_Voltage_V",
                        f"CH{ch}_Temp_C",
                        f"CH{ch}_HW_Time_s",
                    ]
                )

            self.csv_writer.writerow(headers)
            self.csv_file.flush()

        except Exception as exc:
            self.test_running = False
            self.btn_start_test.config(state=tk.NORMAL)
            self.entry_poll_interval.config(state=tk.NORMAL)
            self.btn_stop_test.config(state=tk.DISABLED)
            messagebox.showerror(
                "Logging Error",
                f"Could not create CSV file: {exc}",
            )
            return

        self.log(
            f"Started synchronized logging to {filename} "
            f"(Interval: {self.poll_interval} s)"
        )

    def stop_test(self):
        self.test_running = False

        if self.csv_file:
            try:
                self.csv_file.close()
            except Exception:
                pass
            self.csv_file = None
            self.csv_writer = None

        self.btn_start_test.config(
            state=tk.NORMAL
            if (self.psu is not None or self.daq is not None)
            else tk.DISABLED
        )
        self.entry_poll_interval.config(state=tk.NORMAL)
        self.btn_stop_test.config(state=tk.DISABLED)
        self.log("Data logging stopped.")

    def _set_all_temp_channels(self, visible):
        for ch in CHANNELS:
            self.temp_channel_visible[ch].set(bool(visible))
        self._on_temp_channel_visibility_changed()

    def _on_temp_channel_visibility_changed(self):
        if not hasattr(self, "line_temps"):
            return

        for ch in CHANNELS:
            self.line_temps[ch].set_visible(
                self.temp_channel_visible[ch].get()
            )

        self._refresh_temp_legend_and_scale()
        self.canvas.draw_idle()

    def _refresh_temp_legend_and_scale(self):
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
            self.ax_temp.set_ylim(auto=True)

    def update_plots(self):
        if not self.t_data:
            return

        self.line_v.set_data(self.t_data, self.v_data)
        self.line_i.set_data(self.t_data, self.i_data)

        for ch in CHANNELS:
            self.line_temps[ch].set_data(
                self.t_data,
                self.temp_data[ch],
            )
            self.line_temps[ch].set_visible(
                self.temp_channel_visible[ch].get()
            )

        self.ax_pwr_v.relim()
        self.ax_pwr_v.autoscale_view()
        self.ax_pwr_i.relim()
        self.ax_pwr_i.autoscale_view()
        self._refresh_temp_legend_and_scale()
        self.canvas.draw_idle()

    # ============================================================
    # CORE POLLING LOOP
    # ============================================================
    def hardware_poll_loop(self):
        while self.polling_active:
            loop_start = time.time()
            self.process_ramping()

            current_time_str = datetime.now().isoformat()
            elapsed = (
                loop_start - self.start_time
                if self.test_running and self.start_time
                else 0.0
            )

            v_meas = None
            i_meas = None
            psu_latency_ms = None
            daq_latency_ms = None
            daq_voltages = [None] * len(CHANNELS)
            daq_temps = [None] * len(CHANNELS)
            daq_times = [None] * len(CHANNELS)

            # 1. Poll PSU whenever connected.
            if self.psu and self.psu.is_open:
                try:
                    t_start_psu = time.perf_counter()

                    with self.psu_lock:
                        if self.psu_protocol == "SCPI":
                            v_str = self._psu_query(
                                self.psu,
                                "MEAS:VOLT?",
                                wait_s=0.05,
                            )
                            i_str = self._psu_query(
                                self.psu,
                                "MEAS:CURR?",
                                wait_s=0.05,
                            )
                        else:
                            v_str = self._psu_query(
                                self.psu,
                                "MV?",
                                wait_s=0.05,
                            )
                            i_str = self._psu_query(
                                self.psu,
                                "MC?",
                                wait_s=0.05,
                            )

                        v_meas = float(v_str)
                        i_meas = float(i_str)

                    psu_latency_ms = (
                        time.perf_counter() - t_start_psu
                    ) * 1000.0

                    def update_psu(v=v_meas, i=i_meas):
                        self.lbl_v_meas.config(
                            text=f"Meas V: {v:.3f} V"
                        )
                        self.lbl_i_meas.config(
                            text=f"Meas I: {i:.3f} A"
                        )

                    self.root.after(0, update_psu)

                except Exception as exc:
                    self.log(f"PSU Poll Error: {exc}")

            # 2. Poll all thermocouple channels whenever DAQ is connected.
            # Logging is optional; the live temperature display remains active.
            if self.daq is not None:
                try:
                    t_start_daq = time.perf_counter()
                    daq_voltages, daq_times = self.read_all_channels()
                    daq_latency_ms = (
                        time.perf_counter() - t_start_daq
                    ) * 1000.0

                    daq_temps = [
                        self.voltage_to_temperature(ch, voltage)
                        for ch, voltage in zip(
                            CHANNELS,
                            daq_voltages,
                        )
                    ]

                    temp_str = " | ".join(
                        f"CH{ch}: {temp:.1f}°C"
                        for ch, temp in zip(CHANNELS, daq_temps)
                    )

                    self.root.after(
                        0,
                        lambda text=temp_str: self.lbl_daq_temps.config(
                            text=text
                        ),
                    )

                except Exception as exc:
                    self.log(f"DAQ Poll Error: {exc}")

            # 3. Synchronized CSV logging and graphs.
            if self.test_running:
                if self.csv_writer:
                    row = [
                        current_time_str,
                        round(elapsed, 3),
                        v_meas if v_meas is not None else "",
                        i_meas if i_meas is not None else "",
                        (
                            round(psu_latency_ms, 2)
                            if psu_latency_ms is not None
                            else ""
                        ),
                        (
                            round(daq_latency_ms, 2)
                            if daq_latency_ms is not None
                            else ""
                        ),
                    ]

                    for voltage, temp, hw_time in zip(
                        daq_voltages,
                        daq_temps,
                        daq_times,
                    ):
                        row.extend(
                            [
                                voltage if voltage is not None else "",
                                temp if temp is not None else "",
                                hw_time if hw_time is not None else "",
                            ]
                        )

                    try:
                        self.csv_writer.writerow(row)
                        self.csv_file.flush()
                    except Exception as exc:
                        self.log(f"CSV Write Error: {exc}")

                self.t_data.append(elapsed)
                self.v_data.append(
                    v_meas if v_meas is not None else float("nan")
                )
                self.i_data.append(
                    i_meas if i_meas is not None else float("nan")
                )

                for index, ch in enumerate(CHANNELS):
                    self.temp_data[ch].append(
                        daq_temps[index]
                        if daq_temps[index] is not None
                        else float("nan")
                    )

                self.root.after(0, self.update_plots)

            # 4. Absolute logging cadence while logging is active.
            if self.test_running and self.start_time:
                self.loop_iteration += 1
                next_target_time = (
                    self.start_time
                    + self.loop_iteration * self.poll_interval
                )
                sleep_time = next_target_time - time.time()
                if sleep_time > 0:
                    time.sleep(sleep_time)
            else:
                sleep_time = (
                    DEFAULT_POLL_INTERVAL
                    - (time.time() - loop_start)
                )
                if sleep_time > 0:
                    time.sleep(sleep_time)

    # ============================================================
    # CLEAN SHUTDOWN
    # ============================================================
    def on_closing(self):
        self.polling_active = False
        self.ramping_active = False

        if self.test_running:
            self.stop_test()

        # Safely disable PSU output and return it to local mode.
        with self.psu_lock:
            if self.psu and self.psu.is_open:
                try:
                    if self.psu_protocol == "SCPI":
                        self._psu_write(self.psu, "VOLT 0.0")
                        self._psu_write(self.psu, "CURR 0.0")
                        self._psu_write(self.psu, "OUTP OFF")
                        self._psu_write(self.psu, "SYST:SET LOC")
                    else:
                        self._psu_query(self.psu, "PV 0.0")
                        self._psu_query(self.psu, "PC 0.0")
                        self._psu_query(self.psu, "OUT 0")
                        self._psu_query(self.psu, "RMT 0")
                    self.psu.close()
                except Exception:
                    pass
                self.psu = None

        # Abort and close the Keithley session.
        with self.daq_lock:
            if self.daq:
                try:
                    self.daq.write(":ABOR")
                    self.daq.close()
                except Exception:
                    pass
                self.daq = None

        try:
            self.rm.close()
        except Exception:
            pass

        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = PSUAndThermocoupleApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
