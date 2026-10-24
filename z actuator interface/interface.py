import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import math
import csv
from datetime import datetime

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


# ============================================================
# CONFIGURATION
# ============================================================
BAUD_RATE = 9600
ACTUATOR_STARTUP_SECONDS = 5.0
SERIAL_RESPONSE_TIMEOUT_SECONDS = 0.8
DEFAULT_POLL_INTERVAL = 1.0
QUINTIC_SAMPLE_INTERVAL_SECONDS = 0.5
GRAPH_REFRESH_MS = 150
GRAPH_HISTORY_SECONDS = 300.0


class StandaloneActuatorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Arduino Actuator Controller & Logger")
        self.root.geometry("680x560")
        self.root.minsize(620, 500)

        # ---------------- Hardware ----------------
        self.actuator_lock = threading.Lock()
        self.actuator = None
        self.available_com = []
        self.polling_active = True

        # ---------------- Logging ----------------
        self.test_running = False
        self.csv_file = None
        self.csv_writer = None
        self.poll_interval = DEFAULT_POLL_INTERVAL
        self.start_time = None
        self.log_history = []

        # ---------------- Shared actuator state ----------------
        self.selected_range = tk.StringVar(value="S1")
        self.current_commanded_speed_um_min = None
        self.motion_state = "PAUSED"          # UP / DOWN / PAUSED
        self.current_position_mm = None

        # ---------------- Quintic state ----------------
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

        # ---------------- Popup graph state ----------------
        self.graph_start_time = time.monotonic()
        self.graph_x = [0.0]
        self.graph_y = [0.0]
        self.position_graph_x = [0.0]
        self.position_graph_y = [0.0]

        self.scan_ports()
        self.setup_ui()
        self.bind_shortcuts()

        self.quintic_target_var.trace_add("write", lambda *_: self._update_quintic_calculate_state())
        self.quintic_unit_var.trace_add("write", lambda *_: self._update_quintic_calculate_state())
        self.quintic_duration_var.trace_add("write", lambda *_: self._update_quintic_calculate_state())

        self.poll_thread = threading.Thread(target=self.actuator_poll_loop, daemon=True)
        self.poll_thread.start()
        self.root.after(GRAPH_REFRESH_MS, self._graph_refresh_tick)

    # ============================================================
    # MAIN UI
    # ============================================================
    def scan_ports(self):
        try:
            ports = list(serial.tools.list_ports.comports())
            preferred = []
            other = []
            for p in ports:
                if any(key in p.description for key in ("Arduino", "CH340", "CP210")):
                    preferred.append(p.device)
                else:
                    other.append(p.device)
            self.available_com = preferred + other
        except Exception as exc:
            print(f"COM Port Scan Error: {exc}")
            self.available_com = []

    def setup_ui(self):
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        act_frame = ttk.LabelFrame(main_frame, text="Arduino Motor Control & Logging", padding=10)
        act_frame.pack(fill=tk.X, pady=(0, 10))
        for i in range(4):
            act_frame.columnconfigure(i, weight=1)

        ttk.Label(act_frame, text="Port:").grid(row=0, column=0, sticky="w", pady=5)
        self.combo_act_port = ttk.Combobox(act_frame, values=self.available_com, width=15)
        if self.available_com:
            self.combo_act_port.set(self.available_com[0])
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
        self.rb_s1 = ttk.Radiobutton(speed_radio_frame, text="S1 (1-100 µm/min)",
                                     variable=self.selected_range, value="S1", state=tk.DISABLED,
                                     command=self._update_speed_placeholder)
        self.rb_s1.pack(side=tk.LEFT, expand=True)
        self.rb_s2 = ttk.Radiobutton(speed_radio_frame, text="S2 (100-1000 µm/min)",
                                     variable=self.selected_range, value="S2", state=tk.DISABLED,
                                     command=self._update_speed_placeholder)
        self.rb_s2.pack(side=tk.LEFT, expand=True)
        self.rb_s3 = ttk.Radiobutton(speed_radio_frame, text="S3 (1-45 mm/min)",
                                     variable=self.selected_range, value="S3", state=tk.DISABLED,
                                     command=self._update_speed_placeholder)
        self.rb_s3.pack(side=tk.LEFT, expand=True)

        ttk.Label(act_frame, text="Speed:").grid(row=3, column=0, sticky="w", pady=5)
        self.speed_entry = ttk.Entry(act_frame, width=10, state=tk.DISABLED)
        self.speed_entry.grid(row=3, column=1, sticky="ew", pady=5)
        self.speed_entry.insert(0, "50")
        self.btn_send_speed = ttk.Button(act_frame, text="Set Speed", state=tk.DISABLED,
                                         command=self.send_speed_command)
        self.btn_send_speed.grid(row=3, column=2, columnspan=2, sticky="ew", padx=1, pady=5)

        self.btn_up = ttk.Button(act_frame, text="▲ UP (U1)", state=tk.DISABLED,
                                 command=lambda: self.send_actuator_command("U1"))
        self.btn_up.grid(row=4, column=0, sticky="ew", padx=1, pady=5)
        self.btn_pause = ttk.Button(act_frame, text="⏹ PAUSE (N)", state=tk.DISABLED,
                                    command=lambda: self.send_actuator_command("N"))
        self.btn_pause.grid(row=4, column=1, sticky="ew", padx=1, pady=5)
        self.btn_down = ttk.Button(act_frame, text="▼ DOWN (D1)", state=tk.DISABLED,
                                   command=lambda: self.send_actuator_command("D1"))
        self.btn_down.grid(row=4, column=2, sticky="ew", padx=1, pady=5)
        self.btn_zero = ttk.Button(act_frame, text='Pos "0" (Z)', state=tk.DISABLED,
                                   command=lambda: self.send_actuator_command("Z"))
        self.btn_zero.grid(row=4, column=3, sticky="ew", padx=1, pady=5)

        self.lbl_act_status = ttk.Label(act_frame, text="Status: Disconnected")
        self.lbl_act_status.grid(row=5, column=0, columnspan=2, pady=5, sticky="w")
        self.lbl_pos = ttk.Label(act_frame, text="Pos: -- mm", font=("Arial", 11, "bold"), foreground="blue")
        self.lbl_pos.grid(row=5, column=2, columnspan=2, pady=5, sticky="e")

        ttk.Label(act_frame, text="Poll Interval (s):").grid(row=6, column=0, sticky="w", pady=5)
        self.entry_poll_interval = ttk.Entry(act_frame, width=10)
        self.entry_poll_interval.grid(row=6, column=1, sticky="ew", pady=5)
        self.entry_poll_interval.insert(0, "1.0")
        self.btn_start_test = ttk.Button(act_frame, text="Start Log", command=self.start_test, state=tk.DISABLED)
        self.btn_start_test.grid(row=6, column=2, sticky="ew", padx=1, pady=5)
        self.btn_stop_test = ttk.Button(act_frame, text="Stop Log", command=self.stop_test, state=tk.DISABLED)
        self.btn_stop_test.grid(row=6, column=3, sticky="ew", padx=1, pady=5)

        self.btn_enable_quintic = ttk.Button(
            act_frame,
            text="Enable Quintic Mode...",
            state=tk.DISABLED,
            command=self.open_quintic_popup,
        )
        self.btn_enable_quintic.grid(row=7, column=0, columnspan=4, sticky="ew", pady=(8, 2))

        log_frame = ttk.LabelFrame(main_frame, text="System Log", padding=5)
        log_frame.pack(fill=tk.BOTH, expand=True)
        self.log_text = tk.Text(log_frame, height=10, state=tk.DISABLED, wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def bind_shortcuts(self):
        self.root.bind("<Up>", self.on_key_up)
        self.root.bind("<Down>", self.on_key_down)

    def on_key_up(self, event=None):
        if str(self.btn_up["state"]) == tk.NORMAL:
            self.send_actuator_command("U1")

    def on_key_down(self, event=None):
        if str(self.btn_down["state"]) == tk.NORMAL:
            self.send_actuator_command("D1")

    # ============================================================
    # LOGGING TO BOTH VIEWS
    # ============================================================
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
            widget.config(state=tk.NORMAL)
            widget.insert(tk.END, line)
            widget.see(tk.END)
            widget.config(state=tk.DISABLED)
        except tk.TclError:
            pass

    # ============================================================
    # CONNECTION + REAL SERIAL COMMANDS
    # ============================================================
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
        try:
            temp_serial = serial.Serial(port, BAUD_RATE, timeout=SERIAL_RESPONSE_TIMEOUT_SECONDS)
            self.log(f"Actuator port opened. Waiting {ACTUATOR_STARTUP_SECONDS}s for reset...")
            time.sleep(ACTUATOR_STARTUP_SECONDS)

            with self.actuator_lock:
                temp_serial.reset_input_buffer()
                self.actuator = temp_serial

            def update_ui():
                self.btn_act_conn.config(text="Actuator Connected")
                self.lbl_act_status.config(text="Status: Ready")
                for w in (
                    self.btn_e1, self.btn_e0, self.rb_s1, self.rb_s2, self.rb_s3,
                    self.speed_entry, self.btn_send_speed, self.btn_up, self.btn_pause,
                    self.btn_down, self.btn_zero, self.btn_start_test, self.btn_enable_quintic,
                ):
                    w.config(state=tk.NORMAL)

            self.root.after(0, update_ui)
            self.log("Actuator ready.")
        except Exception as exc:
            self.log(f"Actuator connection failed: {exc}")
            self.root.after(0, lambda: self._connection_failed_ui())

    def _connection_failed_ui(self):
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

    def _apply_command_to_local_state(self, cmd):
        if cmd == "U1":
            self.motion_state = "UP"
        elif cmd == "D1":
            self.motion_state = "DOWN"
        elif cmd in ("N", "E0"):
            self.motion_state = "PAUSED"
        elif cmd == "Z":
            self.current_position_mm = 0.0
        self.root.after(0, self._refresh_state_views)

    def _refresh_state_views(self):
        if self.current_position_mm is not None:
            self.lbl_pos.config(text=f"Pos: {self.current_position_mm:.2f} mm")
        self._update_popup_state()

    # ============================================================
    # NORMAL SPEED CONTROL
    # ============================================================
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

    # ============================================================
    # QUINTIC POPUP UI
    # ============================================================
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
        self.canvas = FigureCanvasTkAgg(self.figure, master=speed_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

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
        for widget in (
            self.rb_s1, self.rb_s2, self.rb_s3, self.speed_entry,
            self.btn_send_speed, self.btn_up, self.btn_pause, self.btn_down,
        ):
            try:
                widget.config(state=state)
            except tk.TclError:
                pass

    # ============================================================
    # QUINTIC MATH + VALIDATION
    # ============================================================
    @staticmethod
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

    @staticmethod
    def _quintic_blend(u):
        return 10 * u**3 - 15 * u**4 + 6 * u**5

    @staticmethod
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

    # ============================================================
    # QUINTIC EXECUTION ON REAL HARDWARE
    # ============================================================
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
                "Quintic ramp completed normally. Motor continues in the same direction at "
                f"{self._format_speed(self.current_commanded_speed_um_min)}."
            )
        elif reason == "cancelled":
            self.log(
                "Quintic ramp terminated. Current commanded speed remains "
                f"{self._format_speed(self.current_commanded_speed_um_min)}."
            )
        else:
            self.log("Quintic ramp ended because of an execution error.")

        # Keep popup open so the final state/graphs/log remain visible.
        # To run another trajectory, unlock the inputs and clear old trajectory.
        self.quintic_trajectory = []
        self.entry_quintic_target.config(state=tk.NORMAL)
        self.combo_quintic_unit.config(state="readonly")
        self.entry_quintic_duration.config(state=tk.NORMAL)
        self.btn_quintic_up.config(state=tk.DISABLED)
        self.btn_quintic_down.config(state=tk.DISABLED)
        self.btn_quintic_pause.config(state=tk.DISABLED)
        self.btn_quintic_cancel.config(state=tk.DISABLED)
        self.btn_exit_quintic.config(state=tk.NORMAL)
        self.lbl_popup_start_speed.config(text=self._format_speed(self.current_commanded_speed_um_min))
        self.lbl_quintic_summary.config(text=f"Current commanded speed: {self._format_speed(self.current_commanded_speed_um_min)}")
        self.lbl_quintic_validation.config(text="Ramp ended. Enter a new target or exit Quintic Mode.")
        self._clear_trajectory_table()
        self._update_quintic_calculate_state()
        self._update_popup_state()

    # ============================================================
    # POPUP STATE + GRAPHS
    # ============================================================
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
            pos_text = "-- mm" if self.current_position_mm is None else f"{self.current_position_mm:.2f} mm"
            self.lbl_position_large.config(text=pos_text)
        except tk.TclError:
            pass

    def _graph_refresh_tick(self):
        if not self.polling_active:
            return

        if self._popup_exists() and hasattr(self, "canvas"):
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
                    text="Position: -- mm" if position is None else f"Position: {position:.2f} mm"
                )
                self._update_popup_state()
                self.figure.tight_layout()
                self.position_figure.tight_layout()
                self.canvas.draw_idle()
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

    # ============================================================
    # CSV LOGGING
    # ============================================================
    def start_test(self):
        try:
            self.poll_interval = float(self.entry_poll_interval.get())
            if self.poll_interval <= 0:
                raise ValueError("Interval must be greater than zero.")
        except ValueError as exc:
            messagebox.showerror("Input Error", f"Invalid poll interval: {exc}")
            return

        self.test_running = True
        self.btn_start_test.config(state=tk.DISABLED)
        self.entry_poll_interval.config(state=tk.DISABLED)
        self.btn_stop_test.config(state=tk.NORMAL)

        filename = f"actuator_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.csv_file = open(filename, mode="w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "PC_Timestamp", "PC_Elapsed_s", "Actuator_Position_mm",
            "Actuator_HW_Time_ms", "Actuator_Latency_ms"
        ])
        self.start_time = time.time()
        self.log(f"Started logging actuator position to {filename} (Interval: {self.poll_interval}s)")

    def stop_test(self):
        self.test_running = False
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
        if self.actuator and self.actuator.is_open:
            self.btn_start_test.config(state=tk.NORMAL)
        self.entry_poll_interval.config(state=tk.NORMAL)
        self.btn_stop_test.config(state=tk.DISABLED)
        self.log("Data logging stopped.")

    # ============================================================
    # POSITION POLLING
    # ============================================================
    def actuator_poll_loop(self):
        while self.polling_active:
            loop_start = time.monotonic()
            pos_mm = None
            act_time_ms = None
            act_latency_ms = None

            if self.actuator and self.actuator.is_open:
                try:
                    t_start_act = time.perf_counter()
                    with self.actuator_lock:
                        self.actuator.reset_input_buffer()
                        self.actuator.write(b"P\n")
                        self.actuator.flush()
                        raw = self.actuator.readline()
                    act_latency_ms = (time.perf_counter() - t_start_act) * 1000.0

                    if raw:
                        text = raw.decode("ascii", errors="replace").strip()
                        parts = text.split(",")
                        pos_mm = float(parts[0].replace(",", "."))
                        act_time_ms = int(parts[1]) if len(parts) > 1 else None
                        self.current_position_mm = pos_mm
                        self.root.after(0, lambda p=pos_mm: self._position_ui_update(p))
                except Exception:
                    pass

            if self.test_running and self.csv_writer and pos_mm is not None:
                elapsed = time.time() - self.start_time
                self.csv_writer.writerow([
                    datetime.now().isoformat(),
                    round(elapsed, 3),
                    pos_mm,
                    act_time_ms if act_time_ms is not None else "",
                    round(act_latency_ms, 2) if act_latency_ms is not None else "",
                ])
                self.csv_file.flush()

            sleep_time = self.poll_interval - (time.monotonic() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def _position_ui_update(self, position):
        self.lbl_pos.config(text=f"Pos: {position:.2f} mm")
        self._update_popup_state()

    # ============================================================
    # SHUTDOWN
    # ============================================================
    def on_closing(self):
        self.polling_active = False
        self.quintic_stop_event.set()
        if self.test_running:
            self.stop_test()

        with self.actuator_lock:
            if self.actuator:
                try:
                    self.actuator.write(b"N\n")
                    self.actuator.write(b"E0\n")
                    self.actuator.close()
                except Exception:
                    pass
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = StandaloneActuatorApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
