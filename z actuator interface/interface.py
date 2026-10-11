import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import time
import csv
from datetime import datetime

# ============================================================
# CONFIGURATION & CONSTANTS
# ============================================================
BAUD_RATE = 9600
ACTUATOR_STARTUP_SECONDS = 5.0
SERIAL_RESPONSE_TIMEOUT_SECONDS = 0.8
DEFAULT_POLL_INTERVAL = 1.0

class StandaloneActuatorApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Arduino Actuator Controller & Logger")
        self.root.geometry("680x520") 
        
        # Hardware Thread Lock
        self.actuator_lock = threading.Lock()
        self.actuator = None
        
        # Test & Logging States
        self.test_running = False
        self.csv_file = None
        self.csv_writer = None
        self.poll_interval = DEFAULT_POLL_INTERVAL

        # Actuator State Variables
        self.selected_range = tk.StringVar(value="S1")
        self.available_com = []

        self.scan_ports() 
        self.setup_ui()
        self.bind_shortcuts()
        
        # Start background polling thread for live position updates
        self.polling_active = True
        self.poll_thread = threading.Thread(target=self.actuator_poll_loop, daemon=True)
        self.poll_thread.start()

    def scan_ports(self):
        try:
            com_ports = serial.tools.list_ports.comports()
            for p in com_ports:
                if "Arduino" in p.description or "CH340" in p.description or "CP210" in p.description:
                    self.available_com.insert(0, p.device)
                else:
                    self.available_com.append(p.device)
        except Exception as e:
            print(f"COM Port Scan Error: {e}")
            self.available_com = []

    def setup_ui(self):
        main_frame = ttk.Frame(self.root, padding=10)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # --- ACTUATOR CONTROLS FRAME ---
        act_frame = ttk.LabelFrame(main_frame, text="Arduino Motor Control & Logging", padding=10)
        act_frame.pack(fill=tk.BOTH, expand=False, pady=(0, 10))

        # Configure equal column weights for balanced layout
        for i in range(4):
            act_frame.columnconfigure(i, weight=1)

        # Row 0: Port & Connection
        ttk.Label(act_frame, text="Port:").grid(row=0, column=0, sticky="w", pady=5)
        self.combo_act_port = ttk.Combobox(act_frame, values=self.available_com, width=15)
        if self.available_com:
            self.combo_act_port.set(self.available_com[0])
        self.combo_act_port.grid(row=0, column=1, padx=5, pady=5, sticky="ew")
        
        self.btn_act_conn = ttk.Button(act_frame, text="Connect Actuator", command=self.connect_actuator_thread)
        self.btn_act_conn.grid(row=0, column=2, columnspan=2, pady=5, sticky="ew")

        # Row 1: State Controls (Enable / Disable)
        self.btn_e1 = ttk.Button(act_frame, text="Enable (E1)", state=tk.DISABLED, command=lambda: self.send_actuator_command("E1"))
        self.btn_e1.grid(row=1, column=0, columnspan=2, pady=2, sticky="ew", padx=1)
        self.btn_e0 = ttk.Button(act_frame, text="Disable (E0)", state=tk.DISABLED, command=lambda: self.send_actuator_command("E0"))
        self.btn_e0.grid(row=1, column=2, columnspan=2, pady=2, sticky="ew", padx=1)

        # Row 2: Speed Range Selection
        speed_radio_frame = ttk.Frame(act_frame)
        speed_radio_frame.grid(row=2, column=0, columnspan=4, pady=5, sticky="ew")
        
        self.rb_s1 = ttk.Radiobutton(speed_radio_frame, text="S1 (1-100 µm/min)", variable=self.selected_range, value="S1", state=tk.DISABLED, command=self._update_speed_placeholder)
        self.rb_s1.pack(side=tk.LEFT, expand=True)
        self.rb_s2 = ttk.Radiobutton(speed_radio_frame, text="S2 (100-1000 µm/min)", variable=self.selected_range, value="S2", state=tk.DISABLED, command=self._update_speed_placeholder)
        self.rb_s2.pack(side=tk.LEFT, expand=True)
        self.rb_s3 = ttk.Radiobutton(speed_radio_frame, text="S3 (1-45 mm/min)", variable=self.selected_range, value="S3", state=tk.DISABLED, command=self._update_speed_placeholder)
        self.rb_s3.pack(side=tk.LEFT, expand=True)

        # Row 3: Speed Entry & Send
        ttk.Label(act_frame, text="Speed:").grid(row=3, column=0, sticky="w", pady=5)
        self.speed_entry = ttk.Entry(act_frame, width=10, state=tk.DISABLED)
        self.speed_entry.grid(row=3, column=1, sticky="ew", pady=5)
        self.speed_entry.insert(0, "50")
        
        self.btn_send_speed = ttk.Button(act_frame, text="Set Speed", state=tk.DISABLED, command=self.send_speed_command)
        self.btn_send_speed.grid(row=3, column=2, columnspan=2, sticky="ew", padx=1, pady=5)

        # Row 4: Motion Controls (Up, Pause, Down, Pos "0")
        self.btn_up = ttk.Button(act_frame, text="▲ UP (U1)", state=tk.DISABLED, command=lambda: self.send_actuator_command("U1"))
        self.btn_up.grid(row=4, column=0, sticky="ew", padx=1, pady=5)
        
        self.btn_pause = ttk.Button(act_frame, text="⏹ PAUSE (N)", state=tk.DISABLED, command=lambda: self.send_actuator_command("N"))
        self.btn_pause.grid(row=4, column=1, sticky="ew", padx=1, pady=5)
        
        self.btn_down = ttk.Button(act_frame, text="▼ DOWN (D1)", state=tk.DISABLED, command=lambda: self.send_actuator_command("D1"))
        self.btn_down.grid(row=4, column=2, sticky="ew", padx=1, pady=5)

        self.btn_zero = ttk.Button(act_frame, text='Pos "0" (Z)', state=tk.DISABLED, command=lambda: self.send_actuator_command("Z"))
        self.btn_zero.grid(row=4, column=3, sticky="ew", padx=1, pady=5)

        # Row 5: Status and Position Readout
        self.lbl_act_status = ttk.Label(act_frame, text="Status: Disconnected")
        self.lbl_act_status.grid(row=5, column=0, columnspan=2, pady=5, sticky="w")
        
        self.lbl_pos = ttk.Label(act_frame, text="Pos: -- mm", font=("Arial", 11, "bold"), foreground="blue")
        self.lbl_pos.grid(row=5, column=2, columnspan=2, pady=5, sticky="e")

        # Row 6: Logging and Poll Interval Controls
        ttk.Label(act_frame, text="Poll Interval (s):").grid(row=6, column=0, sticky="w", pady=5)
        self.entry_poll_interval = ttk.Entry(act_frame, width=10)
        self.entry_poll_interval.grid(row=6, column=1, sticky="ew", pady=5)
        self.entry_poll_interval.insert(0, "1.0")
        
        self.btn_start_test = ttk.Button(act_frame, text="Start Log", command=self.start_test, state=tk.DISABLED)
        self.btn_start_test.grid(row=6, column=2, sticky="ew", padx=1, pady=5)
        
        self.btn_stop_test = ttk.Button(act_frame, text="Stop Log", command=self.stop_test, state=tk.DISABLED)
        self.btn_stop_test.grid(row=6, column=3, sticky="ew", padx=1, pady=5)

        # --- SYSTEM LOG CONSOLE ---
        log_frame = ttk.LabelFrame(main_frame, text="System Log", padding=5)
        log_frame.pack(fill=tk.BOTH, expand=True)

        self.log_text = tk.Text(log_frame, height=10, state=tk.DISABLED)
        self.log_text.pack(fill=tk.BOTH, expand=True)

    def bind_shortcuts(self):
        # Bind keyboard arrow keys globally to the root window
        self.root.bind("<Up>", self.on_key_up)
        self.root.bind("<Down>", self.on_key_down)

    def on_key_up(self, event=None):
        if str(self.btn_up["state"]) == tk.NORMAL:
            self.send_actuator_command("U1")

    def on_key_down(self, event=None):
        if str(self.btn_down["state"]) == tk.NORMAL:
            self.send_actuator_command("D1")

    def log(self, message):
        def append():
            self.log_text.config(state=tk.NORMAL)
            self.log_text.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {message}\n")
            self.log_text.see(tk.END)
            self.log_text.config(state=tk.DISABLED)
        self.root.after(0, append)

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
            with self.actuator_lock:
                temp_serial = serial.Serial(port, BAUD_RATE, timeout=SERIAL_RESPONSE_TIMEOUT_SECONDS)
                
            self.log(f"Actuator port opened. Waiting {ACTUATOR_STARTUP_SECONDS}s for reset...")
            
            time.sleep(ACTUATOR_STARTUP_SECONDS)
            with self.actuator_lock:
                temp_serial.reset_input_buffer()
                self.actuator = temp_serial
            
            def update_ui():
                self.btn_act_conn.config(text="Actuator Connected")
                self.lbl_act_status.config(text="Status: Ready")
                
                widgets = [
                    self.btn_e1, self.btn_e0, self.rb_s1, self.rb_s2, self.rb_s3,
                    self.speed_entry, self.btn_send_speed, self.btn_up,
                    self.btn_pause, self.btn_down, self.btn_zero, self.btn_start_test
                ]
                for w in widgets:
                    w.config(state=tk.NORMAL)
                    
            self.root.after(0, update_ui)
            self.log("Actuator ready.")
        except Exception as e:
            self.log(f"Actuator connection failed: {e}")
            self.root.after(0, lambda: [
                self.btn_act_conn.config(state=tk.NORMAL),
                self.combo_act_port.config(state=tk.NORMAL),
                self.lbl_act_status.config(text="Status: Disconnected")
            ])

    def send_actuator_command(self, cmd):
        if self.actuator and self.actuator.is_open:
            try:
                with self.actuator_lock:
                    self.actuator.write(f"{cmd}\n".encode("utf-8"))
                self.log(f"Sent to Actuator: {cmd}")
            except Exception as e:
                self.log(f"Actuator Write Error: {e}")

    def _update_speed_placeholder(self):
        r = self.selected_range.get()
        self.speed_entry.delete(0, tk.END)
        if r == "S1":
            self.speed_entry.insert(0, "50")
        elif r == "S2":
            self.speed_entry.insert(0, "500")
        elif r == "S3":
            self.speed_entry.insert(0, "20")

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
        elif r == "S2" and not (100 <= val <= 1000):
            messagebox.showwarning("Out of Range", "Range S2 requires values between 100 and 1000 µm/min.")
            return
        elif r == "S3" and not (1 <= val <= 45):
            messagebox.showwarning("Out of Range", "Range S3 requires values between 1 and 45 mm/min.")
            return

        cmd = f"{r}{val}"
        self.send_actuator_command(cmd)

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
        
        filename = f"actuator_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        self.csv_file = open(filename, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file)
        
        headers = ["PC_Timestamp", "PC_Elapsed_s", "Actuator_Position_mm", "Actuator_HW_Time_ms", "Actuator_Latency_ms"]
        self.csv_writer.writerow(headers)
        
        self.start_time = time.time()
        self.log(f"Started logging actuator position to {filename} (Interval: {self.poll_interval}s)")

    def stop_test(self):
        self.test_running = False
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
            
        self.btn_start_test.config(state=tk.NORMAL)
        self.entry_poll_interval.config(state=tk.NORMAL)
        self.btn_stop_test.config(state=tk.DISABLED)
        self.log("Data logging stopped.")

    def actuator_poll_loop(self):
        while self.polling_active:
            loop_start = time.time()
            
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
                    act_latency_ms = (time.perf_counter() - t_start_act) * 1000
                    
                    if raw:
                        text = raw.decode("ascii", errors="replace").strip()
                        parts = text.split(",")
                        pos_mm = float(parts[0].replace(",", "."))
                        act_time_ms = int(parts[1]) if len(parts) > 1 else None
                        
                        self.root.after(0, lambda p=pos_mm: self.lbl_pos.config(text=f"Pos: {p:.2f} mm"))
                except Exception:
                    pass

            if self.test_running and self.csv_writer and pos_mm is not None:
                elapsed = time.time() - self.start_time
                current_time_str = datetime.now().isoformat()
                self.csv_writer.writerow([
                    current_time_str,
                    round(elapsed, 3),
                    pos_mm,
                    act_time_ms if act_time_ms is not None else "",
                    round(act_latency_ms, 2) if act_latency_ms is not None else ""
                ])
                self.csv_file.flush()

            # Enforce poll interval
            sleep_time = self.poll_interval - (time.time() - loop_start)
            if sleep_time > 0:
                time.sleep(sleep_time)

    def on_closing(self):
        self.polling_active = False
        if self.test_running:
            self.stop_test()
            
        with self.actuator_lock:
            if self.actuator:
                try:
                    self.actuator.write(b"N\n")
                    self.actuator.write(b"E0\n")
                    self.actuator.close()
                except: 
                    pass
                
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = StandaloneActuatorApp(root)
    root.protocol("WM_DELETE_WINDOW", app.on_closing)
    root.mainloop()
