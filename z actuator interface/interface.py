import threading
import time
import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports

class MotorControllerApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Arduino Motor Control Dashboard")
        self.root.geometry("520x680")
        self.root.resizable(False, False)

        self.ser = None
        self.is_connected = False
        self.running_thread = False

        self.selected_range = tk.StringVar(value="S1")

        self._build_ui()
        self.refresh_ports()

    def _build_ui(self):
        # 1. Connection Panel
        conn_frame = ttk.LabelFrame(self.root, text="Serial Connection", padding=10)
        conn_frame.pack(fill="x", padx=10, pady=5)

        ttk.Label(conn_frame, text="Port:").grid(row=0, column=0, sticky="w")
        self.port_combobox = ttk.Combobox(conn_frame, state="readonly", width=15)
        self.port_combobox.grid(row=0, column=1, padx=5)

        self.refresh_btn = ttk.Button(conn_frame, text="Refresh", command=self.refresh_ports)
        self.refresh_btn.grid(row=0, column=2, padx=5)

        self.connect_btn = ttk.Button(conn_frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=3, padx=5)

        # 2. State & Operation Mode Controls
        state_frame = ttk.LabelFrame(self.root, text="System States", padding=10)
        state_frame.pack(fill="x", padx=10, pady=5)

        self.btn_e1 = ttk.Button(state_frame, text="Enable Extern (E1)", state="disabled",
                                 command=lambda: self.send_command("E1"))
        self.btn_e1.grid(row=0, column=0, padx=5, pady=3, sticky="ew")

        self.btn_e0 = ttk.Button(state_frame, text="Disable Extern (E0)", state="disabled",
                                 command=lambda: self.send_command("E0"))
        self.btn_e0.grid(row=0, column=1, padx=5, pady=3, sticky="ew")

        state_frame.columnconfigure(0, weight=1)
        state_frame.columnconfigure(1, weight=1)

        # 3. Speed Setting (S1 / S2 / S3)
        speed_frame = ttk.LabelFrame(self.root, text="Speed Selection", padding=10)
        speed_frame.pack(fill="x", padx=10, pady=5)

        range_frame = ttk.Frame(speed_frame)
        range_frame.pack(fill="x", pady=2)

        self.rb_s1 = ttk.Radiobutton(range_frame, text="S1 (1-100 µm/min)", variable=self.selected_range,
                                     value="S1", state="disabled", command=self._update_speed_placeholder)
        self.rb_s1.pack(side="left", expand=True)

        self.rb_s2 = ttk.Radiobutton(range_frame, text="S2 (100-1000 µm/min)", variable=self.selected_range,
                                     value="S2", state="disabled", command=self._update_speed_placeholder)
        self.rb_s2.pack(side="left", expand=True)

        self.rb_s3 = ttk.Radiobutton(range_frame, text="S3 (1-45 mm/min)", variable=self.selected_range,
                                     value="S3", state="disabled", command=self._update_speed_placeholder)
        self.rb_s3.pack(side="left", expand=True)

        input_frame = ttk.Frame(speed_frame)
        input_frame.pack(fill="x", pady=5)

        ttk.Label(input_frame, text="Speed Value:").pack(side="left", padx=5)
        self.speed_entry = ttk.Entry(input_frame, width=12, state="disabled")
        self.speed_entry.pack(side="left", padx=5)
        self.speed_entry.insert(0, "50")

        self.btn_send_speed = ttk.Button(input_frame, text="Set Speed", state="disabled",
                                         command=self.send_speed_command)
        self.btn_send_speed.pack(side="left", padx=5)

        # 4. Motion Controls (Direct Run Direction & Hold/Pause)
        motion_frame = ttk.LabelFrame(self.root, text="Motion Control", padding=10)
        motion_frame.pack(fill="x", padx=10, pady=5)

        self.btn_up = ttk.Button(motion_frame, text="▲ RUN UP (U1)", state="disabled",
                                 command=lambda: self.send_command("U1"))
        self.btn_up.pack(fill="x", pady=3)

        self.btn_pause = ttk.Button(motion_frame, text="⏹ PAUSE / HOLD (N)", state="disabled",
                                    command=lambda: self.send_command("N"))
        self.btn_pause.pack(fill="x", pady=3)

        self.btn_down = ttk.Button(motion_frame, text="▼ RUN DOWN (D1)", state="disabled",
                                   command=lambda: self.send_command("D1"))
        self.btn_down.pack(fill="x", pady=3)

        self.btn_pos = ttk.Button(motion_frame, text="Query Position (P)", state="disabled",
                                  command=lambda: self.send_command("P"))
        self.btn_pos.pack(fill="x", pady=3)

        # 5. Serial Monitor
        log_frame = ttk.LabelFrame(self.root, text="Serial Monitor", padding=10)
        log_frame.pack(fill="both", expand=True, padx=10, pady=5)

        self.log_text = tk.Text(log_frame, height=8, wrap="word", state="disabled", bg="#1e1e1e", fg="#00ff66")
        self.log_text.pack(fill="both", expand=True)

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    def refresh_ports(self):
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combobox["values"] = ports
        if ports:
            self.port_combobox.current(0)

    def toggle_connection(self):
        if not self.is_connected:
            port = self.port_combobox.get()
            if not port:
                messagebox.showwarning("Warning", "No serial port selected.")
                return
            try:
                self.ser = serial.Serial(port, 9600, timeout=1)
                self.is_connected = True
                self.connect_btn.config(text="Disconnect")
                self.set_controls_state("normal")
                self.log_message(f"Connected to {port} at 9600 baud.")

                self.running_thread = True
                self.read_thread = threading.Thread(target=self._read_serial, daemon=True)
                self.read_thread.start()

            except Exception as e:
                messagebox.showerror("Connection Error", str(e))
        else:
            self.close_connection()

    def set_controls_state(self, state):
        widgets = [self.btn_e1, self.btn_e0,
                   self.rb_s1, self.rb_s2, self.rb_s3,
                   self.speed_entry, self.btn_send_speed,
                   self.btn_up, self.btn_pause,
                   self.btn_down, self.btn_pos]
        for w in widgets:
            w.config(state=state)

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
        self.send_command(cmd)

    def send_command(self, cmd):
        if self.ser and self.ser.is_open:
            try:
                self.ser.write(f"{cmd}\n".encode("utf-8"))
                self.log_message(f">> Sent: {cmd}")
            except Exception as e:
                self.log_message(f"Write Error: {e}")

    def _read_serial(self):
        while self.running_thread and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline().decode("utf-8", errors="replace").strip()
                if line:
                    # Safely schedule log updates onto the Tkinter main thread
                    self.root.after(0, self.log_message, f"<< {line}")
            except Exception:
                break

    def log_message(self, msg):
        self.log_text.config(state="normal")
        self.log_text.insert("end", f"{msg}\n")
        self.log_text.see("end")
        self.log_text.config(state="disabled")

    def close_connection(self):
        self.running_thread = False
        if self.ser and self.ser.is_open:
            try:
                self.send_command("N")
                self.send_command("E0")
            except:
                pass
            self.ser.close()
        self.is_connected = False
        self.connect_btn.config(text="Connect")
        self.set_controls_state("disabled")
        self.log_message("Disconnected.")

    def on_close(self):
        self.close_connection()
        self.root.destroy()

if __name__ == "__main__":
    root = tk.Tk()
    app = MotorControllerApp(root)
    root.mainloop()
