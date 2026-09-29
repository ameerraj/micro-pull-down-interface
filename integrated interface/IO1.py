import tkinter as tk
from tkinter import ttk, messagebox
import threading
import time
import csv
import os
import pyvisa
import serial


class CrystalGrowthControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Micro-Pull-Down (µ-PD) Furnace Control System")
        self.root.geometry("800x600")

        # State Variables
        self.polling_active = False
        self.test_running = False
        self.ramping_active = False

        # Hardware Handles
        self.rm = pyvisa.ResourceManager()
        self.psu = None
        self.daq = None
        self.actuator = None

        # Data Logging Handles
        self.csv_file = None
        self.csv_writer = None

        # Setup GUI Components
        self._build_ui()

        # Bind Protocol
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)

    def _build_ui(self):
        """Constructs application UI elements."""
        control_frame = ttk.LabelFrame(self.root, text="System Control")
        control_frame.pack(fill=tk.X, padx=10, pady=5)

        self.btn_connect = ttk.Button(
            control_frame, text="Connect Hardware", command=self.connect_hardware
        )
        self.btn_connect.pack(side=tk.LEFT, padx=5, pady=5)

        self.btn_start_test = ttk.Button(
            control_frame,
            text="Start Logging",
            command=self.start_test,
            state=tk.DISABLED,
        )
        self.btn_start_test.pack(side=tk.LEFT, padx=5, pady=5)

        self.btn_stop_test = ttk.Button(
            control_frame,
            text="Stop Logging",
            command=self.stop_test,
            state=tk.DISABLED,
        )
        self.btn_stop_test.pack(side=tk.LEFT, padx=5, pady=5)

        # Log Output Box
        log_frame = ttk.LabelFrame(self.root, text="System Log")
        log_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=5)

        self.log_text = tk.Text(log_frame, state=tk.DISABLED, wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

    def log(self, message):
        """Appends formatted messages to the UI log display."""
        self.log_text.config(state=tk.NORMAL)
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert(tk.END, f"[{timestamp}] {message}\n")
        self.log_text.see(tk.END)
        self.log_text.config(state=tk.DISABLED)

    def connect_hardware(self):
        """Initializes connection to PSU, DAQ, and Actuator."""
        try:
            # Example VISA connection setups
            # self.psu = self.rm.open_resource("GPIB0::1::INSTR")
            # self.daq = self.rm.open_resource("USB0::0x05E6::0x6510::...::INSTR")
            # self.actuator = serial.Serial('COM3', 9600, timeout=1)

            self.log("Hardware connected successfully.")
            self.btn_start_test.config(state=tk.NORMAL)
            self.start_polling()
        except Exception as e:
            self.log(f"Connection Error: {e}")
            messagebox.showerror("Connection Error", str(e))

    def start_polling(self):
        """Starts background thread for continuous hardware reading."""
        if not self.polling_active:
            self.polling_active = True
            self.poll_thread = threading.Thread(
                target=self._hardware_poll_loop, daemon=True
            )
            self.poll_thread.start()
            self.log("Hardware polling loop started.")

    def _hardware_poll_loop(self):
        """Background thread execution loop for hardware queries."""
        while self.polling_active:
            try:
                # Place PyVISA and Serial read queries here
                # Example: temp = self.daq.query(":READ?")
                time.sleep(0.5)
            except Exception as e:
                if self.polling_active:
                    print(f"Polling loop error: {e}")
                break

    def start_test(self):
        """Initializes CSV log file and flags active test recording."""
        filename = f"growth_run_{time.strftime('%Y%m%d_%H%M%S')}.csv"
        try:
            self.csv_file = open(filename, mode="w", newline="", encoding="utf-8")
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(
                ["Timestamp", "Temperature_C", "Pressure_mbar", "Voltage_V", "Current_A"]
            )
            self.test_running = True
            self.btn_start_test.config(state=tk.DISABLED)
            self.btn_stop_test.config(state=tk.NORMAL)
            self.log(f"Data logging started: {filename}")
        except Exception as e:
            self.log(f"Failed to open log file: {e}")

    def stop_test(self):
        """Stops active data logging and safely flushes the CSV file."""
        self.test_running = False
        if self.csv_file:
            try:
                self.csv_file.flush()
                os.fsync(self.csv_file.fileno())
                self.csv_file.close()
            except Exception as e:
                print(f"Error closing CSV log file: {e}")
            finally:
                self.csv_file = None
                self.csv_writer = None

        self.btn_start_test.config(state=tk.NORMAL)
        self.btn_stop_test.config(state=tk.DISABLED)
        self.log("Data logging safely stopped.")

    def shutdown_psu(self):
        """Safely disables PSU output and resets setpoints to zero."""
        if not self.psu:
            return
        try:
            self.ramping_active = False
            self.psu.write("VOLT 0.0")
            self.psu.write("CURR 0.0")
            self.psu.write("OUTP OFF")
            self.psu.write("SYST:REM LOC")
            self.psu.close()
            print("PSU safely shut down.")
        except Exception as e:
            print(f"Error during PSU shutdown: {e}")

    def shutdown_actuator(self):
        """Closes the linear actuator serial connection."""
        if not self.actuator:
            return
        try:
            self.actuator.close()
            print("Actuator serial port closed.")
        except Exception as e:
            print(f"Error closing actuator port: {e}")

    def shutdown_daq(self):
        """Aborts DAQ scan, resets instrument, and closes VISA session."""
        if not self.daq:
            return
        try:
            self.daq.write(":ABOR")
            self.daq.write("*RST")
            self.daq.write("*CLS")
            self.daq.close()
            print("DAQ session safely closed.")
        except Exception as e:
            print(f"Error during DAQ shutdown: {e}")

    def on_closing(self):
        """Graceful, thread-safe application teardown sequence."""
        # Unbind window close protocol to prevent concurrent close attempts
        self.root.protocol("WM_DELETE_WINDOW", lambda: None)

        # 1. Stop high-level operations
        self.polling_active = False
        if self.test_running:
            self.stop_test()

        # 2. Allow polling loop thread to finish its current iteration
        if hasattr(self, "poll_thread") and self.poll_thread.is_alive():
            self.poll_thread.join(timeout=2.0)
            if self.poll_thread.is_alive():
                print("Warning: Hardware poll thread did not exit cleanly within timeout.")

        # 3. Shutdown hardware connections in priority order
        self.shutdown_psu()
        self.shutdown_actuator()
        self.shutdown_daq()

        # 4. Clean up PyVISA Resource Manager
        try:
            self.rm.close()
        except Exception as e:
            print(f"Error closing PyVISA ResourceManager: {e}")

        # 5. Destroy Tkinter root
        self.root.destroy()


if __name__ == "__main__":
    root = tk.Tk()
    app = CrystalGrowthControlApp(root)
    root.mainloop()
