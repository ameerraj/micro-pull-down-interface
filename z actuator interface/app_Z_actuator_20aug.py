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

        self.btn_send_speed = ttk.Button(input_frame, text="Se Speed", state="disabled",
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
    root.mainloop()import asyncio
import time
from pathlib import Path
from typing import Optional

import serial
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse

# ============================================================
# ACTUATOR TEST SETTINGS
# ============================================================

SERIAL_PORT = "COM3"
BAUD_RATE = 9600

POLL_INTERVAL_SECONDS = 1.0
DURATION_SECONDS = 60.0

# Opening the COM port resets this controller.
CONTROLLER_STARTUP_SECONDS = 5.0

# Maximum time allowed for a response to P\n
SERIAL_RESPONSE_TIMEOUT_SECONDS = 0.8

BASE_DIR = Path(__file__).resolve().parent
INDEX_FILE = BASE_DIR / "index_Zactuator_20aug.html"


app = FastAPI(title="Linear Actuator Position Test")


class ConnectionManager:
    def __init__(self):
        self.connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.connections:
            self.connections.remove(websocket)

    async def broadcast(self, message: dict):
        dead_connections = []

        for websocket in self.connections:
            try:
                await websocket.send_json(message)
            except Exception:
                dead_connections.append(websocket)

        for websocket in dead_connections:
            self.disconnect(websocket)


manager = ConnectionManager()

# GLOBAL HARDWARE CONNECTION
global_ser: Optional[serial.Serial] = None

state = {
    "running": False,
    "connection": "Disconnected",
    "position_mm": None,
    "raw": None,
    "elapsed_seconds": 0.0,
    "remaining_seconds": DURATION_SECONDS,
    "sample_count": 0,
    "error": None,
    "duration_seconds": DURATION_SECONDS,
    "poll_interval_seconds": POLL_INTERVAL_SECONDS,
}

measurement_task: Optional[asyncio.Task] = None
stop_event = asyncio.Event()


def query_position(ser: serial.Serial) -> tuple[float, str]:
    """
    Send the actuator's P\\n command and return its numeric position.
    """
    ser.reset_input_buffer()
    ser.write(b"P\n")
    ser.flush()

    deadline = time.monotonic() + SERIAL_RESPONSE_TIMEOUT_SECONDS

    while True:
        remaining = deadline - time.monotonic()

        if remaining <= 0:
            raise TimeoutError("No numeric position response received.")

        ser.timeout = remaining
        raw = ser.readline()

        if not raw:
            raise TimeoutError("Serial read timed out.")

        text = raw.decode("ascii", errors="replace").strip()

        if not text:
            continue

        try:
            position = float(text.replace(",", "."))
            return position, text
        except ValueError:
            print(f"DEBUG - Ignored non-numeric response: '{text}'")
            continue


async def publish_state():
    await manager.broadcast({"type": "state", **state})


async def poll_actuator():
    global state, global_ser

    try:
        # Update state to clear the "Awaiting Manual Configuration" message
        state.update({
            "running": True,
            "connection": f"Connected to {SERIAL_PORT}",
            "position_mm": None,
            "raw": None,
            "elapsed_seconds": 0.0,
            "remaining_seconds": DURATION_SECONDS,
            "sample_count": 0,
            "error": None,
        })
        await publish_state()

        loop = asyncio.get_running_loop()
        measurement_start = loop.time()
        next_poll = measurement_start

        while not stop_event.is_set():
            now = loop.time()
            elapsed = now - measurement_start

            if elapsed >= DURATION_SECONDS:
                break

            wait_time = next_poll - now
            if wait_time > 0:
                try:
                    await asyncio.wait_for(stop_event.wait(), timeout=wait_time)
                    if stop_event.is_set():
                        break
                except asyncio.TimeoutError:
                    pass

            try:
                position, raw_text = await asyncio.to_thread(query_position, global_ser)
                elapsed = loop.time() - measurement_start

                state.update({
                    "position_mm": position,
                    "raw": raw_text,
                    "elapsed_seconds": round(elapsed, 3),
                    "remaining_seconds": round(max(0.0, DURATION_SECONDS - elapsed), 3),
                    "sample_count": state["sample_count"] + 1,
                    "error": None,
                })

                await manager.broadcast({
                    "type": "sample",
                    **state,
                    "sample_elapsed_seconds": round(elapsed, 3),
                    "sample_position_mm": position,
                })

            except Exception as exc:
                elapsed = loop.time() - measurement_start
                state.update({
                    "elapsed_seconds": round(elapsed, 3),
                    "remaining_seconds": round(max(0.0, DURATION_SECONDS - elapsed), 3),
                    "error": str(exc),
                })
                await publish_state()

            next_poll += POLL_INTERVAL_SECONDS
            after_request = loop.time()
            if next_poll <= after_request:
                missed = int((after_request - next_poll) // POLL_INTERVAL_SECONDS) + 1
                next_poll += missed * POLL_INTERVAL_SECONDS

        # Reached the end of the duration
        elapsed = min(loop.time() - measurement_start, DURATION_SECONDS)
        state.update({
            "running": False,
            "elapsed_seconds": round(elapsed, 3),
            "remaining_seconds": round(max(0.0, DURATION_SECONDS - elapsed), 3),
        })

    except Exception as exc:
        state.update({
            "running": False,
            "error": str(exc),
        })

    finally:
        # We NO LONGER close the serial port here! 
        # This keeps the connection alive so you can start another measurement without resetting.
        state["running"] = False
        if global_ser and global_ser.is_open:
            state["connection"] = f"Connected to {SERIAL_PORT}"
        else:
            state["connection"] = "Disconnected"
            
        await publish_state()


@app.get("/")
async def index():
    return FileResponse(INDEX_FILE)


@app.get("/api/status")
async def api_status():
    return state


@app.post("/api/connect")
async def api_connect():
    """Handles the initial hardware connection and auto-reset."""
    global global_ser, state

    if global_ser is not None and global_ser.is_open:
        return {"ok": True, "message": "Already connected."}

    try:
        state.update({
            "connection": f"Opening {SERIAL_PORT}",
            "error": None
        })
        await publish_state()

        global_ser = await asyncio.to_thread(
            serial.Serial,
            SERIAL_PORT,
            BAUD_RATE,
            timeout=SERIAL_RESPONSE_TIMEOUT_SECONDS,
        )

        state["connection"] = f"Connected to {SERIAL_PORT} — controller starting"
        await publish_state()

        # Let the controller finish its reset/startup sequence.
        await asyncio.sleep(CONTROLLER_STARTUP_SECONDS)

        # Remove startup text such as "COM Port verbunden".
        await asyncio.to_thread(global_ser.reset_input_buffer)

        # Trigger the UI to ask for manual configuration
        state["connection"] = "Awaiting Manual Configuration"
        await publish_state()

        return {"ok": True, "message": "Connected and awaiting configuration."}

    except serial.SerialException as exc:
        state.update({
            "connection": "Serial connection failed",
            "error": str(exc),
        })
        await publish_state()
        raise HTTPException(status_code=500, detail=str(exc))


@app.post("/api/start")
async def api_start():
    global measurement_task, stop_event, global_ser

    # Check if the hardware was connected first
    if not global_ser or not global_ser.is_open:
        raise HTTPException(status_code=400, detail="Hardware is not connected. Please click Connect first.")

    if measurement_task is not None and not measurement_task.done():
        raise HTTPException(status_code=409, detail="Measurement is already running.")

    stop_event = asyncio.Event()
    measurement_task = asyncio.create_task(poll_actuator())

    return {
        "ok": True,
        "message": "Measurement started.",
        "duration_seconds": DURATION_SECONDS,
        "poll_interval_seconds": POLL_INTERVAL_SECONDS,
    }


@app.post("/api/stop")
async def api_stop():
    if measurement_task is None or measurement_task.done():
        return {"ok": True, "message": "No measurement is running."}

    stop_event.set()
    return {"ok": True, "message": "Stop requested."}


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await manager.connect(websocket)

    try:
        await websocket.send_json({"type": "state", **state})

        while True:
            await websocket.receive_text()

    except WebSocketDisconnect:
        manager.disconnect(websocket)
    except Exception:
        manager.disconnect(websocket)


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="127.0.0.1", port=8000)
