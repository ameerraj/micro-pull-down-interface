import asyncio
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
