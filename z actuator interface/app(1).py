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
# 5 s worked reliably in the communication test.
CONTROLLER_STARTUP_SECONDS = 5.0

# Maximum time allowed for a response to P\n
SERIAL_RESPONSE_TIMEOUT_SECONDS = 0.8

BASE_DIR = Path(__file__).resolve().parent
INDEX_FILE = BASE_DIR / "index.html"


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

    Non-numeric lines are ignored. This makes the function tolerant of
    messages such as "COM Port verbunden".
    """
    # Do not allow an old/stale line to be mistaken for this request.
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
            # Also tolerate a decimal comma if firmware is ever changed.
            position = float(text.replace(",", "."))
            return position, text
        except ValueError:
            # Ignore informational/startup text and keep looking for
            # a numeric position until the timeout expires.
            continue


async def publish_state():
    await manager.broadcast({"type": "state", **state})


async def poll_actuator():
    global state

    ser = None

    try:
        state.update({
            "running": True,
            "connection": f"Opening {SERIAL_PORT}",
            "position_mm": None,
            "raw": None,
            "elapsed_seconds": 0.0,
            "remaining_seconds": DURATION_SECONDS,
            "sample_count": 0,
            "error": None,
        })
        await publish_state()

        # Opening the serial port can reset the Arduino controller.
        ser = await asyncio.to_thread(
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
        await asyncio.to_thread(ser.reset_input_buffer)

        state["connection"] = f"Connected to {SERIAL_PORT}"
        await publish_state()

        loop = asyncio.get_running_loop()
        measurement_start = loop.time()
        next_poll = measurement_start

        while not stop_event.is_set():
            now = loop.time()
            elapsed = now - measurement_start

            if elapsed >= DURATION_SECONDS:
                break

            # Keep the polling cadence tied to the monotonic clock rather
            # than sleeping one second after every serial transaction.
            wait_time = next_poll - now
            if wait_time > 0:
                try:
                    await asyncio.wait_for(stop_event.wait(), timeout=wait_time)
                    break
                except asyncio.TimeoutError:
                    pass

            request_time = loop.time()

            try:
                position, raw_text = await asyncio.to_thread(query_position, ser)

                elapsed = loop.time() - measurement_start

                state.update({
                    "position_mm": position,
                    "raw": raw_text,
                    "elapsed_seconds": round(elapsed, 3),
                    "remaining_seconds": round(
                        max(0.0, DURATION_SECONDS - elapsed), 3
                    ),
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
                    "remaining_seconds": round(
                        max(0.0, DURATION_SECONDS - elapsed), 3
                    ),
                    "error": str(exc),
                })

                await publish_state()

            # Schedule the next nominal 1-second boundary.
            next_poll += POLL_INTERVAL_SECONDS

            # If a serial operation took unusually long, do not rapidly
            # "catch up" with several immediate requests. Move to the next
            # future interval instead.
            after_request = loop.time()
            if next_poll <= after_request:
                missed = int(
                    (after_request - next_poll) // POLL_INTERVAL_SECONDS
                ) + 1
                next_poll += missed * POLL_INTERVAL_SECONDS

        elapsed = min(loop.time() - measurement_start, DURATION_SECONDS)

        state.update({
            "running": False,
            "elapsed_seconds": round(elapsed, 3),
            "remaining_seconds": round(
                max(0.0, DURATION_SECONDS - elapsed), 3
            ),
            "connection": f"Connected to {SERIAL_PORT}",
        })

        await publish_state()

    except serial.SerialException as exc:
        state.update({
            "running": False,
            "connection": "Serial connection failed",
            "error": str(exc),
        })
        await publish_state()

    except Exception as exc:
        state.update({
            "running": False,
            "connection": "Error",
            "error": str(exc),
        })
        await publish_state()

    finally:
        if ser is not None and ser.is_open:
            await asyncio.to_thread(ser.close)

        state["connection"] = "Disconnected"
        state["running"] = False
        await publish_state()


@app.get("/")
async def index():
    return FileResponse(INDEX_FILE)


@app.get("/api/status")
async def api_status():
    return state


@app.post("/api/start")
async def api_start():
    global measurement_task, stop_event

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
        # Give a newly opened page the current state immediately.
        await websocket.send_json({"type": "state", **state})

        # The browser does not need to send data; this receive loop simply
        # keeps the WebSocket alive and lets us notice a clean disconnect.
        while True:
            await websocket.receive_text()

    except WebSocketDisconnect:
        manager.disconnect(websocket)

    except Exception:
        manager.disconnect(websocket)
