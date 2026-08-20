import asyncio
import time
from contextlib import asynccontextmanager
from datetime import datetime
from pathlib import Path

import pyvisa
import uvicorn

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse


# ============================================================
# CONFIGURATION
# ============================================================

PSU_NAME = "Main PSU"

# CHANGE THIS TO THE VISA ADDRESS OF THE AFTERHEATER PSU
VISA_RESOURCE = "USB0::0x05E6::0x0000::YYYYYYYY::INSTR"

POLL_INTERVAL_SECONDS = 1.0
VISA_TIMEOUT_MS = 3000

VOLTAGE_QUERY = "MEAS:VOLT?"
CURRENT_QUERY = "MEAS:CURR?"

SERVER_HOST = "127.0.0.1"
SERVER_PORT = 8002


# ============================================================
# GLOBAL STATE
# ============================================================

rm = None
instrument = None
connected_websockets = []

state = {
    "psu": PSU_NAME,
    "connected": False,
    "voltage": None,
    "current": None,
    "idn": None,
    "error": None,
    "timestamp": None,
}


# ============================================================
# VISA CONNECTION
# ============================================================

def connect_instrument():
    global rm, instrument

    if instrument is not None:
        return

    print(f"[{PSU_NAME}] Connecting...")

    rm = pyvisa.ResourceManager()
    instrument = rm.open_resource(VISA_RESOURCE)

    instrument.timeout = VISA_TIMEOUT_MS
    instrument.read_termination = "\n"
    instrument.write_termination = "\n"
    instrument.clear()

    idn = instrument.query("*IDN?").strip()

    state["idn"] = idn
    state["connected"] = True
    state["error"] = None

    print(f"[{PSU_NAME}] Connected:")
    print(idn)


def disconnect_instrument():
    global instrument, rm

    if instrument is not None:
        try:
            instrument.close()
        except Exception:
            pass
        instrument = None

    if rm is not None:
        try:
            rm.close()
        except Exception:
            pass
        rm = None

    state["connected"] = False


# ============================================================
# HARDWARE READ
# ============================================================

def read_measurements():
    global instrument

    if instrument is None:
        connect_instrument()

    voltage_response = instrument.query(VOLTAGE_QUERY)
    current_response = instrument.query(CURRENT_QUERY)

    voltage = float(voltage_response.strip())
    current = float(current_response.strip())

    return voltage, current


# ============================================================
# WEBSOCKET BROADCAST
# ============================================================

async def broadcast_state():
    dead_connections = []

    for websocket in connected_websockets.copy():
        try:
            await websocket.send_json(state)
        except Exception:
            dead_connections.append(websocket)

    for websocket in dead_connections:
        if websocket in connected_websockets:
            connected_websockets.remove(websocket)


# ============================================================
# POLLING LOOP
# ============================================================

async def polling_loop():
    print(f"[{PSU_NAME}] Polling every {POLL_INTERVAL_SECONDS} second(s)")

    next_poll = time.monotonic()

    while True:
        try:
            voltage, current = await asyncio.to_thread(read_measurements)

            state["voltage"] = voltage
            state["current"] = current
            state["connected"] = True
            state["error"] = None

        except Exception as exc:
            print(f"[{PSU_NAME}] Communication error: {exc}")

            state["connected"] = False
            state["error"] = str(exc)
            disconnect_instrument()

        state["timestamp"] = datetime.now().isoformat(timespec="milliseconds")

        await broadcast_state()

        next_poll += POLL_INTERVAL_SECONDS
        delay = next_poll - time.monotonic()

        if delay > 0:
            await asyncio.sleep(delay)
        else:
            next_poll = time.monotonic()


# ============================================================
# FASTAPI LIFESPAN
# ============================================================

@asynccontextmanager
async def lifespan(app: FastAPI):
    polling_task = asyncio.create_task(polling_loop())

    yield

    polling_task.cancel()

    try:
        await polling_task
    except asyncio.CancelledError:
        pass

    disconnect_instrument()


app = FastAPI(
    title="Main PSU Monitor",
    lifespan=lifespan
)


# ============================================================
# WEB PAGE
# ============================================================

@app.get("/")
async def index():
    html_file = Path(__file__).with_name("index.html")
    return FileResponse(html_file)


# ============================================================
# REST API
# ============================================================

@app.get("/api/status")
async def api_status():
    return state


# ============================================================
# WEBSOCKET
# ============================================================

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    connected_websockets.append(websocket)

    print(f"[{PSU_NAME}] Browser connected")

    try:
        await websocket.send_json(state)

        while True:
            await websocket.receive_text()

    except WebSocketDisconnect:
        pass

    except Exception:
        pass

    finally:
        if websocket in connected_websockets:
            connected_websockets.remove(websocket)

        print(f"[{PSU_NAME}] Browser disconnected")


# ============================================================
# START SERVER
# ============================================================

if __name__ == "__main__":
    uvicorn.run(
        app,
        host=SERVER_HOST,
        port=SERVER_PORT
    )
