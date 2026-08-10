import asyncio
import time
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse


BASE_DIR = Path(__file__).resolve().parent

app = FastAPI(title="Monotonic Experiment Clock")


class ExperimentClock:
    """
    One monotonic timebase for the whole application.

    perf_counter_ns() is used instead of wall-clock time because it is
    monotonic and gives integer nanoseconds.
    """

    def __init__(self):
        self._start_ns = time.perf_counter_ns()

    def reset(self) -> None:
        self._start_ns = time.perf_counter_ns()

    def elapsed_ns(self) -> int:
        return time.perf_counter_ns() - self._start_ns

    def elapsed_seconds(self) -> float:
        return self.elapsed_ns() / 1_000_000_000


# This is the single clock instance shared by the application.
experiment_clock = ExperimentClock()


@app.get("/")
async def index():
    """Serve the HTML interface."""
    return FileResponse(BASE_DIR / "index.html")


@app.get("/api/time")
async def get_time():
    """
    Optional HTTP endpoint.
    Useful for testing that the server clock is running.
    """
    elapsed_ns = experiment_clock.elapsed_ns()

    return {
        "elapsed_ns": elapsed_ns,
        "elapsed_seconds": elapsed_ns / 1_000_000_000,
    }


@app.post("/api/reset")
async def reset_time():
    """Reset experiment time to zero."""
    experiment_clock.reset()

    return {
        "status": "reset",
        "elapsed_seconds": experiment_clock.elapsed_seconds(),
    }


@app.websocket("/ws/clock")
async def clock_websocket(websocket: WebSocket):
    """
    Stream snapshots of the Python monotonic clock to the browser.

    Important:
    The WebSocket update rate is only for DISPLAY.
    It is not the measurement scheduling clock.
    """
    await websocket.accept()

    try:
        while True:
            elapsed_ns = experiment_clock.elapsed_ns()

            await websocket.send_json(
                {
                    "elapsed_ns": elapsed_ns,
                    "elapsed_seconds": elapsed_ns / 1_000_000_000,
                }
            )

            # 20 display updates per second.
            # This does NOT define the experiment timing.
            await asyncio.sleep(0.05)

    except WebSocketDisconnect:
        print("Clock client disconnected")
