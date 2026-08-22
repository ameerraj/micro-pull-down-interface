import asyncio
import csv
from datetime import datetime
from pathlib import Path
import time
from typing import Any

import pyvisa
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.responses import HTMLResponse
from pyvisa.errors import VisaIOError


# ============================================================
# USER-ADJUSTABLE DAQ SETTINGS
# ============================================================

RESOURCE_ADDRESS = "USB0::0x05E6::0x6510::04437244::INSTR"

CHANNELS = [101, 102, 103, 104, 105]
CHANNEL_LIST = "(@101:105)"

# Total acquisition duration.
DURATION_MINUTES = 30.0

# Time between complete five-channel scans.
POLL_INTERVAL_SECONDS = 1.0

# Integration time.
# Higher values generally improve accuracy but increase scan time.
NPLC = 1.0

# Maximum permitted duration of one complete scan.
SCAN_TIMEOUT_SECONDS = 10.0

# VISA communication timeout in milliseconds.
VISA_TIMEOUT_MS = 5_000


# ============================================================
# TEMPERATURE CALIBRATION
# ============================================================

TEMPERATURE_CALIBRATION = {
    101: {"gain": 100.0, "offset": 0.0},
    102: {"gain": 100.0, "offset": 0.0},
    103: {"gain": 200.0, "offset": 0.0},
    104: {"gain": 100.0, "offset": 0.0},
    105: {"gain": 100.0, "offset": 0.0},
}


def voltage_to_temperature(channel: int, voltage: float) -> float:
    """
    Convert a channel voltage into temperature in degrees Celsius.
    """
    calibration = TEMPERATURE_CALIBRATION.get(channel)

    if calibration is None:
        raise ValueError(f"No temperature calibration is configured for channel {channel}.")

    gain = float(calibration["gain"])
    offset = float(calibration["offset"])

    return voltage * gain + offset


# ============================================================
# FASTAPI CONFIGURATION
# ============================================================

app = FastAPI(title="Keithley DAQ6510 Live Monitor")

BASE_DIRECTORY = Path(__file__).resolve().parent
INDEX_FILE = BASE_DIRECTORY / "index.html"


class ConnectionManager:
    """
    Store and manage connected WebSocket browser clients.
    """
    def __init__(self) -> None:
        self.active_connections: list[WebSocket] = []

    async def connect(self, websocket: WebSocket) -> None:
        await websocket.accept()
        if websocket not in self.active_connections:
            self.active_connections.append(websocket)
        print(f"Web browser connected. Active clients: {len(self.active_connections)}")

    def disconnect(self, websocket: WebSocket) -> None:
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
        print(f"Browser disconnected. Active clients: {len(self.active_connections)}")

    async def broadcast(self, payload: dict[str, Any]) -> None:
        if not self.active_connections:
            return

        disconnected_connections: list[WebSocket] = []

        for connection in list(self.active_connections):
            try:
                await connection.send_json(payload)
            except Exception as error:
                print(f"WebSocket send error: {error}")
                disconnected_connections.append(connection)

        for connection in disconnected_connections:
            if connection in self.active_connections:
                self.active_connections.remove(connection)


manager = ConnectionManager()


current_status: dict[str, Any] = {
    "type": "status",
    "status": "starting",
    "message": "DAQ acquisition is starting.",
}


async def set_system_status(status: str, message: str) -> None:
    """
    Store and broadcast the current DAQ system status.
    """
    global current_status
    current_status = {
        "type": "status",
        "status": status,
        "message": message,
    }
    await manager.broadcast(current_status)


# ============================================================
# WEB ROUTES
# ============================================================

@app.get("/", response_class=HTMLResponse)
async def get_index() -> HTMLResponse:
    if not INDEX_FILE.exists():
        return HTMLResponse(
            content="<h1>index.html not found</h1><p>Place index.html in the same folder as app.py.</p>",
            status_code=500,
        )
    return HTMLResponse(content=INDEX_FILE.read_text(encoding="utf-8"))


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket) -> None:
    await manager.connect(websocket)

    try:
        await websocket.send_json(current_status)

        while True:
            message = await websocket.receive_text()

            if message == "ping":
                await websocket.send_json({
                    "type": "pong",
                    "timestamp": datetime.now().isoformat(),
                })

            elif message == "disconnect":
                await websocket.close(code=1000)
                break
                
            elif message == "stop":
                acquisition_task = getattr(app.state, "acquisition_task", None)
                if acquisition_task is not None and not acquisition_task.done():
                    acquisition_task.cancel()
                    await set_system_status("stopped", "DAQ manually stopped by user.")

            else:
                print(f"Browser message: {message}")

    except WebSocketDisconnect:
        pass
    except Exception as error:
        print(f"WebSocket runtime error: {error}")
    finally:
        manager.disconnect(websocket)


# ============================================================
# DAQ6510 SUPPORT FUNCTIONS
# ============================================================

def is_no_error(error_message: str) -> bool:
    message = error_message.strip()
    return message.startswith("0,") or message.startswith("+0,") or "No error" in message


def get_instrument_errors(daq, maximum_errors: int = 10) -> list[str]:
    errors: list[str] = []
    for _ in range(maximum_errors):
        error_message = daq.query(":SYST:ERR?").strip()
        if is_no_error(error_message):
            break
        errors.append(error_message)
    return errors


def wait_for_scan(daq, timeout_seconds: float) -> None:
    start_time = time.monotonic()
    while True:
        trigger_state = daq.query(":TRIG:STAT?").strip().upper()
        if "IDLE" in trigger_state:
            return

        elapsed = time.monotonic() - start_time
        if elapsed > timeout_seconds:
            daq.write(":ABOR")
            raise TimeoutError(
                f"DAQ6510 scan timed out after {timeout_seconds:.1f} seconds. "
                f"Last trigger state: {trigger_state}"
            )
        time.sleep(0.02)


def read_all_channels(daq) -> list[float]:
    daq.write(':TRAC:CLEAR "defbuffer1"')
    daq.write(":INIT")
    wait_for_scan(daq, timeout_seconds=SCAN_TIMEOUT_SECONDS)

    readings = daq.query_ascii_values(
        f':TRAC:DATA? 1, {len(CHANNELS)}, "defbuffer1", READ'
    )

    if len(readings) != len(CHANNELS):
        raise RuntimeError(f"Expected {len(CHANNELS)} readings, received {len(readings)}: {readings}")

    return [float(reading) for reading in readings]


def configure_daq(daq) -> None:
    print("Resetting and configuring the DAQ6510...")
    daq.write(":ABOR")
    daq.write("*RST")
    daq.query("*OPC?")
    daq.write("*CLS")

    terminal_position = daq.query(":ROUT:TERM?").strip()
    print(f"Selected terminals: {terminal_position}")

    if "REAR" not in terminal_position.upper():
        raise RuntimeError("The DAQ6510 physical TERMINALS switch is not set to REAR.")

    daq.write(f':SENS:FUNC "VOLT:DC", {CHANNEL_LIST}')
    daq.write(f":SENS:VOLT:DC:RANG:AUTO ON, {CHANNEL_LIST}")
    daq.write(f":SENS:VOLT:DC:NPLC {NPLC}, {CHANNEL_LIST}")
    daq.write(f":ROUT:SCAN:CRE {CHANNEL_LIST}")
    daq.write(":ROUT:SCAN:COUNT:SCAN 1")
    daq.query("*OPC?")

    scan_list = daq.query(":ROUT:SCAN?").strip()
    print(f"Configured scan list: {scan_list}")

    setup_errors = get_instrument_errors(daq)
    if setup_errors:
        raise RuntimeError("DAQ6510 configuration error(s): " + " | ".join(setup_errors))


def setup_instrument():
    """Synchronous function to safely open and configure the DAQ."""
    rm = pyvisa.ResourceManager()
    daq = rm.open_resource(RESOURCE_ADDRESS)
    
    daq.timeout = VISA_TIMEOUT_MS
    daq.write_termination = "\n"
    daq.read_termination = "\n"
    
    identification = daq.query("*IDN?").strip()
    print(f"Connected to: {identification}")
    
    configure_daq(daq)
    return rm, daq


# ============================================================
# ACQUISITION LOOP
# ============================================================

async def acquisition_loop() -> None:
    rm = None
    daq = None

    try:
        await set_system_status("starting", "Opening the DAQ6510 connection.")
        print(f"Opening connection to: {RESOURCE_ADDRESS}")

        # Connect and configure the instrument on a background thread so it doesn't freeze the web server
        rm, daq = await asyncio.to_thread(setup_instrument)

        duration_seconds = DURATION_MINUTES * 60.0
        test_start = time.monotonic()
        test_end = test_start + duration_seconds
        next_poll_time = test_start
        scan_number = 0

        print("\nLive polling started successfully.")
        print(f"Duration: {DURATION_MINUTES} minute(s)")
        print(f"Polling interval: {POLL_INTERVAL_SECONDS} second(s)")
        print(f"Channels: {CHANNELS}")
        print("-" * 150)
        
        # ==========================================
        # SETUP LIVE CSV LOGGING
        # ==========================================
        file_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        log_file_path = BASE_DIRECTORY / f"daq_log_{file_timestamp}.csv"
        
        headers = ["Timestamp", "Sequence", "Elapsed_Seconds"]
        for channel in CHANNELS:
            headers.extend([f"CH{channel}_Voltage_V", f"CH{channel}_Temperature_C"])
            
        with open(log_file_path, mode="w", newline="", encoding="utf-8") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(headers)
            
        print(f"Live logging to: {log_file_path.name}")
        print("-" * 150)

        await set_system_status(
            "running",
            f"Scanning channels 101–105 every {POLL_INTERVAL_SECONDS:g} second(s).",
        )

        while time.monotonic() < test_end:
            delay = next_poll_time - time.monotonic()
            if delay > 0:
                await asyncio.sleep(delay)

            if time.monotonic() >= test_end:
                break

            scan_started = time.monotonic()

            try:
                readings = await asyncio.to_thread(read_all_channels, daq)
                temperatures = [
                    voltage_to_temperature(channel, voltage)
                    for channel, voltage in zip(CHANNELS, readings)
                ]

                scan_number += 1
                elapsed = time.monotonic() - test_start
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]

                voltage_output = " | ".join(
                    f"CH {channel}: {voltage:.9g} V"
                    for channel, voltage in zip(CHANNELS, readings)
                )

                temperature_output = " | ".join(
                    f"CH {channel}: {temperature:.3f} °C"
                    for channel, temperature in zip(CHANNELS, temperatures)
                )

                print(f"{timestamp} | Scan {scan_number:04d} | Elapsed {elapsed:8.2f} s")
                print(f"Voltage:     {voltage_output}")
                print(f"Temperature: {temperature_output}")

                payload = {
                    "type": "measurement",
                    "timestamp": timestamp,
                    "sequence": scan_number,
                    "elapsed_seconds": elapsed,
                    "readings": {
                        str(channel): float(voltage)
                        for channel, voltage in zip(CHANNELS, readings)
                    },
                    "temperatures": {
                        str(channel): float(temperature)
                        for channel, temperature in zip(CHANNELS, temperatures)
                    },
                }

                await manager.broadcast(payload)
                
                # ==========================================
                # APPEND LIVE DATA TO CSV
                # ==========================================
                csv_row = [timestamp, scan_number, round(elapsed, 3)]
                for voltage, temperature in zip(readings, temperatures):
                    csv_row.extend([voltage, temperature])
                    
                with open(log_file_path, mode="a", newline="", encoding="utf-8") as csv_file:
                    writer = csv.writer(csv_file)
                    writer.writerow(csv_row)

            except Exception as scan_error:
                timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3]
                print(f"{timestamp} | Scan error: {scan_error}")
                await set_system_status("warning", f"DAQ scan error: {scan_error}")

                try:
                    await asyncio.to_thread(daq.write, ":ABOR")
                except Exception:
                    pass

            next_poll_time += POLL_INTERVAL_SECONDS
            current_time = time.monotonic()

            if next_poll_time < current_time:
                scan_duration = current_time - scan_started
                print(
                    f"Warning: scan required {scan_duration:.3f} seconds. "
                    f"Requested interval is {POLL_INTERVAL_SECONDS:.3f} seconds."
                )
                next_poll_time = current_time

        total_elapsed = time.monotonic() - test_start

        print("-" * 150)
        print("Live polling completed.")
        print(f"Completed scans: {scan_number}")
        print(f"Total elapsed time: {total_elapsed:.2f} seconds")

        await set_system_status(
            "completed",
            f"Acquisition completed after {scan_number} scans.",
        )

    except asyncio.CancelledError:
        print("DAQ acquisition task cancelled.")
        await set_system_status("stopped", "DAQ acquisition was stopped.")
        raise
    except VisaIOError as error:
        print(f"VISA communication error: {error}")
        await set_system_status("error", f"VISA communication error: {error}")
    except Exception as error:
        print(f"Critical DAQ error: {error}")
        await set_system_status("error", str(error))
    finally:
        if daq is not None:
            try:
                daq.write(":ABOR")
            except Exception:
                pass
            try:
                daq.close()
            except Exception:
                pass
            print("DAQ6510 session closed.")

        if rm is not None:
            try:
                rm.close()
            except Exception:
                pass


# ============================================================
# STARTUP AND SHUTDOWN
# ============================================================

@app.on_event("startup")
async def startup_event() -> None:
    app.state.acquisition_task = asyncio.create_task(acquisition_loop())
    print("DAQ acquisition task created.")


@app.on_event("shutdown")
async def shutdown_event() -> None:
    acquisition_task = getattr(app.state, "acquisition_task", None)
    if acquisition_task is not None:
        acquisition_task.cancel()
        try:
            await acquisition_task
        except asyncio.CancelledError:
            pass


# ============================================================
# PROGRAM ENTRY POINT
# ============================================================

if __name__ == "__main__":
    import uvicorn

    uvicorn.run(
        "app:app",  # Pass as import string rather than instance to avoid certain thread issues in Windows
        host="0.0.0.0",
        port=8000,
        log_level="info",
        ws="websockets",
        timeout_graceful_shutdown=3,
    )