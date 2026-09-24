import pyvisa
from fastapi import FastAPI, HTTPException
from fastapi.responses import HTMLResponse
from pydantic import BaseModel, Field
import uvicorn

# --- Configuration & Safety Limits ---
COM_PORT = "ASRL4::INSTR" # Update to your specific COM port
MAX_VOLTAGE = 30.0        # Update based on your specific 1500W model rating
MAX_CURRENT = 50.0        # Update based on your specific 1500W model rating
HARDWARE_OVP = 32.0       # Must be at least 105% of set voltage or OVP rating

app = FastAPI(title="Afterheater Control (GEN 1500W)")
rm = pyvisa.ResourceManager("@py")
psu = None

# --- Data Validation ---
class Setpoints(BaseModel):
    voltage: float = Field(..., ge=0, le=MAX_VOLTAGE, description="Target voltage")
    current: float = Field(..., ge=0, le=MAX_CURRENT, description="Target current")

# --- Hardware Lifecycle ---
@app.on_event("startup")
def startup_event():
    global psu
    try:
        psu = rm.open_resource(
            COM_PORT,
            baud_rate=9600,
            data_bits=8,
            parity=pyvisa.constants.Parity.none,
            stop_bits=pyvisa.constants.StopBits.one,
            read_termination="\r",
            write_termination="\r",
            timeout=2000
        )
        # 1. Address the instrument
        psu.write("ADR 6")
        
        # 2. Apply Hardware Safety (OVP)
        psu.write(f"OVP {HARDWARE_OVP}")
        
        # 3. Set safe defaults before enabling output
        psu.write("PV 0.0")
        psu.write("PC 0.0")
        psu.write("OUT 1")
        print("Afterheater connected and output enabled.")
    except Exception as e:
        print(f"Failed to connect to hardware: {e}")

@app.on_event("shutdown")
def shutdown_event():
    global psu
    if psu:
        print("Shutting down: Disabling afterheater DC output.")
        psu.write("OUT 0")
        psu.close()

# --- API Endpoints ---
@app.get("/api/status")
def get_status():
    if not psu:
        raise HTTPException(status_code=503, detail="Instrument not connected")
    
    try:
        v_meas = float(psu.query("MV?"))
        i_meas = float(psu.query("MC?"))
        v_set = float(psu.query("PV?"))
        i_set = float(psu.query("PC?"))
        return {
            "v_meas": round(v_meas, 3), 
            "i_meas": round(i_meas, 3), 
            "v_set": round(v_set, 3), 
            "i_set": round(i_set, 3)
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

@app.post("/api/set")
def update_setpoints(data: Setpoints):
    if not psu:
        raise HTTPException(status_code=503, detail="Instrument not connected")
    
    try:
        psu.write(f"PV {data.voltage}")
        psu.write(f"PC {data.current}")
        return {"status": "success", "voltage": data.voltage, "current": data.current}
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))

# --- HTML Frontend ---
@app.get("/")
def get_dashboard():
    html_content = f"""
    <!DOCTYPE html>
    <html>
    <head>
        <title>Afterheater Control</title>
        <style>
            body {{ font-family: system-ui, sans-serif; background: #121212; color: #ffffff; padding: 2rem; max-width: 600px; margin: auto; }}
            .card {{ background: #1e1e1e; padding: 1.5rem; border-radius: 8px; margin-bottom: 1rem; border: 1px solid #333; }}
            .readings {{ display: flex; justify-content: space-between; font-size: 1.5rem; }}
            .val {{ color: #f97316; font-weight: bold; }}
            input {{ width: 100px; padding: 0.5rem; background: #2d2d2d; border: 1px solid #444; color: white; border-radius: 4px; }}
            button {{ background: #ea580c; color: white; border: none; padding: 0.5rem 1rem; border-radius: 4px; cursor: pointer; }}
            button:hover {{ background: #c2410c; }}
            .error {{ color: #ef4444; margin-top: 1rem; font-size: 0.9rem; }}
        </style>
    </head>
    <body>
        <h2>Afterheater Control Panel</h2>
        
        <div class="card">
            <h3>Live Measurements</h3>
            <div class="readings">
                <div>Voltage: <span class="val" id="v_meas">0.000</span> V</div>
                <div>Current: <span class="val" id="i_meas">0.000</span> A</div>
            </div>
            <p style="color: #888; font-size: 0.9rem;">Target Setpoints: <span id="v_set">0</span> V | <span id="i_set">0</span> A</p>
        </div>

        <div class="card">
            <h3>Update Setpoints</h3>
            <p style="color: #888; font-size: 0.8rem;">Max limits: {MAX_VOLTAGE}V, {MAX_CURRENT}A</p>
            <form id="controlForm">
                <label>Voltage (V): <input type="number" id="v_input" step="0.1" min="0" max="{MAX_VOLTAGE}" required></label>
                <label style="margin-left: 1rem;">Current (A): <input type="number" id="i_input" step="0.1" min="0" max="{MAX_CURRENT}" required></label>
                <button type="submit" style="margin-left: 1rem;">Apply</button>
            </form>
            <div id="error_msg" class="error"></div>
        </div>

        <script>
            setInterval(async () => {{
                try {{
                    const res = await fetch('/api/status');
                    const data = await res.json();
                    if(res.ok) {{
                        document.getElementById('v_meas').innerText = data.v_meas.toFixed(3);
                        document.getElementById('i_meas').innerText = data.i_meas.toFixed(3);
                        document.getElementById('v_set').innerText = data.v_set.toFixed(2);
                        document.getElementById('i_set').innerText = data.i_set.toFixed(2);
                    }}
                }} catch (e) {{
                    console.error("Polling error:", e);
                }}
            }}, 1000);

            document.getElementById('controlForm').addEventListener('submit', async (e) => {{
                e.preventDefault();
                const errorMsg = document.getElementById('error_msg');
                errorMsg.innerText = "";
                
                const payload = {{
                    voltage: parseFloat(document.getElementById('v_input').value),
                    current: parseFloat(document.getElementById('i_input').value)
                }};

                try {{
                    const res = await fetch('/api/set', {{
                        method: 'POST',
                        headers: {{ 'Content-Type': 'application/json' }},
                        body: JSON.stringify(payload)
                    }});
                    const data = await res.json();
                    
                    if(!res.ok) {{
                        errorMsg.innerText = data.detail[0]?.msg || JSON.stringify(data.detail);
                    }}
                }} catch (e) {{
                    errorMsg.innerText = "Failed to communicate with server.";
                }}
            }});
        </script>
    </body>
    </html>
    """
    return HTMLResponse(content=html_content)

if __name__ == "__main__":
    uvicorn.run(app, host="127.0.0.1", port=8001)
