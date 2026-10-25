import pyvisa

def test_connection():
    print("Initializing PyVISA Resource Manager...")
    try:
        # Initialize ResourceManager with the PySerial backend '@py'
        rm = pyvisa.ResourceManager("@py")
        
        # List all available ports/resources to verify your hardware is detected
        resources = rm.list_resources()
        print(f"Available VISA resources: {resources}")
        
        target_port = "ASRL4::INSTR"
        if target_port not in resources:
            print(f"\n[Warning] {target_port} was not found in the available resources list.")
            print("Please check your USB/Serial cable connection and device manager.")
        
        print(f"\nAttempting to connect to {target_port}...")
        psu = rm.open_resource(
            target_port,
            baud_rate=115200,
            data_bits=8,
            parity=pyvisa.constants.Parity.none,
            stop_bits=pyvisa.constants.StopBits.one,
            read_termination="\r\n",
            write_termination="\r",
            timeout=2000
        )
        
        # Select multidrop instrument address (matching your app)
        psu.write("INST:NSEL 6")
        
        # Query basic status or IDN
        try:
            idn = psu.query("*IDN?")
            print(f"Success! Instrument ID: {idn.strip()}")
        
            v_meas = psu.query("MEAS:VOLT?")
            print(f"Success! Current voltage measurement: {v_meas.strip()} V")
        except Exception:
            # Fallback to a measurement query if *IDN? isn't supported
            v_meas = psu.query("MEAS:VOLT?")
            print(f"Success! Current voltage measurement: {v_meas.strip()} V")
            
        psu.write("SYST:REM LOC")
        psu.close()
        print("Connection closed cleanly.")

    except Exception as e:
        print(f"\nConnection failed: {e}")
        print("\nTroubleshooting tips:")
        print("1. Ensure 'pyserial' is installed (pip install pyserial).")
        print("2. Verify that no other application (like NI-MAX or another script) has the COM port locked.")

if __name__ == "__main__":
    test_connection()
