Keithley PSU FastAPI Demo
=========================

This ZIP contains two completely independent FastAPI applications:

1. afterheater/
   - app.py
   - index.html
   - runs on http://127.0.0.1:8001

2. main_psu/
   - app.py
   - index.html
   - runs on http://127.0.0.1:8002

Setup
-----

1. Install dependencies:

   python -m pip install -r requirements.txt

2. Find connected VISA instruments:

   python -c "import pyvisa; rm=pyvisa.ResourceManager(); print(rm.list_resources())"

3. Update VISA_RESOURCE in:
   - afterheater/app.py
   - main_psu/app.py

4. Start Afterheater:

   cd afterheater
   python app.py

5. In a second terminal, start Main PSU:

   cd main_psu
   python app.py

Important
---------

The default SCPI queries are:

MEAS:VOLT?
MEAS:CURR?

Confirm these commands for your exact Keithley models before operating real hardware.
