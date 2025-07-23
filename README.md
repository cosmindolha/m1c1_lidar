M1C1-Mini LiDAR Visualization and Driver

A real-time 2D LiDAR scanning solution using the M1C1-Mini sensor, a Python driver, and a web-based visualization.

Note: This project was developed and tested on macOS.

<img width="752" height="885" alt="image" src="https://github.com/user-attachments/assets/7023a4e1-26e5-47b0-84fc-451e5d26bc43" />


Features
	•	Python Driver (M1C1MiniLidar): Parses raw sensor packets and converts polar readings to Cartesian coordinates. (See read.py)
	•	HTTP Server (web_server.py): Streams live scan data as JSON over HTTP.
	•	Web Visualization (index.html): Client-side app with pan, zoom, scan-rate controls, and live plotting of LiDAR data.
	•	Configurable Parameters: Adjust baud rate, serial port, scan frequency, and display settings via environment variables or CLI options.

Getting Started
	1.	Clone the repository

git clone https://github.com/yourusername/m1c1-mini-lidar
cd m1c1-mini-lidar


	2.	Set up a Python virtual environment

python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt


	3.	Connect the LiDAR sensor via a USB-to-serial adapter.
	4.	Run the server

python web_server.py --port 8000 --device /dev/tty.usbserial


	5.	Open the web client
Navigate to http://localhost:8000 in your browser to start viewing scans.

Platform Support

macOS (tested)
	•	Serial Device: Typically /dev/tty.usbserial-* or /dev/tty.SLAB_USBtoUART on Silicon and Intel Macs.
	•	Permissions: Ensure your user has access to the serial device. You may need to install drivers (e.g., FTDI VCP).
	•	Dependencies: Confirm compatibility with Python 3.9+.

Configuration

Configure via environment variables or CLI flags:

Option	Env Variable	Description	Default
--device	LIDAR_DEVICE	Serial port path (e.g., /dev/ttyUSB0)	/dev/ttyUSB0
--baudrate	LIDAR_BAUDRATE	Sensor baud rate	115200
--port	HTTP_PORT	HTTP server port	8000
--scan-rate	LIDAR_SCAN_RATE	Desired scans per second (if supported)	10

File Overview
	•	read.py — Core LiDAR driver implementation (packet parsing, noise filtering).
	•	web_server.py — Serves scan data and static files.
	•	index.html — Front-end visualization; depends on D3.js for rendering.
	•	requirements.txt — Python dependencies (pyserial, flask, etc.).

Troubleshooting
	•	No Data: Check serial wiring and port name. Verify the sensor is powered and connected.
	•	Permission Denied: Ensure your user has access to the serial device (see Platform Support).
	•	Web Client Blank: Open browser console for errors. Confirm server is running and CORS is permitted.

Cosmin Dolha - http://cosmindolha.com
