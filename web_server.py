#!/usr/bin/env python
# -*- coding: utf-8 -*-

import json
import time
import math
from http.server import HTTPServer, SimpleHTTPRequestHandler
import threading
from read import M1C1MiniLidar

# Global variables to store the latest scan data
latest_scan = []
scan_lock = threading.Lock()

class LidarDataHandler(SimpleHTTPRequestHandler):
    """HTTP request handler to serve LiDAR data and static files."""
    
    def do_GET(self):
        """Handle GET requests."""
        if self.path == '/data':
            # Return LiDAR data as JSON
            self.send_response(200)
            self.send_header('Content-type', 'application/json')
            self.send_header('Access-Control-Allow-Origin', '*')  # Allow CORS
            self.end_headers()
            
            # Get the latest scan data
            with scan_lock:
                data = latest_scan
            
            # Print debug info
            print(f"Sending {len(data)} points to client")
            
            # Convert to JSON and send
            self.wfile.write(json.dumps(data).encode())
        else:
            # Serve static files
            return SimpleHTTPRequestHandler.do_GET(self)

def lidar_thread_function(lidar):
    """Thread function to continuously read data from the LiDAR."""
    global latest_scan
    
    print("LiDAR thread started")
    lidar.start_scanning()
    accum = []  # accumulate one full revolution
    last_publish = time.time()  # fallback timer
    
    try:
        while True:
            lidar.pump_serial()
            points = lidar.get_scan_points()

            if points:
                # accumulate partial packets until we roughly cover a full circle
                accum.extend(points)

                # normalize angles and check coverage
                if len(accum) > 0:
                    angs = [(a % (2 * math.pi)) for a, _ in accum]
                    span = max(angs) - min(angs)
                    if span > (2 * math.pi * 0.9):  # ~90% of a revolution
                        scan_data = [list(lidar.polar_to_cartesian(a, d)) for a, d in accum if d > 0]

                        with scan_lock:
                            latest_scan = scan_data

                        # debug
                        print(f"Published {len(scan_data)} cartesian points")
                        last_publish = time.time()

                        accum = []

                # Fallback: if we haven't published for 0.3s but have data, flush what we have
                if (time.time() - last_publish) > 0.3 and accum:
                    scan_data = [list(lidar.polar_to_cartesian(a, d)) for a, d in accum if d > 0]
                    with scan_lock:
                        latest_scan = scan_data
                    print(f"Published (timeout) {len(scan_data)} cartesian points")
                    accum = []
                    last_publish = time.time()
            else:
                # no new points this loop, just continue
                time.sleep(0.005)

            time.sleep(0.01)
    except Exception as e:
        print(f"Error in LiDAR thread: {e}")
    finally:
        lidar.stop_scanning()
        lidar.disconnect()
        print("LiDAR thread stopped")

def run_server(port=8080):
    """Run the HTTP server."""
    server_address = ('', port)
    httpd = HTTPServer(server_address, LidarDataHandler)
    print(f"Starting server on port {port}...")
    httpd.serve_forever()

if __name__ == "__main__":
    import math
    
    # Create LiDAR instance with longer timeout for debugging
    lidar = M1C1MiniLidar('/dev/tty.usbserial-A5069RR4', timeout=1.0)
    
    # Connect to the LiDAR
    if not lidar.connect():
        print("Failed to connect to LiDAR. Exiting.")
        exit(1)
    
    print("Connected to LiDAR.")
    
    # Start LiDAR thread
    lidar_thread = threading.Thread(target=lidar_thread_function, args=(lidar,))
    lidar_thread.daemon = True
    lidar_thread.start()
    
    try:
        # Run the server
        run_server()
    except KeyboardInterrupt:
        print("Server stopped by user")
    finally:
        # Clean up
        print("Shutting down...")
        # The LiDAR thread will be terminated when the main thread exits
