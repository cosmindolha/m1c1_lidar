#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial
import math
import time

class M1C1MiniLidar:
    """
    Simple driver for the M1C1-Mini 360° 2D scanning LiDAR.
    
    This driver handles communication with the M1C1-Mini LiDAR via serial port,
    extracts distance data from packets, and provides methods to access the scan data.
    """
    
    # Packet header bytes
    HEADER_1 = 0xAA
    HEADER_2 = 0x55
    
    # Maximum number of samples in a packet (sanity check)
    MAX_SAMPLES = 60
    
    # Angle correction constants
    ANGLE_CORRECTION_ENABLED = True
    ANGLE_CORRECTION_FACTOR_1 = 19.16
    ANGLE_CORRECTION_FACTOR_2 = 90.15
    
    def __init__(self, port, baudrate=115200, timeout=0.1):
        """
        Initialize the M1C1-Mini LiDAR driver.
        
        Args:
            port: Serial port name (e.g., '/dev/ttyUSB0')
            baudrate: Baud rate (default: 115200)
            timeout: Serial read timeout in seconds (default: 0.1)
        """
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial = None
        self.buffer = bytearray()
        self.is_scanning = False
        self.current_scan = []
        
    def connect(self):
        """
        Connect to the LiDAR device.
        
        Returns:
            bool: True if connection successful, False otherwise
        """
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self.timeout
            )
            return True
        except serial.SerialException as e:
            print("Error connecting to LiDAR: {}".format(e))
            return False
    
    def disconnect(self):
        """Close the serial connection to the LiDAR."""
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.is_scanning = False
    
    def start_scanning(self):
        """Start the scanning process."""
        if not self.serial or not self.serial.is_open:
            if not self.connect():
                return
        
        self.is_scanning = True
        self.buffer = bytearray()
    
    def stop_scanning(self):
        """Stop the scanning process."""
        self.is_scanning = False
    
    def check_packet_checksum(self, packet):
        """
        Validate the packet checksum.
        According to research notes: XOR each 16-bit pair of bytes except the pair 
        containing the start angle and compare with checksum bytes at position 8-9.
        """
        if len(packet) < 10:
            return False
            
        chk0 = 0
        chk1 = 0
        
        # XOR all byte pairs except the checksum itself (bytes 8-9)
        for i in range(0, len(packet), 2):
            if i == 8:  # Skip checksum bytes
                continue
            if i + 1 < len(packet):
                chk0 ^= packet[i]
                chk1 ^= packet[i + 1]
        
        # Compare with checksum at bytes 8-9
        return (chk0 == packet[8]) and (chk1 == packet[9])
    
    def extract_packet_data(self, packet):
        """
        Extract data from a packet.
        
        Args:
            packet: Raw data packet bytes
            
        Returns:
            tuple: (start_angle_rad, end_angle_rad, distances, start_of_scan)
        """
        # Extract frame type and check if it's the start of a new scan
        frame_type = packet[2]
        start_of_scan = (frame_type & 0x01) != 0
        
        # Extract number of samples
        num_samples = packet[3]
        
        # Extract start and end angles (in radians) - FIXED ENDIANNESS (little-endian)
        start_angle_raw = ((packet[5] << 8) | packet[4]) >> 1
        end_angle_raw = ((packet[7] << 8) | packet[6]) >> 1
        
        # Convert to radians - corrected formula from reference implementation
        start_angle_rad = start_angle_raw * math.pi / (64 * 180)
        end_angle_rad = end_angle_raw * math.pi / (64 * 180)
        
        # Extract distances (starting at byte 10, after the checksum)
        distances = []
        for i in range(num_samples):
            distance_idx = 10 + (i * 2)
            if distance_idx + 1 < len(packet):
                # Use little-endian byte order (consistent with angles)
                distance_raw = (packet[distance_idx + 1] << 8) | packet[distance_idx]
                # The distance is stored in the upper 14 bits (right-shift by 2)
                distance_mm = distance_raw >> 2
                
                # Filter out invalid distances (too small or too large)
                # According to the datasheet, valid range is 0.1m to 8m
                if 100 <= distance_mm <= 8000:
                    distances.append(distance_mm)
                else:
                    # Use 0 for invalid distances
                    distances.append(0)
        
        return (start_angle_rad, end_angle_rad, distances, start_of_scan)
    
    def pump_serial(self, chunk_size=2048):
        """
        Read raw bytes from the serial port into the internal buffer.
        """
        if self.serial and self.serial.is_open:
            data = self.serial.read(chunk_size)
            if data:
                self.buffer.extend(data)

    def get_scan_points(self):
        """
        Read data from the LiDAR and extract points.
        
        Returns:
            List[Tuple[float, float]]: List of (angle_rad, distance_mm) tuples
        """
        if not self.is_scanning:
            self.start_scanning()
        
        # (Serial port reading now handled by pump_serial)
        
        # Find packet headers
        points = []
        new_scan_started = False
        i = 0
        while i < len(self.buffer) - 1:
            if self.buffer[i] == self.HEADER_1 and self.buffer[i + 1] == self.HEADER_2:
                # Check if we have enough bytes for frame type and sample count
                if i + 3 < len(self.buffer):
                    # Check frame type (0x00 or 0x01 for data frames)
                    frame_type = self.buffer[i + 2]
                    if frame_type not in [0x00, 0x01]:
                        i += 1
                        continue
                    
                    num_samples = self.buffer[i + 3]
                    
                    # Sanity check for num_samples (should be reasonable)
                    if 1 <= num_samples <= self.MAX_SAMPLES:
                        packet_end = i + 10 + (num_samples * 2)
                        
                        # Check if we have a complete packet
                        if len(self.buffer) >= packet_end:
                            packet = self.buffer[i:packet_end]
                            
                            # Validate checksum
                            if not self.check_packet_checksum(packet):
                                i += 1
                                continue
                            
                            # Extract data from packet
                            start_angle, end_angle, distances, start_of_scan = self.extract_packet_data(packet)
                            
                            # Track if a new scan has started
                            if start_of_scan:
                                new_scan_started = True
                            
                            # Handle angle wrap-around
                            if end_angle < start_angle:
                                end_angle += 2 * math.pi
                            
                            # Calculate angle step
                            angle_step = (end_angle - start_angle) / (len(distances) - 1) if len(distances) > 1 else 0
                            
                            # Create points
                            for j, distance in enumerate(distances):
                                if distance > 0:  # Only include valid distances
                                    angle = start_angle + (j * angle_step)
                                    
                                    # Apply angle correction based on distance
                                    if self.ANGLE_CORRECTION_ENABLED and distance > self.ANGLE_CORRECTION_FACTOR_2:
                                        angle_correction = math.atan(
                                            self.ANGLE_CORRECTION_FACTOR_1 * 
                                            (distance - self.ANGLE_CORRECTION_FACTOR_2) / 
                                            (self.ANGLE_CORRECTION_FACTOR_2 * distance)
                                        )
                                        angle += angle_correction
                                    
                                    # Normalize angle to [0, 2π)
                                    angle = angle % (2 * math.pi)
                                    points.append((angle, distance))
                            
                            # Remove the packet from the buffer
                            del self.buffer[:packet_end]
                            i = 0
                            continue
                    else:
                        # Invalid num_samples, skip this byte
                        i += 1
                        continue
            i += 1
        
        # Limit buffer size to prevent memory issues
        if len(self.buffer) > 10000:
            del self.buffer[:-5000]
        
        return points
    
    def polar_to_cartesian(self, angle_rad, distance_mm):
        """
        Convert polar coordinates to Cartesian coordinates.
        
        Args:
            angle_rad: Angle in radians
            distance_mm: Distance in millimeters
            
        Returns:
            Tuple[float, float]: (x, y) coordinates in millimeters
        """
        # According to research notes:
        # x = distance_mm * math.sin(angle_rad)  # forward/backward axis
        # y = distance_mm * math.cos(angle_rad)  # left/right axis
        x = distance_mm * math.sin(angle_rad)
        y = distance_mm * math.cos(angle_rad)
        return (x, y)
    
    def get_cartesian_points(self, points):
        """
        Convert a list of polar points to Cartesian coordinates.
        
        Args:
            points: List of (angle_rad, distance_mm) tuples
            
        Returns:
            List[Tuple[float, float]]: List of (x, y) coordinates in millimeters
        """
        return [self.polar_to_cartesian(angle, distance) for angle, distance in points]

def main():
    """Main function to demonstrate the LiDAR driver."""
    # Create LiDAR instance
    lidar = M1C1MiniLidar('/dev/tty.usbserial-A5069RR4')
    
    try:
        # Connect to the LiDAR
        if not lidar.connect():
            print("Failed to connect to LiDAR. Exiting.")
            return
        
        print("Connected to LiDAR. Starting scanning...")
        lidar.start_scanning()
        
        # Read data for a short time
        print("Reading data for 5 seconds...")
        start_time = time.time()
        total_points = 0
        scan_count = 0
        packet_count = 0
        
        # Read for 5 seconds only
        while time.time() - start_time < 5:
            lidar.pump_serial()
            points = lidar.get_scan_points()
            if points:
                total_points += len(points)
                scan_count += 1
                
                # Print the first scan's data
                if scan_count == 1:
                    print("\nFirst scan: {} points".format(len(points)))
                    if len(points) > 0:
                        print("First 5 points (angle_rad, distance_mm):")
                        for i, (angle, distance) in enumerate(points[:5]):
                            print("  Point {}: angle={:.4f} rad, distance={} mm".format(i+1, angle, distance))
                        
                        # Convert to Cartesian coordinates
                        cartesian = lidar.get_cartesian_points(points[:5])
                        print("\nFirst 5 points (x, y):")
                        for i, (x, y) in enumerate(cartesian):
                            print("  Point {}: x={:.2f} mm, y={:.2f} mm".format(i+1, x, y))
            
            # Short delay to avoid busy-waiting
            time.sleep(0.1)
        
        # Print summary
        print("\nSummary after 5 seconds:")
        print("Total scans: {}".format(scan_count))
        print("Total points: {}".format(total_points))
        print("Total packet headers found: {}".format(packet_count))
        print("Average points per scan: {:.2f}".format(total_points / scan_count if scan_count > 0 else 0))
        print("Buffer size: {} bytes".format(len(lidar.buffer)))
        
        if total_points > 0:
            print("\nDriver is working! Successfully extracted data from the LiDAR.")
        else:
            print("\nNo points were extracted. Possible issues:")
            print("1. Serial connection issues")
            print("2. Packet structure might be different than expected")
            print("3. Checksum validation might be too strict")
        
    except KeyboardInterrupt:
        print("Interrupted by user")
    finally:
        # Clean up
        lidar.stop_scanning()
        lidar.disconnect()
        print("LiDAR disconnected")

if __name__ == "__main__":
    main()
