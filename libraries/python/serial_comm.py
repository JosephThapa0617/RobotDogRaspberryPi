import time

import serial


class SerialComm:
    def __init__(self, port, baudrate, timeout=1.0):
        """Initialize the serial connection."""
        try:
            self.ser = serial.Serial(port, baudrate, timeout=timeout)
            time.sleep(2)  # Give time for Arduino to reset
            print(f"[INFO] Connected to {port} at {baudrate} baud.")
        except serial.SerialException as e:
            print(f"[ERROR] Could not open serial port {port}: {e}")
            self.ser = None

    def is_open(self):
        """Check if serial port is open."""
        return self.ser is not None and self.ser.is_open

    def read_line(self):
        """Read a line from the serial port."""
        if self.is_open():
            try:
                line = self.ser.readline().decode("utf-8").strip()
                return line
            except Exception as e:
                print(f"[ERROR] Failed to read line: {e}")
        return ""

    def write_line(self, data):
        """Write a line to the serial port."""
        if self.is_open():
            try:
                self.ser.write((str(data) + "\n").encode("utf-8"))
                print(data)
            except Exception as e:
                print(f"[ERROR] Failed to write line: {e}")

    def close(self):
        """Close the serial port."""
        if self.is_open():
            self.ser.close()
            print("[INFO] Serial connection closed.")
