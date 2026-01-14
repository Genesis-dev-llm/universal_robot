"""
IMU communication interface
Handles Serial and WiFi communication with ESP32 IMU
"""

import serial
import socket
from core.error_handler import RobotError

class IMUInterface:
    """
    Manages communication with ESP32 IMU via Serial or WiFi
    """
    
    def __init__(self, use_wifi=False, serial_port='/dev/ttyUSB0', 
                 baud_rate=115200, wifi_host='192.168.4.1', wifi_port=3333):
        """
        Initialize IMU communication interface
        
        Args:
            use_wifi: Use WiFi (True) or Serial (False)
            serial_port: Serial port path
            baud_rate: Serial baud rate
            wifi_host: WiFi host IP
            wifi_port: WiFi port number
        """
        self.use_wifi = use_wifi
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.wifi_host = wifi_host
        self.wifi_port = wifi_port
        
        self.ser = None
        self.sock = None
        self.connected = False
        self.communication_mode = "None"
    
    def connect(self):
        """
        Establish connection to IMU
        
        Returns:
            True if successful, False otherwise
        """
        if self.use_wifi:
            return self._connect_wifi()
        else:
            return self._connect_serial()
    
    def _connect_serial(self):
        """Connect via USB Serial"""
        self.communication_mode = "USB"
        print(f"Connecting to ESP32 via USB ({self.serial_port})...")
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.02)
            self.connected = True
            print(f"Connected via USB Serial")
            return True
        except Exception as e:
            error_msg = RobotError.format_error('E104', str(e), f"Serial: {self.serial_port}")
            print(error_msg)
            print("Failed to connect. Check USB connection and port name.")
            return False
    
    def _connect_wifi(self):
        """Connect via WiFi TCP"""
        self.communication_mode = "WiFi"
        print(f"Connecting to ESP32 via WiFi ({self.wifi_host}:{self.wifi_port})...")
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.wifi_host, self.wifi_port))
            self.sock.settimeout(0.02)
            self.connected = True
            print(f"Connected via WiFi TCP")
            return True
        except Exception as e:
            error_msg = RobotError.format_error('E105', str(e), 
                f"WiFi: {self.wifi_host}:{self.wifi_port}")
            print(error_msg)
            print("Failed to connect. Check ESP32 WiFi AP and IP address.")
            return False
    
    def read_line(self):
        """
        Read a line of data from IMU
        
        Returns:
            String data line, or empty string if no data
        """
        if not self.connected:
            return ""
        
        if self.use_wifi:
            return self._read_wifi()
        else:
            return self._read_serial()
    
    def _read_serial(self):
        """Read from serial connection and drain buffer to get the LATEST line"""
        if not self.ser or not self.ser.is_open:
            return ""
        
        try:
            # Drain buffer to get the absolute latest packet
            if self.ser.in_waiting > 0:
                raw_data = self.ser.read(self.ser.in_waiting)
                decoded_data = raw_data.decode('utf-8', errors='ignore')
                
                # Get the last complete line
                if '\n' in decoded_data:
                    lines = decoded_data.split('\n')
                    # The last element might be incomplete, pick the one before it
                    for i in range(len(lines) - 1, -1, -1):
                        line = lines[i].strip()
                        if line:
                            return line
            return ""
        except Exception as e:
            # Try to reconnect
            try:
                self.ser.close()
                self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.02)
            except:
                pass
            return ""
    
    def _read_wifi(self):
        """Read from WiFi connection"""
        if not self.sock:
            return ""
        
        try:
            data = self.sock.recv(1024)
            if data:
                received = data.decode('utf-8', errors='ignore').strip()
                if '\n' in received:
                    lines = received.split('\n')
                    line = lines[-2] if len(lines) > 1 and lines[-2] else lines[-1]
                else:
                    line = received
                return line
        except socket.timeout:
            return ""
        except ConnectionResetError:
            print("WiFi connection lost. Attempting to reconnect...")
            try:
                self.sock.close()
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.sock.settimeout(0.02)
                self.sock.connect((self.wifi_host, self.wifi_port))
                print("WiFi reconnected")
            except:
                pass
            return ""
        except Exception:
            return ""
    
    def close(self):
        """Close communication connection"""
        if self.ser and self.ser.is_open:
            self.ser.close()
        if self.sock:
            try:
                self.sock.close()
            except:
                pass
        self.connected = False
    
    def is_connected(self):
        """Check if connection is active"""
        return self.connected
    
    def get_mode(self):
        """Get communication mode string"""
        return self.communication_mode