"""
IMU communication interface
Handles Serial and WiFi communication with ESP32 IMU

FIXES:
- Graceful connection failure
- Better error recovery
- Proper resource cleanup
- Connection status tracking
"""

import serial
import socket
import time
from core.error_handler import RobotError

class IMUInterface:
    """Manages communication with ESP32 IMU via Serial or WiFi"""
    
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
        self.last_error = None
    
    def connect(self):
        """
        Establish connection to IMU
        
        Returns:
            True if successful, False otherwise
        """
        try:
            if self.use_wifi:
                return self._connect_wifi()
            else:
                return self._connect_serial()
        except Exception as e:
            self.last_error = str(e)
            print(f"IMU connection failed: {e}")
            return False
    
    def _connect_serial(self):
        """Connect via USB Serial"""
        self.communication_mode = "USB"
        print(f"Attempting USB connection to {self.serial_port}...")
        
        try:
            self.ser = serial.Serial(
                self.serial_port, 
                self.baud_rate, 
                timeout=0.02,
                write_timeout=1.0
            )
            self.connected = True
            print(f"✓ IMU connected via USB ({self.serial_port})")
            return True
            
        except serial.SerialException as e:
            print(f"✗ USB connection failed: {e}")
            print(f"  Checked port: {self.serial_port}")
            print(f"  Available ports: Check with 'ls /dev/ttyUSB*' or 'ls /dev/ttyACM*'")
            self.last_error = str(e)
            return False
            
        except Exception as e:
            error_msg = RobotError.format_error('E104', str(e), f"Serial: {self.serial_port}")
            print(error_msg)
            self.last_error = str(e)
            return False
    
    def _connect_wifi(self):
        """Connect via WiFi TCP"""
        self.communication_mode = "WiFi"
        print(f"Attempting WiFi connection to {self.wifi_host}:{self.wifi_port}...")
        
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(5.0)
            self.sock.connect((self.wifi_host, self.wifi_port))
            self.sock.settimeout(0.02)
            self.connected = True
            print(f"✓ IMU connected via WiFi ({self.wifi_host}:{self.wifi_port})")
            return True
            
        except socket.timeout:
            print(f"✗ WiFi connection timeout")
            print(f"  Check if ESP32 AP '{self.wifi_host}' is active")
            self.last_error = "Connection timeout"
            return False
            
        except ConnectionRefusedError:
            print(f"✗ WiFi connection refused")
            print(f"  ESP32 may not be running server on port {self.wifi_port}")
            self.last_error = "Connection refused"
            return False
            
        except Exception as e:
            error_msg = RobotError.format_error('E105', str(e), 
                f"WiFi: {self.wifi_host}:{self.wifi_port}")
            print(error_msg)
            self.last_error = str(e)
            return False
    
    def read_line(self):
        """
        Read a line of data from IMU
        
        Returns:
            String data line, or empty string if no data/error
        """
        if not self.connected:
            return ""
        
        try:
            if self.use_wifi:
                return self._read_wifi()
            else:
                return self._read_serial()
        except Exception as e:
            # Silently handle read errors (common during disconnection)
            return ""
    
    def _read_serial(self):
        """Read from serial connection and get the latest line"""
        if not self.ser or not self.ser.is_open:
            self.connected = False
            return ""
        
        try:
            # Drain buffer to get the absolute latest packet
            if self.ser.in_waiting > 0:
                raw_data = self.ser.read(self.ser.in_waiting)
                decoded_data = raw_data.decode('utf-8', errors='ignore')
                
                # Get the last complete line
                if '\n' in decoded_data:
                    lines = decoded_data.split('\n')
                    # Find the last non-empty line
                    for i in range(len(lines) - 1, -1, -1):
                        line = lines[i].strip()
                        if line:
                            return line
            return ""
            
        except serial.SerialException:
            # Connection lost
            self.connected = False
            self._attempt_serial_reconnect()
            return ""
            
        except Exception:
            return ""
    
    def _read_wifi(self):
        """Read from WiFi connection"""
        if not self.sock:
            self.connected = False
            return ""
        
        try:
            data = self.sock.recv(1024)
            if data:
                received = data.decode('utf-8', errors='ignore').strip()
                if '\n' in received:
                    lines = received.split('\n')
                    # Get last complete line
                    line = lines[-2] if len(lines) > 1 and lines[-2] else lines[-1]
                else:
                    line = received
                return line
                
        except socket.timeout:
            return ""
            
        except (ConnectionResetError, BrokenPipeError):
            self.connected = False
            self._attempt_wifi_reconnect()
            return ""
            
        except Exception:
            return ""
    
    def _attempt_serial_reconnect(self):
        """Attempt to reconnect serial"""
        try:
            if self.ser:
                self.ser.close()
            time.sleep(0.5)
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.02)
            self.connected = True
            print("IMU USB reconnected")
        except:
            pass
    
    def _attempt_wifi_reconnect(self):
        """Attempt to reconnect WiFi"""
        try:
            if self.sock:
                self.sock.close()
            time.sleep(0.5)
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.settimeout(0.02)
            self.sock.connect((self.wifi_host, self.wifi_port))
            self.connected = True
            print("IMU WiFi reconnected")
        except:
            pass
    
    def close(self):
        """Close communication connection"""
        try:
            if self.ser and self.ser.is_open:
                self.ser.close()
                print("IMU USB connection closed")
        except:
            pass
        
        try:
            if self.sock:
                self.sock.close()
                print("IMU WiFi connection closed")
        except:
            pass
        
        self.connected = False
    
    def is_connected(self):
        """Check if connection is active"""
        return self.connected
    
    def get_mode(self):
        """Get communication mode string"""
        return self.communication_mode
    
    def get_last_error(self):
        """Get last error message"""
        return self.last_error if self.last_error else "No errors"