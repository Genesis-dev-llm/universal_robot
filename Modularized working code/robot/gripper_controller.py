"""
Hand-E Gripper Controller
Manages socket communication and control of Robotiq Hand-E gripper

Features:
- Non-blocking threaded operation
- Automatic activation sequence
- Position, speed, and force control
- Connection recovery and error handling
- Thread-safe command queue
"""

import socket
import time
import threading
from queue import Queue, Empty
from datetime import datetime

from config.constants import (
    GRIPPER_PORT, GRIPPER_TIMEOUT, GRIPPER_RECONNECT_ATTEMPTS,
    GRIPPER_RECONNECT_COOLDOWN, GRIPPER_MIN_POSITION, GRIPPER_MAX_POSITION,
    GRIPPER_DEFAULT_SPEED, GRIPPER_DEFAULT_FORCE, GRIPPER_MIN_SPEED,
    GRIPPER_MAX_SPEED, GRIPPER_MIN_FORCE, GRIPPER_MAX_FORCE,
    GRIPPER_UPDATE_INTERVAL, FLEX_DEADZONE
)
from core.error_handler import RobotError


class GripperController:
    """
    Controls Robotiq Hand-E gripper via socket communication
    Thread-safe, non-blocking operation
    """
    
    def __init__(self, robot_ip, config_manager, log_file=None):
        """
        Initialize gripper controller
        
        Args:
            robot_ip: IP address of robot (gripper listens on same IP)
            config_manager: ConfigManager instance for settings
            log_file: Optional log file for command logging
        """
        self.robot_ip = robot_ip
        self.config_manager = config_manager
        self.log_file = log_file
        
        # Socket connection
        self.socket = None
        self.connected = False
        self.activated = False
        
        # Control parameters
        self.current_speed = config_manager.get('gripper', 'default_speed', GRIPPER_DEFAULT_SPEED)
        self.current_force = config_manager.get('gripper', 'default_force', GRIPPER_DEFAULT_FORCE)
        self.current_position = 0  # Start at fully open
        self.last_sent_position = -1
        
        # Rate limiting
        self.last_update_time = 0
        self.update_interval = config_manager.get('gripper', 'update_interval', GRIPPER_UPDATE_INTERVAL)
        
        # Connection recovery
        self.reconnect_attempts = 0
        self.last_reconnect_attempt = 0
        
        # Threading
        self.command_queue = Queue(maxsize=10)
        self.worker_thread = None
        self.running = False
        self.thread_lock = threading.Lock()
        
        # Statistics
        self.total_commands = 0
        self.successful_commands = 0
        self.failed_commands = 0
    
    def connect(self):
        """
        Establish socket connection to gripper
        
        Returns:
            True if successful, False otherwise
        """
        try:
            print(f"Connecting to Hand-E gripper at {self.robot_ip}:{GRIPPER_PORT}...")
            
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(GRIPPER_TIMEOUT)
            self.socket.connect((self.robot_ip, GRIPPER_PORT))
            
            self.connected = True
            self.reconnect_attempts = 0
            
            print(f"✓ Gripper connected successfully")
            self.log_command(f"# Gripper connected to {self.robot_ip}:{GRIPPER_PORT}")
            
            return True
            
        except socket.timeout:
            print(f"✗ Gripper connection timeout")
            self.log_command(f"# ERROR: Gripper connection timeout")
            RobotError.log_error(self.log_file, 'E601', 
                f"Timeout connecting to {self.robot_ip}:{GRIPPER_PORT}")
            return False
            
        except ConnectionRefusedError:
            print(f"✗ Gripper connection refused - check if robot is running")
            self.log_command(f"# ERROR: Gripper connection refused")
            RobotError.log_error(self.log_file, 'E601',
                f"Connection refused to {self.robot_ip}:{GRIPPER_PORT}")
            return False
            
        except Exception as e:
            print(f"✗ Gripper connection error: {e}")
            self.log_command(f"# ERROR: Gripper connection failed - {e}")
            RobotError.log_error(self.log_file, 'E601', str(e),
                f"IP: {self.robot_ip}:{GRIPPER_PORT}")
            return False
    
    def activate(self):
        """
        Send activation sequence to gripper
        
        Returns:
            True if successful, False otherwise
        """
        if not self.connected:
            print("Cannot activate - gripper not connected")
            return False
        
        try:
            print("Activating gripper...")
            self.log_command("# Starting gripper activation sequence")
            
            # Activation sequence
            self._send_raw_command("SET ACT 1")
            time.sleep(0.5)
            self._send_raw_command("SET GTO 1")
            time.sleep(1.5)
            
            self.activated = True
            print("✓ Gripper activated")
            self.log_command("# Gripper activation complete")
            
            return True
            
        except Exception as e:
            print(f"✗ Gripper activation failed: {e}")
            self.log_command(f"# ERROR: Activation failed - {e}")
            RobotError.log_error(self.log_file, 'E602', str(e))
            self.activated = False
            return False
    
    def start_worker_thread(self):
        """Start background worker thread for command processing"""
        if self.worker_thread and self.worker_thread.is_alive():
            return
        
        self.running = True
        self.worker_thread = threading.Thread(target=self._worker_loop, daemon=True)
        self.worker_thread.start()
        print("Gripper worker thread started")
    
    def _worker_loop(self):
        """Background thread that processes command queue"""
        while self.running:
            try:
                # Get command from queue (blocking with timeout)
                command = self.command_queue.get(timeout=0.1)
                
                if command:
                    self._execute_command(command)
                    
            except Empty:
                # No commands in queue - continue
                pass
            except Exception as e:
                print(f"Gripper worker thread error: {e}")
                time.sleep(0.1)
    
    def _execute_command(self, command):
        """
        Execute a gripper command
        
        Args:
            command: Dictionary with 'position', 'speed', 'force'
        """
        if not self.connected or not self.activated:
            return
        
        try:
            position = command.get('position', 0)
            speed = command.get('speed', self.current_speed)
            force = command.get('force', self.current_force)
            
            # Send gripper commands
            self._send_raw_command(f"SET SPE {speed}")
            self._send_raw_command(f"SET FOR {force}")
            self._send_raw_command(f"SET POS {position}")
            
            self.current_position = position
            self.successful_commands += 1
            
        except Exception as e:
            self.failed_commands += 1
            if self.total_commands % 100 == 0:  # Occasional logging
                print(f"Gripper command error: {e}")
    
    def _send_raw_command(self, cmd):
        """
        Send raw command string to gripper
        
        Args:
            cmd: Command string (e.g., "SET POS 128")
        """
        if self.socket:
            try:
                self.socket.send(f"{cmd}\n".encode('utf-8'))
                time.sleep(0.01)  # Small delay for gripper processing
            except Exception as e:
                raise Exception(f"Socket send error: {e}")
    
    def update_position(self, flex_percent):
        """
        Update gripper position based on flex sensor percentage
        
        Args:
            flex_percent: Flex sensor value (0-100)
        """
        if not self.activated or not self.connected:
            return
        
        current_time = time.time()
        
        # Rate limiting
        if current_time - self.last_update_time < self.update_interval:
            return
        
        # Deadzone check
        delta = abs(flex_percent - self.last_sent_position)
        if delta < FLEX_DEADZONE and self.last_sent_position != -1:
            return
        
        # Map 0-100 percent to 0-255 gripper position
        gripper_position = int((flex_percent / 100.0) * GRIPPER_MAX_POSITION)
        gripper_position = max(GRIPPER_MIN_POSITION, min(GRIPPER_MAX_POSITION, gripper_position))
        
        # Queue command
        command = {
            'position': gripper_position,
            'speed': self.current_speed,
            'force': self.current_force
        }
        
        try:
            self.command_queue.put_nowait(command)
            self.last_sent_position = flex_percent
            self.last_update_time = current_time
            self.total_commands += 1
            
        except:
            # Queue full - skip this update
            pass
    
    def set_speed(self, speed):
        """
        Set gripper speed
        
        Args:
            speed: Speed value (0-255)
        """
        speed = max(GRIPPER_MIN_SPEED, min(GRIPPER_MAX_SPEED, speed))
        self.current_speed = speed
        self.config_manager.set(speed, 'gripper', 'default_speed')
        print(f"Gripper speed: {speed}")
    
    def set_force(self, force):
        """
        Set gripper force
        
        Args:
            force: Force value (0-255)
        """
        force = max(GRIPPER_MIN_FORCE, min(GRIPPER_MAX_FORCE, force))
        self.current_force = force
        self.config_manager.set(force, 'gripper', 'default_force')
        print(f"Gripper force: {force}")
    
    def open_gripper(self):
        """Manually open gripper"""
        if not self.activated:
            print("Gripper not activated")
            return
        
        command = {
            'position': GRIPPER_MIN_POSITION,
            'speed': self.current_speed,
            'force': self.current_force
        }
        
        try:
            self.command_queue.put_nowait(command)
            print("Gripper opening...")
            self.log_command("# Manual OPEN command")
        except:
            print("Command queue full")
    
    def close_gripper(self):
        """Manually close gripper"""
        if not self.activated:
            print("Gripper not activated")
            return
        
        command = {
            'position': GRIPPER_MAX_POSITION,
            'speed': self.current_speed,
            'force': self.current_force
        }
        
        try:
            self.command_queue.put_nowait(command)
            print("Gripper closing...")
            self.log_command("# Manual CLOSE command")
        except:
            print("Command queue full")
    
    def attempt_reconnect(self):
        """
        Attempt to reconnect to gripper
        
        Returns:
            True if successful, False otherwise
        """
        current_time = time.time()
        
        if current_time - self.last_reconnect_attempt < GRIPPER_RECONNECT_COOLDOWN:
            return False
        
        if self.reconnect_attempts >= GRIPPER_RECONNECT_ATTEMPTS:
            return False
        
        self.last_reconnect_attempt = current_time
        self.reconnect_attempts += 1
        
        print(f"Reconnecting gripper ({self.reconnect_attempts}/{GRIPPER_RECONNECT_ATTEMPTS})...")
        
        # Close existing connection
        try:
            if self.socket:
                self.socket.close()
        except:
            pass
        
        # Attempt reconnection
        if self.connect():
            return self.activate()
        
        return False
    
    def is_connected(self):
        """Check if gripper is connected"""
        return self.connected
    
    def is_activated(self):
        """Check if gripper is activated"""
        return self.activated
    
    def get_status(self):
        """
        Get gripper status dictionary
        
        Returns:
            Dictionary with gripper state
        """
        status = "READY" if (self.connected and self.activated) else "DISCONNECTED"
        if self.connected and not self.activated:
            status = "CONNECTED (Not Activated)"
        
        success_rate = (self.successful_commands / self.total_commands * 100) if self.total_commands > 0 else 0
        
        return {
            'connected': self.connected,
            'activated': self.activated,
            'status': status,
            'position': self.current_position,
            'speed': self.current_speed,
            'force': self.current_force,
            'total_commands': self.total_commands,
            'success_rate': success_rate
        }
    
    def log_command(self, message):
        """Log command to file with timestamp"""
        if self.log_file and not self.log_file.closed:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            self.log_file.write(f"[{timestamp}] {message}\n")
            self.log_file.flush()
    
    def close(self):
        """Close gripper connection and stop worker thread"""
        print("Closing gripper connection...")
        
        # Stop worker thread
        self.running = False
        if self.worker_thread:
            self.worker_thread.join(timeout=2.0)
        
        # Open gripper before closing
        if self.connected and self.activated:
            try:
                self._send_raw_command(f"SET SPE {GRIPPER_MAX_SPEED}")
                self._send_raw_command(f"SET FOR {GRIPPER_DEFAULT_FORCE}")
                self._send_raw_command(f"SET POS {GRIPPER_MIN_POSITION}")
                time.sleep(0.5)
                print("Gripper opened before shutdown")
            except:
                pass
        
        # Close socket
        try:
            if self.socket:
                self.socket.close()
                print("Gripper socket closed")
        except:
            pass
        
        self.connected = False
        self.activated = False
        
        # Log final statistics
        stats = self.get_status()
        self.log_command(f"# Gripper Statistics:")
        self.log_command(f"#   Total commands: {stats['total_commands']}")
        self.log_command(f"#   Success rate: {stats['success_rate']:.1f}%")
        self.log_command(f"# Gripper connection closed")