import socket
import time

class HandEGripper:
    def __init__(self, robot_ip):
        self.robot_ip = robot_ip
        self.socket = None
        
    def connect(self):
        """Connect to the gripper socket server"""
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket.connect((self.robot_ip, 63352))
        print(f"Connected to gripper at {self.robot_ip}:63352")
        
    def send_command(self, cmd):
        """Send a command and return response"""
        self.socket.send(f"{cmd}\n".encode('utf-8'))
        time.sleep(0.01)  # Small delay for response
        
    def get_response(self):
        """Get response from gripper"""
        try:
            response = self.socket.recv(1024).decode('utf-8')
            return response.strip()
        except:
            return None
    
    def activate(self):
        """Activate the gripper"""
        print("Activating gripper...")
        self.send_command("SET ACT 1")
        time.sleep(0.5)
        self.send_command("SET GTO 1")
        time.sleep(1.0)
        print("Gripper activated")
        
    def close(self, speed=255, force=255):
        """Close gripper (POS=255 is fully closed)"""
        self.send_command(f"SET SPE {speed}")
        self.send_command(f"SET FOR {force}")
        self.send_command("SET POS 255")
        
    def open(self, speed=255, force=255):
        """Open gripper (POS=0 is fully open)"""
        self.send_command(f"SET SPE {speed}")
        self.send_command(f"SET FOR {force}")
        self.send_command("SET POS 0")
        
    def move_to(self, position, speed=255, force=255):
        """
        Move to specific position
        position: 0 (open) to 255 (closed)
        speed: 0-255
        force: 0-255
        """
        self.send_command(f"SET SPE {speed}")
        self.send_command(f"SET FOR {force}")
        self.send_command(f"SET POS {position}")
        
    def get_position(self):
        """Get current gripper position"""
        self.send_command("GET POS")
        response = self.get_response()
        if response:
            try:
                return int(response.split()[-1])
            except:
                return None
        return None
    
    def get_status(self):
        """Get gripper status"""
        self.send_command("GET STA")
        return self.get_response()
    
    def is_object_detected(self):
        """Check if object is gripped"""
        self.send_command("GET OBJ")
        response = self.get_response()
        if response:
            obj_status = int(response.split()[-1])
            # 1 or 2 means object detected
            return obj_status in [1, 2]
        return False
    
    def disconnect(self):
        """Close socket connection"""
        if self.socket:
            self.socket.close()
            print("Disconnected from gripper")


# Usage example
if __name__ == "__main__":
    ROBOT_IP = "192.168.1.191"  # Change to your robot IP
    
    gripper = HandEGripper(ROBOT_IP)
    gripper.connect()
    
    # Activate gripper
    gripper.activate()
    
    # Open gripper
    print("Opening gripper...")
    gripper.open()
    time.sleep(2)
    
    # Close gripper slowly
    print("Closing gripper slowly...")
    gripper.close(speed=50, force=100)
    time.sleep(2)
    
    # Check if object detected
    if gripper.is_object_detected():
        print("Object detected!")
    else:
        print("No object detected")
    
    # Get current position
    pos = gripper.get_position()
    print(f"Current position: {pos}")
    
    gripper.disconnect()