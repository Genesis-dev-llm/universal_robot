"""
URe5 + Hand-E Gripper Control Library
======================================
Complete Python controller for Universal Robots e-Series (URe5) with Robotiq Hand-E gripper

Requirements:
    pip install ur-rtde

Compatible with:
    - URCap version: UCG-1.8.4.4844
    - Polyscope version: 5.6.0.90886
    - Hand-E firmware: GD1-1.3.16

Author: Claude
Date: 2026-01-29
"""

import socket
import time
from rtde_control import RTDEControlInterface
from rtde_receive import RTDEReceiveInterface


class HandEGripper:
    """
    Controller for Robotiq Hand-E gripper via socket communication (port 63352)
    
    The Hand-E communicates through the URCap server running on the robot.
    All commands are sent as ASCII strings over TCP socket.
    """
    
    def __init__(self, robot_ip):
        """
        Initialize gripper controller
        
        Args:
            robot_ip (str): IP address of the UR robot
        """
        self.robot_ip = robot_ip
        self.socket = None
        self.is_activated = False
        
    def connect(self):
        """Connect to the gripper socket server on port 63352"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.robot_ip, 63352))
            self.socket.settimeout(2.0)
            print(f"✓ Connected to Hand-E gripper at {self.robot_ip}:63352")
            return True
        except Exception as e:
            print(f"✗ Failed to connect to gripper: {e}")
            return False
        
    def send_command(self, cmd):
        """
        Send a command to the gripper
        
        Args:
            cmd (str): Command string (e.g., "SET POS 255")
        """
        if not self.socket:
            print("✗ Socket not connected. Call connect() first.")
            return
            
        try:
            self.socket.send(f"{cmd}\n".encode('utf-8'))
            time.sleep(0.01)  # Small delay for command processing
        except Exception as e:
            print(f"✗ Error sending command '{cmd}': {e}")
        
    def get_response(self):
        """
        Get response from gripper
        
        Returns:
            str: Response string or None if no response
        """
        try:
            response = self.socket.recv(1024).decode('utf-8')
            return response.strip()
        except socket.timeout:
            return None
        except Exception as e:
            print(f"✗ Error receiving response: {e}")
            return None
    
    def activate(self):
        """
        Activate the gripper
        
        This must be called before the gripper can be used.
        Takes about 1-2 seconds to complete.
        """
        print("Activating Hand-E gripper...")
        self.send_command("SET ACT 1")
        time.sleep(0.5)
        self.send_command("SET GTO 1")
        time.sleep(1.5)
        self.is_activated = True
        print("✓ Gripper activated and ready")
        
    def reset(self):
        """Reset the gripper (deactivate and reactivate)"""
        print("Resetting gripper...")
        self.send_command("SET ACT 0")
        time.sleep(0.5)
        self.activate()
        
    def close(self, speed=255, force=255):
        """
        Close the gripper
        
        Args:
            speed (int): Closing speed (0-255, where 255 is fastest)
            force (int): Gripping force (0-255, corresponds to 20N-160N)
        """
        self.send_command(f"SET SPE {speed}")
        self.send_command(f"SET FOR {force}")
        self.send_command("SET POS 255")
        
    def open(self, speed=255, force=255):
        """
        Open the gripper
        
        Args:
            speed (int): Opening speed (0-255)
            force (int): Force (0-255)
        """
        self.send_command(f"SET SPE {speed}")
        self.send_command(f"SET FOR {force}")
        self.send_command("SET POS 0")
        
    def move_to(self, position, speed=255, force=255):
        """
        Move gripper to specific position
        
        Args:
            position (int): Target position (0=fully open, 255=fully closed)
            speed (int): Movement speed (0-255)
            force (int): Gripping force (0-255)
        """
        position = max(0, min(255, position))  # Clamp to valid range
        self.send_command(f"SET SPE {speed}")
        self.send_command(f"SET FOR {force}")
        self.send_command(f"SET POS {position}")
        
    def get_position(self):
        """
        Get current gripper position
        
        Returns:
            int: Current position (0-255) or None if error
        """
        self.send_command("GET POS")
        response = self.get_response()
        if response:
            try:
                return int(response.split()[-1])
            except:
                return None
        return None
    
    def get_status(self):
        """
        Get gripper status
        
        Returns:
            str: Status string
        """
        self.send_command("GET STA")
        return self.get_response()
    
    def get_object_status(self):
        """
        Get object detection status
        
        Returns:
            int: Object status code
                0 = fingers in motion
                1 = object detected while opening
                2 = object detected while closing
                3 = fingers at requested position
        """
        self.send_command("GET OBJ")
        response = self.get_response()
        if response:
            try:
                return int(response.split()[-1])
            except:
                return None
        return None
    
    def is_object_detected(self):
        """
        Check if an object is gripped
        
        Returns:
            bool: True if object detected, False otherwise
        """
        obj_status = self.get_object_status()
        # Status 1 or 2 means object detected during motion
        return obj_status in [1, 2]
    
    def is_moving(self):
        """
        Check if gripper is currently moving
        
        Returns:
            bool: True if moving, False if stopped
        """
        obj_status = self.get_object_status()
        return obj_status == 0
    
    def wait_for_motion_complete(self, timeout=5.0):
        """
        Wait until gripper motion is complete
        
        Args:
            timeout (float): Maximum time to wait in seconds
            
        Returns:
            bool: True if motion completed, False if timeout
        """
        start_time = time.time()
        while time.time() - start_time < timeout:
            if not self.is_moving():
                return True
            time.sleep(0.05)
        return False
    
    def get_current_force(self):
        """
        Get current force reading
        
        Returns:
            int: Current force value or None
        """
        self.send_command("GET FOR")
        response = self.get_response()
        if response:
            try:
                return int(response.split()[-1])
            except:
                return None
        return None
    
    def disconnect(self):
        """Close socket connection"""
        if self.socket:
            self.socket.close()
            self.socket = None
            print("✓ Disconnected from gripper")


class URe5Controller:
    """
    Controller for URe5 robot using RTDE interface
    
    Provides high-level control for robot motion with simplified interface.
    """
    
    def __init__(self, robot_ip):
        """
        Initialize robot controller
        
        Args:
            robot_ip (str): IP address of the UR robot
        """
        self.robot_ip = robot_ip
        self.rtde_c = None
        self.rtde_r = None
        
    def connect(self):
        """Connect to the robot via RTDE"""
        try:
            self.rtde_c = RTDEControlInterface(self.robot_ip)
            self.rtde_r = RTDEReceiveInterface(self.robot_ip)
            print(f"✓ Connected to URe5 robot at {self.robot_ip}")
            return True
        except Exception as e:
            print(f"✗ Failed to connect to robot: {e}")
            return False
    
    def move_j(self, joint_positions, speed=1.05, acceleration=1.4, async_mode=False):
        """
        Move robot in joint space
        
        Args:
            joint_positions (list): Target joint angles in radians [j0, j1, j2, j3, j4, j5]
            speed (float): Joint speed in rad/s
            acceleration (float): Joint acceleration in rad/s^2
            async_mode (bool): If True, don't wait for motion to complete
            
        Returns:
            bool: True if motion started successfully
        """
        return self.rtde_c.moveJ(joint_positions, speed, acceleration, async_mode)
    
    def move_l(self, pose, speed=0.25, acceleration=1.2, async_mode=False):
        """
        Move robot in linear cartesian space
        
        Args:
            pose (list): Target pose [x, y, z, rx, ry, rz] in meters and radians
            speed (float): Tool speed in m/s
            acceleration (float): Tool acceleration in m/s^2
            async_mode (bool): If True, don't wait for motion to complete
            
        Returns:
            bool: True if motion started successfully
        """
        return self.rtde_c.moveL(pose, speed, acceleration, async_mode)
    
    def get_joint_positions(self):
        """
        Get current joint positions
        
        Returns:
            list: Current joint angles in radians
        """
        return self.rtde_r.getActualQ()
    
    def get_tcp_pose(self):
        """
        Get current TCP (tool center point) pose
        
        Returns:
            list: Current pose [x, y, z, rx, ry, rz]
        """
        return self.rtde_r.getActualTCPPose()
    
    def get_tcp_force(self):
        """
        Get current TCP force
        
        Returns:
            list: Force vector [fx, fy, fz, mx, my, mz]
        """
        return self.rtde_r.getActualTCPForce()
    
    def stop(self):
        """Emergency stop - stops robot motion immediately"""
        self.rtde_c.stopScript()
        print("✓ Robot stopped")
    
    def is_moving(self):
        """
        Check if robot is currently moving
        
        Returns:
            bool: True if moving, False if stopped
        """
        # Check if actual speed is near zero
        actual_q_dot = self.rtde_r.getActualQd()
        return any(abs(speed) > 0.01 for speed in actual_q_dot)
    
    def disconnect(self):
        """Disconnect from robot"""
        if self.rtde_c:
            self.rtde_c.disconnect()
            print("✓ Disconnected from robot")


class URHandESystem:
    """
    Combined controller for URe5 + Hand-E gripper system
    
    This class provides a unified interface for controlling both the robot
    and gripper together for common manipulation tasks.
    """
    
    def __init__(self, robot_ip):
        """
        Initialize the complete system
        
        Args:
            robot_ip (str): IP address of the UR robot
        """
        self.robot = URe5Controller(robot_ip)
        self.gripper = HandEGripper(robot_ip)
        
    def connect(self):
        """Connect to both robot and gripper"""
        robot_ok = self.robot.connect()
        gripper_ok = self.gripper.connect()
        
        if robot_ok and gripper_ok:
            self.gripper.activate()
            print("✓ System ready!")
            return True
        else:
            print("✗ System initialization failed")
            return False
    
    def pick_object(self, pick_pose, approach_offset=[0, 0, 0.1], 
                    gripper_force=150, gripper_speed=255):
        """
        Execute a pick operation
        
        Args:
            pick_pose (list): Target pick pose [x, y, z, rx, ry, rz]
            approach_offset (list): Offset for approach position [dx, dy, dz]
            gripper_force (int): Gripping force (0-255)
            gripper_speed (int): Gripper closing speed (0-255)
            
        Returns:
            bool: True if object was successfully gripped
        """
        # Calculate approach pose
        approach_pose = pick_pose.copy()
        approach_pose[0] += approach_offset[0]
        approach_pose[1] += approach_offset[1]
        approach_pose[2] += approach_offset[2]
        
        # Move to approach position
        print("Moving to approach position...")
        self.robot.move_l(approach_pose, speed=0.25, acceleration=1.2)
        
        # Open gripper
        print("Opening gripper...")
        self.gripper.open()
        time.sleep(1.0)
        
        # Move to pick position
        print("Moving to pick position...")
        self.robot.move_l(pick_pose, speed=0.1, acceleration=0.5)
        
        # Close gripper
        print("Closing gripper...")
        self.gripper.close(speed=gripper_speed, force=gripper_force)
        time.sleep(1.5)
        
        # Check if object was gripped
        if self.gripper.is_object_detected():
            print("✓ Object gripped successfully!")
            # Lift object
            self.robot.move_l(approach_pose, speed=0.1, acceleration=0.5)
            return True
        else:
            print("✗ Failed to grip object")
            self.gripper.open()
            return False
    
    def place_object(self, place_pose, approach_offset=[0, 0, 0.1]):
        """
        Execute a place operation
        
        Args:
            place_pose (list): Target place pose [x, y, z, rx, ry, rz]
            approach_offset (list): Offset for approach position [dx, dy, dz]
            
        Returns:
            bool: True if object was placed successfully
        """
        # Calculate approach pose
        approach_pose = place_pose.copy()
        approach_pose[0] += approach_offset[0]
        approach_pose[1] += approach_offset[1]
        approach_pose[2] += approach_offset[2]
        
        # Move to approach position
        print("Moving to approach position...")
        self.robot.move_l(approach_pose, speed=0.25, acceleration=1.2)
        
        # Move to place position
        print("Moving to place position...")
        self.robot.move_l(place_pose, speed=0.1, acceleration=0.5)
        
        # Open gripper to release
        print("Releasing object...")
        self.gripper.open()
        time.sleep(1.0)
        
        # Lift away
        self.robot.move_l(approach_pose, speed=0.1, acceleration=0.5)
        
        print("✓ Object placed")
        return True
    
    def home_position(self, joint_positions=[0, -1.57, 0, -1.57, 0, 0]):
        """
        Move robot to home position
        
        Args:
            joint_positions (list): Home joint angles (default is a safe home pose)
        """
        print("Moving to home position...")
        self.robot.move_j(joint_positions, speed=1.05, acceleration=1.4)
        self.gripper.open()
        print("✓ At home position")
    
    def disconnect(self):
        """Disconnect from both robot and gripper"""
        self.gripper.disconnect()
        self.robot.disconnect()


# Example usage and test functions
def simple_gripper_test(robot_ip):
    """Simple test for gripper only"""
    print("\n=== Hand-E Gripper Test ===\n")
    
    gripper = HandEGripper(robot_ip)
    
    if not gripper.connect():
        return
    
    gripper.activate()
    
    # Open
    print("\nOpening gripper...")
    gripper.open()
    time.sleep(2)
    
    # Close slowly
    print("Closing gripper slowly...")
    gripper.close(speed=50, force=100)
    time.sleep(2)
    
    # Check status
    print(f"Position: {gripper.get_position()}")
    print(f"Object detected: {gripper.is_object_detected()}")
    
    gripper.disconnect()


def simple_robot_test(robot_ip):
    """Simple test for robot only"""
    print("\n=== URe5 Robot Test ===\n")
    
    robot = URe5Controller(robot_ip)
    
    if not robot.connect():
        return
    
    # Get current state
    print(f"Current joint positions: {robot.get_joint_positions()}")
    print(f"Current TCP pose: {robot.get_tcp_pose()}")
    
    # Move to home
    home = [0, -1.57, 0, -1.57, 0, 0]
    print(f"\nMoving to home position...")
    robot.move_j(home, speed=1.05, acceleration=1.4)
    
    robot.disconnect()


def full_pick_and_place_test(robot_ip):
    """Full system test with pick and place"""
    print("\n=== Full System Pick and Place Test ===\n")
    
    system = URHandESystem(robot_ip)
    
    if not system.connect():
        return
    
    # Define pick and place poses (adjust these for your setup!)
    pick_pose = [-0.3, -0.4, 0.15, 2.22, 2.22, 0]
    place_pose = [0.3, -0.4, 0.15, 2.22, 2.22, 0]
    
    # Go home first
    system.home_position()
    time.sleep(1)
    
    # Execute pick
    if system.pick_object(pick_pose, gripper_force=150):
        time.sleep(1)
        # Execute place
        system.place_object(place_pose)
    
    # Return home
    system.home_position()
    
    system.disconnect()


if __name__ == "__main__":
    # Change this to your robot's IP address
    ROBOT_IP = "192.168.1.191"
    
    print("URe5 + Hand-E Controller")
    print("=" * 50)
    print("\nAvailable tests:")
    print("1. Gripper only test")
    print("2. Robot only test")
    print("3. Full pick and place test")
    
    choice = input("\nEnter test number (1-3): ")
    
    if choice == "1":
        simple_gripper_test(ROBOT_IP)
    elif choice == "2":
        simple_robot_test(ROBOT_IP)
    elif choice == "3":
        full_pick_and_place_test(ROBOT_IP)
    else:
        print("Invalid choice")