"""
Robot safety validation system
Validates robot state and command data integrity

CLEANED UP VERSION:
- Removed singularity checks (UR controller handles this)
- Removed joint limit checks (IK solver handles this)
- Kept essential validations: robot state, data integrity, gripper safety
"""

import math
from core.error_handler import RobotError
from core.math_utils import is_valid_pose
from config.constants import (
    ROBOT_MODE_NAMES, SAFETY_MODE_NAMES,
    GRIPPER_MIN_POSITION, GRIPPER_MAX_POSITION, 
    GRIPPER_MIN_SPEED, GRIPPER_MAX_SPEED, 
    GRIPPER_MIN_FORCE, GRIPPER_MAX_FORCE
)


class SafetyChecker:
    """
    Validates robot state and command data integrity
    Focus: State validation and data sanity checks only
    """
    
    def __init__(self, rtde_receive=None, log_file=None):
        """
        Initialize safety checker
        
        Args:
            rtde_receive: RTDE receive interface for robot state
            log_file: Log file for error reporting
        """
        self.rtde_r = rtde_receive
        self.log_file = log_file
    
    def set_rtde_receive(self, rtde_receive):
        """Update RTDE receive interface"""
        self.rtde_r = rtde_receive
    
    def validate_robot_state(self):
        """
        Check if robot is in safe operational state
        
        Returns:
            Tuple (is_valid: bool, message: str)
        """
        if not self.rtde_r:
            return False, RobotError.format_error('E101', 
                details="RTDE not initialized")
        
        try:
            # Check robot mode
            robot_mode = self.rtde_r.getRobotMode()
            if robot_mode != 7:  # 7 = Running mode
                mode_name = ROBOT_MODE_NAMES.get(robot_mode, f"Unknown ({robot_mode})")
                return False, RobotError.format_error('E201',
                    details=f"Current mode: {mode_name}",
                    context="Robot must be in Running mode (7)")
            
            # Check safety status
            safety_status = self.rtde_r.getSafetyMode()
            if safety_status != 1:  # 1 = Normal
                safety_name = SAFETY_MODE_NAMES.get(safety_status, 
                    f"Unknown ({safety_status})")
                
                if safety_status == 3:
                    return False, RobotError.format_error('E202',
                        details=f"Safety: {safety_name}",
                        context="Clear protective stop on teach pendant")
                elif safety_status in [6, 7]:
                    return False, RobotError.format_error('E203',
                        details=f"Safety: {safety_name}",
                        context="Reset emergency stop button")
                else:
                    return False, RobotError.format_error('E205',
                        details=f"Safety: {safety_name}",
                        context="Check safety system")
            
            return True, "OK"
            
        except Exception as e:
            return False, RobotError.format_error('E101', str(e), 
                "State validation failed")
    
    def validate_pose_data(self, pose):
        """
        Validate pose data integrity (NaN/Inf checks)
        
        Args:
            pose: Target pose [x, y, z, rx, ry, rz]
        
        Returns:
            Tuple (is_valid: bool, message: str)
        """
        if not is_valid_pose(pose):
            return False, RobotError.format_error('E401', 
                details="NaN/Inf or invalid pose length",
                context=f"Pose: {pose}")
        
        return True, "OK"
    
    def validate_velocity_data(self, velocity_vector):
        """
        Validate velocity command data integrity
        
        Args:
            velocity_vector: 6D velocity [vx, vy, vz, rx, ry, rz]
        
        Returns:
            Tuple (is_valid: bool, message: str)
        """
        if len(velocity_vector) != 6:
            return False, RobotError.format_error('E305',
                details=f"Expected 6 values, got {len(velocity_vector)}",
                context=f"Velocity: {velocity_vector}")
        
        if not all(math.isfinite(v) for v in velocity_vector):
            return False, RobotError.format_error('E305',
                details="NaN or Inf in velocity command",
                context=f"Velocity: {velocity_vector}")
        
        return True, "OK"
    
    def validate_joint_velocity_data(self, joint_velocities):
        """
        Validate joint velocity command data integrity
        
        Args:
            joint_velocities: 6 joint velocities [q0, q1, q2, q3, q4, q5]
        
        Returns:
            Tuple (is_valid: bool, message: str)
        """
        if len(joint_velocities) != 6:
            return False, RobotError.format_error('E305',
                details=f"Expected 6 joint values, got {len(joint_velocities)}",
                context=f"Joint velocities: {joint_velocities}")
        
        if not all(math.isfinite(v) for v in joint_velocities):
            return False, RobotError.format_error('E305',
                details="NaN or Inf in joint velocity command",
                context=f"Joint velocities: {joint_velocities}")
        
        return True, "OK"
    
    # ========================================================================
    # GRIPPER SAFETY VALIDATIONS
    # ========================================================================
    
    def validate_gripper_position(self, position):
        """
        Validate gripper position command
        
        Args:
            position: Gripper position (0-255)
        
        Returns:
            Tuple (is_valid: bool, message: str)
        """
        if not isinstance(position, (int, float)):
            return False, RobotError.format_error('E605',
                details="Position must be numeric",
                context=f"Received: {type(position)}")
        
        if not math.isfinite(position):
            return False, RobotError.format_error('E605',
                details="Position is NaN or Inf",
                context=f"Value: {position}")
        
        if not (GRIPPER_MIN_POSITION <= position <= GRIPPER_MAX_POSITION):
            return False, RobotError.format_error('E605',
                details=f"Position out of range: {position}",
                context=f"Valid range: {GRIPPER_MIN_POSITION}-{GRIPPER_MAX_POSITION}")
        
        return True, "OK"
    
    def validate_gripper_speed(self, speed):
        """
        Validate gripper speed parameter
        
        Args:
            speed: Gripper speed (0-255)
        
        Returns:
            Tuple (is_valid: bool, message: str)
        """
        if not isinstance(speed, (int, float)):
            return False, RobotError.format_error('E605',
                details="Speed must be numeric",
                context=f"Received: {type(speed)}")
        
        if not math.isfinite(speed):
            return False, RobotError.format_error('E605',
                details="Speed is NaN or Inf",
                context=f"Value: {speed}")
        
        if not (GRIPPER_MIN_SPEED <= speed <= GRIPPER_MAX_SPEED):
            return False, RobotError.format_error('E605',
                details=f"Speed out of range: {speed}",
                context=f"Valid range: {GRIPPER_MIN_SPEED}-{GRIPPER_MAX_SPEED}")
        
        return True, "OK"
    
    def validate_gripper_force(self, force):
        """
        Validate gripper force parameter
        
        Args:
            force: Gripper force (0-255)
        
        Returns:
            Tuple (is_valid: bool, message: str)
        """
        if not isinstance(force, (int, float)):
            return False, RobotError.format_error('E605',
                details="Force must be numeric",
                context=f"Received: {type(force)}")
        
        if not math.isfinite(force):
            return False, RobotError.format_error('E605',
                details="Force is NaN or Inf",
                context=f"Value: {force}")
        
        if not (GRIPPER_MIN_FORCE <= force <= GRIPPER_MAX_FORCE):
            return False, RobotError.format_error('E605',
                details=f"Force out of range: {force}",
                context=f"Valid range: {GRIPPER_MIN_FORCE}-{GRIPPER_MAX_FORCE}")
        
        return True, "OK"
    
    def validate_gripper_state(self, gripper_controller):
        """
        Check if gripper is ready to accept commands
        
        Args:
            gripper_controller: GripperController instance
        
        Returns:
            Tuple (is_ready: bool, message: str)
        """
        if not gripper_controller:
            return False, "Gripper controller not initialized"
        
        if not gripper_controller.is_connected():
            return False, RobotError.format_error('E601',
                details="Gripper not connected",
                context="Check network connection")
        
        if not gripper_controller.is_activated():
            return False, RobotError.format_error('E602',
                details="Gripper not activated",
                context="Activation sequence required")
        
        return True, "OK"