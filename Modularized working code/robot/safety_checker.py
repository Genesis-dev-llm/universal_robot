"""
Robot safety validation system
All safety checks for robot operations

UPDATED: Added gripper safety validations
"""

import math
from core.error_handler import RobotError
from core.math_utils import is_valid_pose
from config.constants import (
    UR_JOINT_LIMITS, JOINT_SAFETY_MARGIN,
    WRIST_SINGULARITY_THRESHOLD, SHOULDER_SINGULARITY_THRESHOLD,
    ELBOW_SINGULARITY_THRESHOLD, ROBOT_MODE_NAMES, SAFETY_MODE_NAMES,
    GRIPPER_MIN_POSITION, GRIPPER_MAX_POSITION, GRIPPER_MIN_SPEED,
    GRIPPER_MAX_SPEED, GRIPPER_MIN_FORCE, GRIPPER_MAX_FORCE
)

class SafetyChecker:
    """
    Validates robot state and motion safety
    Now includes gripper safety checks
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
        Check if robot is in safe state to accept commands
        
        Returns:
            Tuple (is_valid: bool, message: str)
        """
        if not self.rtde_r:
            return False, RobotError.format_error('E101', details="RTDE not initialized")
        
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
                safety_name = SAFETY_MODE_NAMES.get(safety_status, f"Unknown ({safety_status})")
                
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
            return False, RobotError.format_error('E101', str(e), "State validation failed")
    
    def validate_pose_data(self, pose):
        """
        Validate pose data before sending to robot
        
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
    
    def check_joint_limits(self, target_pose):
        """
        Check if target pose would violate joint limits
        
        Args:
            target_pose: Target TCP pose [x, y, z, rx, ry, rz]
        
        Returns:
            Tuple (is_safe: bool, message: str)
        """
        if not self.rtde_r:
            return True, "No robot connection"
        
        try:
            current_joints = self.rtde_r.getActualQ()
            if not current_joints or len(current_joints) != 6:
                return True, "Could not read joints"
            
            # Use inverse kinematics to predict joint positions
            predicted_joints = self.rtde_r.getInverseKinematics(target_pose, current_joints)
            
            if not predicted_joints or len(predicted_joints) != 6:
                return False, RobotError.format_error('E304', 
                    details="IK solution not found",
                    context="Pose may be unreachable")
            
            # Check each joint
            joint_names = ['Base', 'Shoulder', 'Elbow', 'Wrist1', 'Wrist2', 'Wrist3']
            limit_list = [
                UR_JOINT_LIMITS['joint_0'], UR_JOINT_LIMITS['joint_1'],
                UR_JOINT_LIMITS['joint_2'], UR_JOINT_LIMITS['joint_3'],
                UR_JOINT_LIMITS['joint_4'], UR_JOINT_LIMITS['joint_5']
            ]
            
            for i, (joint_val, (min_val, max_val)) in enumerate(zip(predicted_joints, limit_list)):
                if joint_val < (min_val + JOINT_SAFETY_MARGIN) or joint_val > (max_val - JOINT_SAFETY_MARGIN):
                    return False, RobotError.format_error('E301',
                        details=f"{joint_names[i]} would exceed limit",
                        context=f"Predicted: {math.degrees(joint_val):.1f}°, Range: [{math.degrees(min_val):.1f}°, {math.degrees(max_val):.1f}°]")
            
            return True, "OK"
            
        except Exception as e:
            if self.log_file:
                RobotError.log_error(self.log_file, 'E301', str(e), "Joint limit check error")
            return False, f"Joint limit check error: {str(e)}"
    
    def check_singularity_proximity(self, target_pose=None):
        """
        Check for kinematic singularities
        
        Args:
            target_pose: Optional target pose to check, or None for current position
        
        Returns:
            Tuple (is_safe: bool, message: str)
        """
        if not self.rtde_r:
            return True, "No robot connection"
        
        try:
            if target_pose:
                current_joints = self.rtde_r.getActualQ()
                joint_positions = self.rtde_r.getInverseKinematics(target_pose, current_joints)
                if not joint_positions:
                    return False, RobotError.format_error('E302',
                        details="IK failed",
                        context="Pose may be singular")
            else:
                joint_positions = self.rtde_r.getActualQ()
                if not joint_positions:
                    return True, "Could not read joints"
            
            # Wrist singularity (J4 near zero)
            if abs(joint_positions[4]) < WRIST_SINGULARITY_THRESHOLD:
                return False, RobotError.format_error('E302',
                    details=f"Wrist singularity: J4={math.degrees(joint_positions[4]):.1f}°",
                    context="Wrist axes aligned")
            
            # Shoulder singularity (TCP above/below base)
            shoulder_elbow_sum = abs(joint_positions[1] + joint_positions[2])
            if shoulder_elbow_sum < SHOULDER_SINGULARITY_THRESHOLD:
                return False, RobotError.format_error('E302',
                    details=f"Shoulder singularity: J1+J2={math.degrees(shoulder_elbow_sum):.1f}°",
                    context="TCP near base axis")
            
            # Elbow singularity (arm fully extended/retracted)
            if abs(joint_positions[2]) < ELBOW_SINGULARITY_THRESHOLD:
                return False, RobotError.format_error('E302',
                    details=f"Elbow singularity: J2={math.degrees(joint_positions[2]):.1f}°",
                    context="Arm near full extension")
            
            if abs(abs(joint_positions[2]) - math.pi) < ELBOW_SINGULARITY_THRESHOLD:
                return False, RobotError.format_error('E302',
                    details=f"Elbow singularity: J2={math.degrees(joint_positions[2]):.1f}°",
                    context="Arm near full retraction")
            
            return True, "OK"
            
        except Exception as e:
            if self.log_file:
                RobotError.log_error(self.log_file, 'E302', str(e), "Singularity check error")
            return False, f"Singularity check error: {str(e)}"
    
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