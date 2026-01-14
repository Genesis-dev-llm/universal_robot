"""
Robot safety validation system
All safety checks for robot operations
"""

import math
from core.error_handler import RobotError
from core.math_utils import is_valid_pose
from config.constants import (
    UR_JOINT_LIMITS, JOINT_SAFETY_MARGIN,
    WRIST_SINGULARITY_THRESHOLD, SHOULDER_SINGULARITY_THRESHOLD,
    ELBOW_SINGULARITY_THRESHOLD, ROBOT_MODE_NAMES, SAFETY_MODE_NAMES
)

class SafetyChecker:
    """
    Validates robot state and motion safety
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