"""
Control Mode Dispatcher
Handles logic for different robot control modes

FIXES:
- Removed momentum/velocity accumulation causing lag
- Direct pass-through of scaled input values
- No artificial decay or smoothing in dispatcher
- Fixes thread issues by using direct speedL commands
"""

import numpy as np
from config.constants import CONTROL_MODES, MOVEMENT_DEADZONE, DEADZONE_RAMP_WIDTH
from robot.movement_modes import MovementModes
from core.math_utils import (
    apply_deadzone_ramp,
    quaternion_difference,
    quaternion_to_rotation_vector,
    rotation_vector_add
)
from core.coordinate_frames import frame_transform

class ControlModeDispatcher:
    """
    Dispatches control commands based on active mode
    """
    
    def __init__(self, rtde_controller):
        """
        Initialize dispatcher
        
        Args:
            rtde_controller: RTDEController instance
        """
        self.rtde_controller = rtde_controller
        self.movement_modes = MovementModes()
        
        # Track state for display only (not for control)
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.current_angular_velocity = np.array([0.0, 0.0, 0.0])
        self.last_mode = 0
        
        # Mimic mode state
        self.mimic_ref_hand_quat = None
        self.mimic_ref_robot_pose = None
        
        # REMOVED: Input hysteresis (was causing lag)
        # Instead we rely on RTDE's built-in command suppression
    
    def reset_mimic_references(self):
        """Force reset of mimic mode references (call after calibration)"""
        self.mimic_ref_hand_quat = None
        self.mimic_ref_robot_pose = None
        print("Mimic references reset.")
    
    def get_mode_name(self, mode_id):
        """Get human-readable name for mode"""
        return CONTROL_MODES.get(mode_id, f"UNKNOWN_{mode_id}")
    
    def execute_mode(self, mode, imu_data, runtime_config):
        """
        Execute control logic for the current mode
        
        Args:
            mode: Current mode ID
            imu_data: Dictionary with parsed IMU data
            runtime_config: RuntimeConfig instance
        """
        # Reset mimic state on mode switch
        if mode != self.last_mode:
            self.mimic_ref_hand_quat = None
            self.mimic_ref_robot_pose = None
            self.last_mode = mode
        
        if mode == 0:
            # IDLE - send stop command
            self.rtde_controller.move_speed([0.0]*3, [0.0]*3, acceleration=1.0, mode_name="IDLE")
            self.current_velocity = np.array([0.0, 0.0, 0.0])
            self.current_angular_velocity = np.array([0.0, 0.0, 0.0])
            return
        
        # Extract Euler angles (already calibrated)
        rel_roll, rel_pitch, rel_yaw = imu_data['euler']
        
        # Apply smoothing via controller (now linear averaging)
        smooth_roll, smooth_pitch, smooth_yaw = self.rtde_controller.apply_smoothing(
            rel_roll, rel_pitch, rel_yaw)
        
        # Apply deadzone filtering
        filtered_roll = apply_deadzone_ramp(smooth_roll, MOVEMENT_DEADZONE, DEADZONE_RAMP_WIDTH)
        filtered_pitch = apply_deadzone_ramp(smooth_pitch, MOVEMENT_DEADZONE, DEADZONE_RAMP_WIDTH)
        filtered_yaw = apply_deadzone_ramp(smooth_yaw, MOVEMENT_DEADZONE, DEADZONE_RAMP_WIDTH)
        
        # Convert to Standard IMU frame, then apply target mapping
        imu_euler = {
            'roll': filtered_roll,
            'pitch': filtered_pitch,
            'yaw': filtered_yaw
        }
        
        # Get robot-frame deltas using coordinate transform
        robot_trans = frame_transform.to_robot_translation(imu_euler)
        robot_rot = frame_transform.to_robot_rotation(imu_euler)
        
        # Calculate velocity scale (use filtered values)
        velocity_scale = self.rtde_controller.calculate_velocity_scale(
            filtered_roll, filtered_pitch, filtered_yaw)
        
        # Get mode name for logging
        mode_name = self.get_mode_name(mode)
        
        try:
            # CHANGED: Direct velocity calculation (no momentum accumulation)
            velocity_pos = [0.0, 0.0, 0.0]
            velocity_rot = [0.0, 0.0, 0.0]
            
            if mode == 1:
                # BASE_FRAME_XY
                velocity_pos, velocity_rot = self.movement_modes.calculate_base_frame_xy(
                    robot_trans['x'], robot_trans['y'], runtime_config.LINEAR_SPEED_SCALE)
            
            elif mode == 2:
                # VERTICAL_Z
                velocity_pos, velocity_rot = self.movement_modes.calculate_vertical_z(
                    robot_trans['z'], runtime_config.LINEAR_SPEED_SCALE)
            
            elif mode == 3:
                # BASE_FRAME_ORIENT
                velocity_pos, velocity_rot = self.movement_modes.calculate_base_frame_orientation(
                    robot_rot['rx'], robot_rot['ry'], robot_rot['rz'],
                    runtime_config.ANGULAR_SPEED_SCALE)
            
            elif mode == 4:
                # TCP_XY
                velocity_pos, velocity_rot = self.movement_modes.calculate_tcp_xy(
                    robot_trans['x'], robot_trans['y'], runtime_config.LINEAR_SPEED_SCALE)
            
            elif mode == 5:
                # TCP_Z
                velocity_pos, velocity_rot = self.movement_modes.calculate_tcp_z(
                    robot_trans['z'], runtime_config.LINEAR_SPEED_SCALE)
            
            elif mode == 6:
                # TCP_ORIENT - Absolute control (Mimic)
                hand_quat = imu_data['quaternion']
                
                # 1. Capture references on first run
                if self.mimic_ref_hand_quat is None:
                    self.mimic_ref_hand_quat = hand_quat
                    self.mimic_ref_robot_pose = self.rtde_controller.current_tcp_pose[:]
                    print(f"Mimic Mode Initialized. Ref Pose: {self.mimic_ref_robot_pose[:3]}")
                    return
                
                # 2. Calculate Hand Delta (Current - Ref)
                delta_quat = quaternion_difference(self.mimic_ref_hand_quat, hand_quat)
                
                # 3. Convert delta to Rotation Vector
                delta_vec = quaternion_to_rotation_vector(delta_quat)
                
                # 4. Apply to Robot Reference Orientation
                ref_orientation = self.mimic_ref_robot_pose[3:6]
                target_orientation = rotation_vector_add(ref_orientation, delta_vec)
                
                # 5. Construct Target Pose (Keep Ref Position)
                target_pose = self.mimic_ref_robot_pose[:3] + target_orientation
                
                # 6. Execute servoL
                self.rtde_controller.move_servo(
                    target_pose,
                    velocity=0.5 * runtime_config.ANGULAR_SPEED_SCALE,
                    acceleration=0.5,
                    lookahead_time=0.05,
                    gain=500,
                    mode_name=mode_name)
                return
            
            # CHANGED: Direct pass-through (no momentum, no decay)
            # Update display velocities
            self.current_velocity = np.array(velocity_pos)
            self.current_angular_velocity = np.array(velocity_rot)
            
            # Check if there's any movement
            is_moving = np.any(np.abs(self.current_velocity) > 0.0001) or \
                       np.any(np.abs(self.current_angular_velocity) > 0.0001)
            
            if not is_moving:
                # Send stop command
                self.rtde_controller.move_speed([0.0]*3, [0.0]*3, acceleration=1.0, mode_name=mode_name)
                return
            
            # CHANGED: Direct velocity command with higher acceleration for responsiveness
            self.rtde_controller.move_speed(
                self.current_velocity.tolist(),
                self.current_angular_velocity.tolist(),
                acceleration=1.5,  # CHANGED: Increased from 0.5 to 1.5 for faster response
                mode_name=mode_name
            )
        
        except Exception as e:
            print(f"Error executing mode {mode}: {e}")