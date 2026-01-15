"""
Control Mode Dispatcher
Handles logic for different robot control modes
"""

import numpy as np
from config.constants import CONTROL_MODES, MOVEMENT_DEADZONE, DEADZONE_RAMP_WIDTH, VELOCITY_DECAY, IMU_AXIS_MAPPING
from robot.movement_modes import MovementModes
from core.math_utils import apply_deadzone_ramp, apply_axis_mapping, quaternion_difference, quat_to_euler, rotation_vector_add

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
        
        # Momentum state
        self.current_velocity = np.array([0.0, 0.0, 0.0]) # [x, y, z]
        self.current_angular_velocity = np.array([0.0, 0.0, 0.0]) # [rx, ry, rz]
        self.last_mode = 0
        
        # Mimic mode state
        self.mimic_ref_hand_quat = None
        self.mimic_ref_robot_pose = None
        
    def reset_mimic_references(self):
        """Force reset of mimic mode references (call after calibration)"""
        self.mimic_ref_hand_quat = None
        self.mimic_ref_robot_pose = None
        print("Mimic references reset.")
        
        # Input Hysteresis (Command Suppression)
        self.last_input_angles = None
        self.INPUT_THRESHOLD = 3.0 # degrees

        # Flex Sensor (Gripper) state
        self.last_flex_value = None
        self.FLEX_THRESHOLD = 5 # 0-255 scale
    
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
        # Reset momentum and mimic state on mode switch
        if mode != self.last_mode:
            self.current_velocity = np.array([0.0, 0.0, 0.0])
            self.current_angular_velocity = np.array([0.0, 0.0, 0.0])
            self.mimic_ref_hand_quat = None
            self.mimic_ref_robot_pose = None
            self.last_mode = mode
            
        # 1. Process Flex Sensor (Gripper) - works in all modes
        flex_value = imu_data.get('flex', None)
        if flex_value is not None:
            if self.last_flex_value is None or abs(flex_value - self.last_flex_value) >= self.FLEX_THRESHOLD:
                # Perform gripper move command
                # Note: Robotiq .move(pos, speed, force)
                self.rtde_controller.move_gripper(flex_value)
                self.last_flex_value = flex_value

        if mode == 0:
            return
            
        # Extract Euler angles (already calibrated)
        rel_roll, rel_pitch, rel_yaw = imu_data['euler']
        
        # Apply smoothing via controller
        smooth_roll, smooth_pitch, smooth_yaw = self.rtde_controller.apply_smoothing(
            rel_roll, rel_pitch, rel_yaw)
            
        # Apply deadzone filtering
        filtered_roll = apply_deadzone_ramp(smooth_roll, MOVEMENT_DEADZONE, DEADZONE_RAMP_WIDTH)
        filtered_pitch = apply_deadzone_ramp(smooth_pitch, MOVEMENT_DEADZONE, DEADZONE_RAMP_WIDTH)
        filtered_yaw = apply_deadzone_ramp(smooth_yaw, MOVEMENT_DEADZONE, DEADZONE_RAMP_WIDTH)
        
        filtered_angles = np.array([filtered_roll, filtered_pitch, filtered_yaw])
        
        # Apply Input Hysteresis (3-degree threshold for updates)
        is_returning_to_zero = np.all(filtered_angles == 0)
        was_at_zero = self.last_input_angles is not None and np.all(self.last_input_angles == 0)
        
        if self.last_input_angles is not None and not (is_returning_to_zero and not was_at_zero):
            # Calculate max angular change across all 3 axes
            max_delta = np.max(np.abs(filtered_angles - self.last_input_angles))
            if max_delta < self.INPUT_THRESHOLD:
                # Still process data for internal state, but skip sending fresh robot commands
                # (Note: Mode 6 is absolute and handled separately below, we focus on 1-5 here)
                if mode != 6:
                    return

        self.last_input_angles = filtered_angles.copy()
        
        # Apply axis mapping (configurable in constants.py)
        mapped = apply_axis_mapping(filtered_roll, filtered_pitch, filtered_yaw, IMU_AXIS_MAPPING)
        
        # Calculate velocity scale (use filtered values to avoid scaling up noise)
        velocity_scale = self.rtde_controller.calculate_velocity_scale(
            filtered_roll, filtered_pitch, filtered_yaw)
        
        try:
            # Initialize accelerations (deltas from input)
            accel_pos = [0.0, 0.0, 0.0]
            accel_rot = [0.0, 0.0, 0.0]
            
            if mode == 1:
                # BASE_FRAME_XY - uses mapped x and y
                accel_pos, accel_rot = self.movement_modes.calculate_base_frame_xy(
                    mapped['x'], mapped['y'], runtime_config.LINEAR_SPEED_SCALE)
                
            elif mode == 2:
                # VERTICAL_Z - uses mapped z
                accel_pos, accel_rot = self.movement_modes.calculate_vertical_z(
                    mapped['z'], runtime_config.LINEAR_SPEED_SCALE)
                
            elif mode == 3:
                # BASE_FRAME_ORIENT - uses mapped rx, ry, rz
                accel_pos, accel_rot = self.movement_modes.calculate_base_frame_orientation(
                    mapped['rx'], mapped['ry'], mapped['rz'], 
                    runtime_config.ANGULAR_SPEED_SCALE)
                
            elif mode == 4:
                # TCP_XY - uses mapped x and y
                accel_pos, accel_rot = self.movement_modes.calculate_tcp_xy(
                    mapped['x'], mapped['y'], runtime_config.LINEAR_SPEED_SCALE)
                
            elif mode == 5:
                # TCP_Z - uses mapped z
                accel_pos, accel_rot = self.movement_modes.calculate_tcp_z(
                    mapped['z'], runtime_config.LINEAR_SPEED_SCALE)
                
            elif mode == 6:
                # TCP_ORIENT - Absolute control (Mimic)
                # Apply axis mapping to the absolute hand orientation
                hand_quat = imu_data['quaternion']
                mapped_hand_quat = apply_axis_mapping_quat(hand_quat, IMU_AXIS_MAPPING)

                # 1. Capture references on first run
                if self.mimic_ref_hand_quat is None:
                    self.mimic_ref_hand_quat = mapped_hand_quat
                    self.mimic_ref_robot_pose = self.rtde_controller.current_tcp_pose[:]
                    print(f"Mimic Mode Initialized. Ref Pose: {self.mimic_ref_robot_pose[:3]}")
                    return

                # 2. Calculate Hand Delta (Current - Ref) in the mapped frame
                # Formula: q_delta = mapped_current * inverse(mapped_ref)
                delta_quat = quaternion_difference(self.mimic_ref_hand_quat, mapped_hand_quat)
                
                # 3. Convert delta to Rotation Vector
                # This vector is what we'll add to the robot's reference orientation
                mapped_delta_vec = quaternion_to_rotation_vector(delta_quat)
                
                # 4. Apply to Robot Reference Orientation
                ref_orientation = self.mimic_ref_robot_pose[3:6]
                target_orientation = rotation_vector_add(ref_orientation, mapped_delta_vec)
                
                # 5. Construct Target Pose (Keep Ref Position)
                target_pose = self.mimic_ref_robot_pose[:3] + target_orientation
                
                # 6. Execute servoL
                self.rtde_controller.move_servo(
                    target_pose, 
                    velocity=0.5 * runtime_config.ANGULAR_SPEED_SCALE,
                    acceleration=0.5,
                    lookahead_time=0.05,
                    gain=500)
                return # Exit early for Mode 6
            
            # Apply Momentum / Input Decay for Modes 1-5
            # 1. Add input acceleration to current velocity
            # Scaling down acceleration slightly to make it controllable with accumulation
            ACCEL_FACTOR = 0.5 
            self.current_velocity += np.array(accel_pos) * ACCEL_FACTOR
            self.current_angular_velocity += np.array(accel_rot) * ACCEL_FACTOR
            
            # 2. Apply Decay (Friction)
            self.current_velocity *= VELOCITY_DECAY
            self.current_angular_velocity *= VELOCITY_DECAY
            
            # 3. Use accumulated velocity as the command
            # Threshold to stop completely if very slow
            is_moving = np.any(self.current_velocity > 0.0001) or np.any(self.current_angular_velocity > 0.0001)
            
            if not is_moving:
                # If we were moving, send one final zero command to ensure stop
                was_moving = self.rtde_controller.last_sent_velocity is not None and any(v != 0 for v in self.rtde_controller.last_sent_velocity)
                if was_moving:
                    self.rtde_controller.move_speed([0.0]*3, [0.0]*3, acceleration=1.0)
                
                self.current_velocity = np.array([0.0, 0.0, 0.0])
                self.current_angular_velocity = np.array([0.0, 0.0, 0.0])
                return

            # Execute move if there is significant velocity
            # Use speedL for smooth velocity control
            self.rtde_controller.move_speed(
                self.current_velocity.tolist(), 
                self.current_angular_velocity.tolist(), 
                acceleration=0.5
            )
                
        except Exception as e:
            print(f"Error executing mode {mode}: {e}")
