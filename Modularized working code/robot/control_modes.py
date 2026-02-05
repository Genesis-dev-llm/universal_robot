"""
Control Mode Dispatcher - Mode 1 Priority System
Handles logic for different robot control modes

MODE 1 PRIORITY SYSTEM:
- Compares abs(x_delta) vs abs(y_delta)
- Dominant input wins:
  * If left/right tilt stronger: Use speedJ for base rotation
  * If forward/back tilt stronger: Use speedL for TCP translation
- 5° deadzone prevents accidental dual triggering

GRIPPER INTEGRATION - UPDATED:
- Modes 1-6: Continuous flex tracking (respects LOCK)
- Mode 7: Gripper-only control (BYPASSES lock)
- Universal gripper: Now works in ALL modes

MODE 3 UPDATE:
- Now supports full XY plane control (both forward/back and left/right)
- Uses base frame directly (no TCP transformation)
"""

import numpy as np
from config.constants import CONTROL_MODES, MOVEMENT_DEADZONE, DEADZONE_RAMP_WIDTH
from robot.movement_modes import MovementModes
from core.math_utils import (
    apply_deadzone_ramp,
    quaternion_difference,
    quaternion_to_rotation_vector,
    rotation_vector_add,
    transform_tcp_velocity_to_base
)
from core.coordinate_frames import frame_transform

class ControlModeDispatcher:
    """
    Dispatches control commands based on active mode
    Handles frame transformations, RTDE commands, and gripper control
    
    UPDATED: Universal gripper support for all modes
    """
    
    def __init__(self, rtde_controller):
        """
        Initialize dispatcher
        
        Args:
            rtde_controller: RTDEController instance (includes gripper)
        """
        self.rtde_controller = rtde_controller
        self.movement_modes = MovementModes()
        
        # Track state for display only (not for control)
        self.current_velocity = np.array([0.0, 0.0, 0.0])
        self.current_angular_velocity = np.array([0.0, 0.0, 0.0])
        self.last_mode = -1  # Changed to -1 to ensure first mode prints
        
        # Mimic mode state (Mode 6)
        self.mimic_ref_hand_quat = None
        self.mimic_ref_robot_pose = None
    
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
            imu_data: Dictionary with parsed IMU data (includes 'flex' key)
            runtime_config: RuntimeConfig instance
        """
        # CRITICAL: Mode change detection at TOP to prevent spam
        if mode != self.last_mode:
            # Only print/log on actual mode change
            mode_name = self.get_mode_name(mode)
            if mode != 0:  # Don't spam IDLE messages
                print(f"Mode: {mode_name}")
            self.rtde_controller.log_command(f"# Mode: {mode_name}")
            
            # Reset mimic state on mode switch
            self.mimic_ref_hand_quat = None
            self.mimic_ref_robot_pose = None
            
            self.last_mode = mode
        
        # ====================================================================
        # UNIVERSAL GRIPPER CONTROL LOGIC - UPDATED
        # ====================================================================
        flex_percent = imu_data.get('flex', 0)  # Get flex sensor value (0-100)
        
        # Determine if gripper should track flex based on mode
        gripper_enabled = runtime_config.GRIPPER_ENABLED
        gripper_locked = runtime_config.GRIPPER_LOCKED
        gripper = self.rtde_controller.gripper
        
        if gripper and gripper.is_activated() and gripper_enabled:
            # Mode 7: Gripper-only control - BYPASS LOCK
            if mode == 7:
                gripper.update_position(flex_percent)
            
            # Modes 1-6: Universal gripper - RESPECT LOCK
            elif mode in [1, 2, 3, 4, 5, 6]:
                if not gripper_locked:
                    gripper.update_position(flex_percent)
                # If locked, gripper holds position (no update)
        
        # ====================================================================
        # ROBOT CONTROL LOGIC
        # ====================================================================
        
        # Handle IDLE mode
        if mode == 0:
            self.rtde_controller.move_speed([0.0]*3, [0.0]*3, acceleration=1.0, mode_name="IDLE")
            self.current_velocity = np.array([0.0, 0.0, 0.0])
            self.current_angular_velocity = np.array([0.0, 0.0, 0.0])
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
        
        # Convert to Standard IMU frame, then apply target mapping
        imu_euler = {
            'roll': filtered_roll,
            'pitch': filtered_pitch,
            'yaw': filtered_yaw
        }
        
        # Get robot-frame deltas using coordinate transform
        robot_trans = frame_transform.to_robot_translation(imu_euler)
        robot_rot = frame_transform.to_robot_rotation(imu_euler)
        
        # Get current TCP orientation for frame transformations (Mode 1 only now)
        current_tcp_orientation = [0.0, 0.0, 0.0]
        if mode == 1 and self.rtde_controller.rtde_r:
            try:
                current_pose = self.rtde_controller.rtde_r.getActualTCPPose()
                if current_pose:
                    current_tcp_orientation = current_pose[3:6]
            except:
                pass  # Use zero orientation if read fails
        
        # Get mode name for logging (file only, not terminal)
        mode_name = self.get_mode_name(mode)
        
        try:
            # Initialize velocity vectors
            velocity_pos = [0.0, 0.0, 0.0]
            velocity_rot = [0.0, 0.0, 0.0]
            
            # ================================================================
            # MODE 1: CRANE MODE - PRIORITY SYSTEM
            # ================================================================
            if mode == 1:
                # Calculate both movements
                tcp_linear_vel, base_angular_vel = self.movement_modes.calculate_crane_mode(
                    robot_trans['x'],  # Forward/back
                    robot_trans['y'],  # Left/right
                    runtime_config.LINEAR_SPEED_SCALE,
                    runtime_config.ANGULAR_SPEED_SCALE
                )
                
                # PRIORITY SYSTEM: Compare absolute values
                abs_x = abs(robot_trans['x'])  # Forward/back magnitude
                abs_y = abs(robot_trans['y'])  # Left/right magnitude
                
                # Decide which command to send based on dominant input
                if abs_y > abs_x:
                    # LEFT/RIGHT DOMINANT: Use speedJ for base rotation
                    joint_velocities = self.movement_modes.calculate_base_rotation_joint_velocity(
                        robot_trans['y'],
                        runtime_config.ANGULAR_SPEED_SCALE
                    )
                    
                    # Update display velocities
                    self.current_velocity = np.array([0.0, 0.0, 0.0])
                    self.current_angular_velocity = np.array([0.0, 0.0, joint_velocities[0]])
                    
                    # Check if there's movement
                    is_moving = abs(joint_velocities[0]) > 0.0001
                    
                    if not is_moving:
                        self.rtde_controller.move_speed_joint([0.0]*6, acceleration=2.0, mode_name=mode_name)
                        return
                    
                    # Send speedJ for base rotation
                    self.rtde_controller.move_speed_joint(
                        joint_velocities,
                        acceleration=2.0,
                        mode_name=mode_name
                    )
                    return
                
                else:
                    # FORWARD/BACK DOMINANT: Use speedL for TCP translation
                    # Transform TCP-frame linear velocity to base frame
                    base_linear_vel = transform_tcp_velocity_to_base(
                        tcp_linear_vel,
                        current_tcp_orientation
                    )
                    
                    velocity_pos = base_linear_vel
                    velocity_rot = [0.0, 0.0, 0.0]  # No rotation when translating
            
            # ================================================================
            # MODE 2: VERTICAL Z
            # ================================================================
            elif mode == 2:
                velocity_pos, velocity_rot = self.movement_modes.calculate_vertical_z(
                    robot_trans['z'],
                    runtime_config.LINEAR_SPEED_SCALE
                )
            
            # ================================================================
            # MODE 3: BASE XY PLANE CONTROL - UPDATED
            # ================================================================
            elif mode == 3:
                # Get base-frame XY velocity (no transformation needed)
                velocity_pos, velocity_rot = self.movement_modes.calculate_lateral_precise(
                    robot_trans['x'],  # Forward/back → Base Y
                    robot_trans['y'],  # Left/right → Base X
                    runtime_config.LINEAR_SPEED_SCALE
                )
                # velocity_pos is already in base frame [vx, vy, 0]
                # No TCP transformation needed!
            
            # ================================================================
            # MODE 4: WRIST ORIENTATION (Wrist 1 & 2) - CORRECTED
            # ================================================================
            elif mode == 4:
                joint_velocities = self.movement_modes.calculate_wrist_joint_velocities(
                    robot_rot['rz'],   # Nod forward/back → Wrist 2
                    robot_rot['rx'],  # Tilt left/right → Wrist 1
                    runtime_config.ANGULAR_SPEED_SCALE
                )
                
                # Update display velocities (position locked in this mode)
                self.current_velocity = np.array([0.0, 0.0, 0.0])
                self.current_angular_velocity = np.array([
                    joint_velocities[3],  # Wrist 1
                    joint_velocities[4],  # Wrist 2
                    0.0
                ])
                
                # Check if there's any movement
                is_moving = any(abs(v) > 0.0001 for v in joint_velocities)
                
                if not is_moving:
                    self.rtde_controller.move_speed_joint([0.0]*6, acceleration=2.0, mode_name=mode_name)
                    return
                
                # Use speedJ for direct wrist control
                self.rtde_controller.move_speed_joint(
                    joint_velocities,
                    acceleration=2.0,
                    mode_name=mode_name
                )
                return
            
            # ================================================================
            # MODE 5: WRIST SCREW (Wrist 3) - CORRECTED
            # ================================================================
            elif mode == 5:
                joint_velocities = self.movement_modes.calculate_wrist3_joint_velocity(
                    robot_rot['rx'],  # Left/right tilt → Wrist 3
                    runtime_config.ANGULAR_SPEED_SCALE
                )
                
                # Update display velocities (position locked in this mode)
                self.current_velocity = np.array([0.0, 0.0, 0.0])
                self.current_angular_velocity = np.array([
                    0.0,
                    0.0,
                    joint_velocities[5]  # Wrist 3
                ])
                
                # Check if there's any movement
                is_moving = any(abs(v) > 0.0001 for v in joint_velocities)
                
                if not is_moving:
                    self.rtde_controller.move_speed_joint([0.0]*6, acceleration=2.0, mode_name=mode_name)
                    return
                
                # Use speedJ for direct wrist control
                self.rtde_controller.move_speed_joint(
                    joint_velocities,
                    acceleration=2.0,
                    mode_name=mode_name
                )
                return
            
            # ================================================================
            # MODE 6: TCP ORIENTATION MIMIC
            # ================================================================
            elif mode == 6:
                # Absolute orientation control using servoL
                hand_quat = imu_data['quaternion']
                
                # Capture references on first run
                if self.mimic_ref_hand_quat is None:
                    self.mimic_ref_hand_quat = hand_quat
                    self.mimic_ref_robot_pose = self.rtde_controller.current_tcp_pose[:]
                    print(f"Mimic Mode Initialized. Ref Pose: {self.mimic_ref_robot_pose[:3]}")
                    return
                
                # Calculate hand delta (Current - Ref)
                delta_quat = quaternion_difference(self.mimic_ref_hand_quat, hand_quat)
                
                # Convert delta to rotation vector
                delta_vec = quaternion_to_rotation_vector(delta_quat)
                
                # Apply to robot reference orientation
                ref_orientation = self.mimic_ref_robot_pose[3:6]
                target_orientation = rotation_vector_add(ref_orientation, delta_vec)
                
                # Construct target pose (keep ref position)
                target_pose = self.mimic_ref_robot_pose[:3] + target_orientation
                
                # Execute servoL
                self.rtde_controller.move_servo(
                    target_pose,
                    velocity=0.5 * runtime_config.ANGULAR_SPEED_SCALE,
                    acceleration=0.5,
                    lookahead_time=0.05,
                    gain=500,
                    mode_name=mode_name
                )
                return
            
            # ================================================================
            # MODE 7: GRIPPER ONLY
            # ================================================================
            elif mode == 7:
                # Gripper control handled at top of function (bypasses lock)
                # Keep robot stationary
                self.rtde_controller.move_speed(
                    [0.0]*3, [0.0]*3, 
                    acceleration=1.0, 
                    mode_name=mode_name
                )
                
                # Update display velocities (robot stationary)
                self.current_velocity = np.array([0.0, 0.0, 0.0])
                self.current_angular_velocity = np.array([0.0, 0.0, 0.0])
                return
            
            # ================================================================
            # SEND VELOCITY COMMAND (Modes 1-3 speedL path)
            # ================================================================
            
            # Update display velocities
            self.current_velocity = np.array(velocity_pos)
            self.current_angular_velocity = np.array(velocity_rot)
            
            # Check if there's any movement
            is_moving = np.any(np.abs(self.current_velocity) > 0.0001) or \
                       np.any(np.abs(self.current_angular_velocity) > 0.0001)
            
            if not is_moving:
                # Send stop command
                self.rtde_controller.move_speed(
                    [0.0]*3, [0.0]*3, 
                    acceleration=1.0, 
                    mode_name=mode_name
                )
                return
            
            # Send velocity command with appropriate acceleration
            self.rtde_controller.move_speed(
                self.current_velocity.tolist(),
                self.current_angular_velocity.tolist(),
                acceleration=1.5,
                mode_name=mode_name
            )
        
        except Exception as e:
            print(f"Error executing mode {mode}: {e}")
            import traceback
            traceback.print_exc()