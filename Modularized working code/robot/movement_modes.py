"""
Robot movement mode implementations - UR Coordinate System Corrected
Maps IMU axis values to robot velocity commands

UR BASE FRAME CONVENTION (ISO 9787):
    X-axis: Left (-) / Right (+)
    Y-axis: Back (-) / Forward (+)  
    Z-axis: Down (-) / Up (+)

REDESIGNED MODE SYSTEM:
- Mode 1: Crane control (TCP translation OR base rotation - priority based)
- Mode 2: Vertical Z control
- Mode 3: Base XY plane control (UPDATED - full XY translation in base frame)
- Mode 4: Wrist 1 & 2 direct joint control (speedJ)
- Mode 5: Wrist 3 screwdriver motion (speedJ)
- Mode 6: Full orientation mimic (handled separately)

UPDATED:
- Mode 3: Now handles both X and Y axes for full base-frame XY plane control
- No TCP transformation needed - direct base frame velocities
"""

from config.constants import CONTROL_PARAMS

class MovementModes:
    """
    Control mode movement calculations.
    All methods expect pre-mapped axis values from coordinate_frames.py
    
    Returns:
    - Modes 1-3: Velocities for speedL (linear + angular in appropriate frames)
    - Modes 4-5: Joint velocities for speedJ (6-element array)
    - Mode 1 base rotation: Joint velocities for speedJ (6-element array)
    """
    
    @staticmethod
    def calculate_crane_mode(x_delta, y_delta, linear_scale, angular_scale):
        """
        MODE 1: Crane Control
        
        Behavior:
        - Forward/Back tilt → TCP moves forward/back (UR Y-axis in TCP frame)
        - Left/Right tilt → Base rotates (crane-like swing)
        
        NOTE: This returns TCP-frame linear velocity. The control dispatcher
        will decide whether to use speedL (for translation) or speedJ (for rotation)
        based on which input is dominant.
        
        Args:
            x_delta: Forward/back input (degrees)
            y_delta: Left/right input (degrees)
            linear_scale: Linear speed multiplier
            angular_scale: Angular speed multiplier
        
        Returns:
            Tuple: (linear_velocity_tcp_frame, angular_velocity_base_frame)
            - linear: [0, vy, 0] in TCP frame (needs transformation in dispatcher)
            - angular: [0, 0, rz] in base frame (base rotation)
        """
        # Forward/back translation uses UR Y-axis
        linear_velocity = [
            0.0,
            x_delta * CONTROL_PARAMS['base_translation'] * linear_scale,  # Y = Forward/Back
            0.0
        ]
        
        # Left/right base rotation (Base Z-axis)
        angular_velocity = [
            0.0,
            0.0,
            y_delta * CONTROL_PARAMS['base_rotation'] * angular_scale
        ]
        
        return (linear_velocity, angular_velocity)
    
    @staticmethod
    def calculate_base_rotation_joint_velocity(y_delta, angular_scale):
        """
        MODE 1: Base Rotation (speedJ variant)
        
        Calculates joint velocity for ONLY base rotation (Joint 0).
        Used when left/right tilt is dominant in Mode 1.
        
        Args:
            y_delta: Left/right input (degrees)
            angular_scale: Angular speed multiplier
        
        Returns:
            List of 6 joint velocities [q0, q1, q2, q3, q4, q5]
            Only Joint 0 (base) has non-zero value
        """
        joint_velocities = [
            y_delta * CONTROL_PARAMS['base_rotation'] * angular_scale,  # Joint 0: Base rotation
            0.0,  # Shoulder (locked)
            0.0,  # Elbow (locked)
            0.0,  # Wrist 1 (locked)
            0.0,  # Wrist 2 (locked)
            0.0   # Wrist 3 (locked)
        ]
        
        return joint_velocities
    
    @staticmethod
    def calculate_vertical_z(z_delta, linear_scale):
        """
        MODE 2: Vertical Control
        
        Behavior:
        - Up/Down tilt → TCP moves vertically (UR Z-axis)
        
        Args:
            z_delta: Vertical input (degrees)
            linear_scale: Linear speed multiplier
        
        Returns:
            Tuple: (linear_velocity, angular_velocity)
            - linear: [0, 0, vz]
            - angular: [0, 0, 0]
        """
        linear_velocity = [
            0.0,
            0.0,
            z_delta * CONTROL_PARAMS['vertical'] * linear_scale
        ]
        
        angular_velocity = [0.0, 0.0, 0.0]
        
        return (linear_velocity, angular_velocity)
    
    @staticmethod
    def calculate_lateral_precise(x_delta, y_delta, linear_scale):
        """
        MODE 3: Base XY Plane Control (UPDATED)
        
        Behavior:
        - Left/Right tilt → Robot moves left/right (UR X-axis in BASE frame)
        - Forward/Back tilt → Robot moves forward/back (UR Y-axis in BASE frame)
        
        CHANGED: Now accepts both X and Y deltas for full XY plane control
        No TCP transformation needed - velocities are directly in base frame
        
        Args:
            x_delta: Forward/back input (degrees)
            y_delta: Left/right input (degrees)
            linear_scale: Linear speed multiplier
        
        Returns:
            Tuple: (linear_velocity_base_frame, angular_velocity)
            - linear: [vx, vy, 0] in BASE frame (direct control, no transformation needed)
            - angular: [0, 0, 0]
        """
        # Base frame XY plane translation
        linear_velocity = [
            y_delta * CONTROL_PARAMS['tcp_translation'] * linear_scale,  # X = Left/Right
            x_delta * CONTROL_PARAMS['tcp_translation'] * linear_scale,  # Y = Forward/Back
            0.0                                                           # Z = No vertical
        ]
        
        angular_velocity = [0.0, 0.0, 0.0]
        
        return (linear_velocity, angular_velocity)
    
    @staticmethod
    def calculate_wrist_joint_velocities(rx_delta, rz_delta, angular_scale):
        """
        MODE 4: Wrist 1 & 2 Direct Joint Control
        
        Behavior:
        - Left/Right tilt → Wrist 1 rotation (Joint 3)
        - Forward/Back tilt → Wrist 2 rotation (Joint 4)
        
        Uses speedJ for precise wrist-only control.
        
        Args:
            rx_delta: Left/right input (degrees) - for Wrist 1
            rz_delta: Forward/back input (degrees) - for Wrist 2
            angular_scale: Angular speed multiplier
        
        Returns:
            List of 6 joint velocities [q0, q1, q2, q3, q4, q5]
            Only Wrist 1 (q3) and Wrist 2 (q4) have non-zero values
        """
        joint_velocities = [
            0.0,  # Base (locked)
            0.0,  # Shoulder (locked)
            0.0,  # Elbow (locked)
            -rx_delta * CONTROL_PARAMS['wrist_rotation'] * angular_scale,  # Wrist 1 (INVERTED)
            rz_delta * CONTROL_PARAMS['wrist_rotation'] * angular_scale,   # Wrist 2
            0.0   # Wrist 3 (locked)
        ]
        
        return joint_velocities
    
    @staticmethod
    def calculate_wrist3_joint_velocity(rx_delta, angular_scale):
        """
        MODE 5: Wrist 3 Screwdriver Motion
        
        Behavior:
        - Left/Right tilt → Wrist 3 rotation (Joint 5) - like turning a screwdriver
        
        Uses speedJ for precise wrist-only control.
        
        Args:
            rx_delta: Left/right input (degrees) - for Wrist 3
            angular_scale: Angular speed multiplier
        
        Returns:
            List of 6 joint velocities [q0, q1, q2, q3, q4, q5]
            Only Wrist 3 (q5) has non-zero value
        """
        joint_velocities = [
            0.0,  # Base (locked)
            0.0,  # Shoulder (locked)
            0.0,  # Elbow (locked)
            0.0,  # Wrist 1 (locked)
            0.0,  # Wrist 2 (locked)
            -rx_delta * CONTROL_PARAMS['wrist_rotation'] * angular_scale   # Wrist 3 (INVERTED)
        ]
        
        return joint_velocities