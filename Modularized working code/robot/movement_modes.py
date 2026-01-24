"""
Robot movement mode implementations - Complete Redesign
Maps IMU axis values to robot velocity commands

REDESIGNED MODE SYSTEM:
- Mode 1: Crane control (TCP translation + base rotation)
- Mode 2: Vertical Z control
- Mode 3: Lateral precise control (TCP-relative)
- Mode 4: Wrist 1 & 2 direct joint control (speedJ)
- Mode 5: Wrist 3 screwdriver motion (speedJ)
- Mode 6: Full orientation mimic (handled separately)
"""

from config.constants import CONTROL_PARAMS

class MovementModes:
    """
    Control mode movement calculations.
    All methods expect pre-mapped axis values from coordinate_frames.py
    
    Returns:
    - Modes 1-3: Velocities for speedL (linear + angular in appropriate frames)
    - Modes 4-5: Joint velocities for speedJ (6-element array)
    """
    
    @staticmethod
    def calculate_crane_mode(x_delta, y_delta, linear_scale, angular_scale):
        """
        MODE 1: Crane Control
        
        Behavior:
        - Forward/Back tilt → TCP moves forward/back (in TCP frame)
        - Left/Right tilt → Base rotates (crane-like swing)
        
        Args:
            x_delta: Forward/back input (degrees)
            y_delta: Left/right input (degrees)
            linear_scale: Linear speed multiplier
            angular_scale: Angular speed multiplier
        
        Returns:
            Tuple: (linear_velocity_tcp_frame, angular_velocity_base_frame)
            - linear: [vx, 0, 0] in TCP frame (needs transformation in dispatcher)
            - angular: [0, 0, rz] in base frame (base rotation)
        """
        # Forward/back translation (TCP X-axis)
        linear_velocity = [
            x_delta * CONTROL_PARAMS['base_translation'] * linear_scale,
            0.0,
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
    def calculate_vertical_z(z_delta, linear_scale):
        """
        MODE 2: Vertical Control
        
        Behavior:
        - Up/Down tilt → TCP moves vertically
        
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
    def calculate_lateral_precise(y_delta, linear_scale):
        """
        MODE 3: Lateral Precise Control
        
        Behavior:
        - Left/Right tilt → TCP moves left/right (in TCP frame)
        
        Args:
            y_delta: Left/right input (degrees)
            linear_scale: Linear speed multiplier
        
        Returns:
            Tuple: (linear_velocity_tcp_frame, angular_velocity)
            - linear: [0, vy, 0] in TCP frame (needs transformation in dispatcher)
            - angular: [0, 0, 0]
        """
        linear_velocity = [
            0.0,
            y_delta * CONTROL_PARAMS['tcp_translation'] * linear_scale,
            0.0
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
            rx_delta * CONTROL_PARAMS['wrist_rotation'] * angular_scale,  # Wrist 1 (left/right)
            rz_delta * CONTROL_PARAMS['wrist_rotation'] * angular_scale,  # Wrist 2 (forward/back)
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
            rx_delta * CONTROL_PARAMS['wrist_rotation'] * angular_scale   # Wrist 3 (left/right)
        ]
        
        return joint_velocities