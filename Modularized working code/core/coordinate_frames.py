"""
Universal Coordinate Frame Transform System
Handles all transformations between sensor, visualization, and robot frames

Design:
    BNO085 Raw → Standard IMU Frame → Target Frames (Robot/Visualization)
    
Standard IMU Frame (Right-Handed):
    +X = Forward (toward fingers when hand extended)
    +Y = Left (thumb direction when palm down)
    +Z = Up (back of hand direction)

CHANGES:
- Fixed translation_x inversion: Changed from -1 to +1
- Forward tilt (positive roll) now produces forward movement (positive X)
"""

import numpy as np
import math
from typing import Dict, Tuple

class CoordinateFrameTransform:
    """
    Universal coordinate frame transformation system.
    Provides clean separation between sensor mounting and target frame usage.
    """
    
    def __init__(self):
        """
        Initialize transform system with BNO085 mounting configuration.
        
        Based on physical mounting (from AXIS_MAPPING_GUIDE.md):
        - User tilts forward/back → BNO reads ROLL
        - User tilts left/right → BNO reads PITCH  
        - User twists wrist → BNO reads YAW
        """
        
        # BNO085 is mounted 90° rotated relative to standard orientation
        # This mapping corrects for the physical mounting
        self.sensor_mounting = {
            'roll': 'roll',    # BNO roll = Standard roll (forward/back tilt)
            'pitch': 'pitch',  # BNO pitch = Standard pitch (left/right tilt)
            'yaw': 'yaw'       # BNO yaw = Standard yaw (twist)
        }
        
        # Robot Base Frame Mapping (UR Convention)
        # Maps Standard IMU axes to UR robot base frame
        # NOTE: These are the original values - test with robot before changing
        self.robot_translation_map = {
            'x': ('roll', 1),     # Forward tilt = forward movement
            'y': ('pitch', -1),   # Tilt left/right → Robot Y
            'z': ('pitch', 1)     # Mode 2 vertical
        }
        
        self.robot_rotation_map = {
            'rx': ('pitch', 1),   # Tilt left/right → Rotate around X
            'ry': ('yaw', -1),    # Twist wrist → Rotate around Y
            'rz': ('roll', 1)     # Tilt forward/back → Rotate around Z
        }
        
        # Visualization (OpenGL Cube) Frame Mapping
        # Maps Standard IMU to OpenGL cube rotations
        # UPDATED: Mirror-style control (cube faces away, moves with you)
        self.visualization_map = {
            'x': ('pitch', -1),   # INVERTED: Nod forward → cube nods away from you
            'y': ('yaw', 1),      # INVERTED: Look left → cube looks left (your perspective)
            'z': ('roll', 1)      # Tilt ear to shoulder → Cube Z rotation (unchanged)
        }
    
    def sensor_to_standard(self, bno_euler: Dict[str, float]) -> Dict[str, float]:
        """
        Convert BNO085 sensor frame to Standard IMU frame.
        
        Args:
            bno_euler: Dict with 'roll', 'pitch', 'yaw' in degrees from BNO085
        
        Returns:
            Dict with 'roll', 'pitch', 'yaw' in Standard IMU frame
        """
        # Currently 1:1 mapping since mounting correction is already handled
        # in the sensor_mounting definition
        return {
            'roll': bno_euler['roll'],
            'pitch': bno_euler['pitch'],
            'yaw': bno_euler['yaw']
        }
    
    def apply_mapping(self, 
                      imu_euler: Dict[str, float], 
                      mapping: Dict[str, Tuple[str, int]]) -> Dict[str, float]:
        """
        Generic mapping function to convert IMU frame to any target frame.
        
        Args:
            imu_euler: Standard IMU Euler angles in degrees
            mapping: Dictionary mapping target axes to (source_axis, sign)
        
        Returns:
            Dict with mapped values for each target axis
        """
        result = {}
        for target_axis, (source_axis, sign) in mapping.items():
            result[target_axis] = imu_euler[source_axis] * sign
        return result
    
    def to_robot_translation(self, imu_euler: Dict[str, float]) -> Dict[str, float]:
        """
        Convert Standard IMU frame to Robot translation deltas.
        
        Args:
            imu_euler: Standard IMU Euler angles in degrees
        
        Returns:
            Dict with 'x', 'y', 'z' translation values for robot base frame
        """
        return self.apply_mapping(imu_euler, self.robot_translation_map)
    
    def to_robot_rotation(self, imu_euler: Dict[str, float]) -> Dict[str, float]:
        """
        Convert Standard IMU frame to Robot rotation deltas.
        
        Args:
            imu_euler: Standard IMU Euler angles in degrees
        
        Returns:
            Dict with 'rx', 'ry', 'rz' rotation values for robot orientation
        """
        return self.apply_mapping(imu_euler, self.robot_rotation_map)
    
    def to_visualization(self, imu_euler: Dict[str, float]) -> Dict[str, float]:
        """
        Convert Standard IMU frame to OpenGL visualization rotations.
        
        Args:
            imu_euler: Standard IMU Euler angles in degrees
        
        Returns:
            Dict with 'x', 'y', 'z' rotation values for OpenGL cube
        """
        return self.apply_mapping(imu_euler, self.visualization_map)
    
    def apply_quaternion_mapping(self, quaternion: np.ndarray) -> np.ndarray:
        """
        Apply coordinate transform to a quaternion for visualization.
        
        Args:
            quaternion: [qi, qj, qk, qr] from BNO085
        
        Returns:
            Transformed quaternion for OpenGL rendering
        """
        # Convert quaternion to Euler, apply mapping, convert back
        from core.math_utils import quat_to_euler, euler_to_quat
        
        # Extract Euler angles (in radians)
        rx, ry, rz = quat_to_euler(quaternion)
        
        # Convert to degrees
        imu_euler = {
            'roll': math.degrees(rx),
            'pitch': math.degrees(ry),
            'yaw': math.degrees(rz)
        }
        
        # Apply visualization mapping
        viz_mapped = self.to_visualization(imu_euler)
        
        # Convert back to radians and reconstruct quaternion
        mapped_quat = euler_to_quat(
            math.radians(viz_mapped['x']),
            math.radians(viz_mapped['y']),
            math.radians(viz_mapped['z'])
        )
        
        return mapped_quat
    
    def get_info(self) -> str:
        """Return human-readable description of current frame mappings."""
        info = []
        info.append("="*70)
        info.append("COORDINATE FRAME MAPPINGS")
        info.append("="*70)
        info.append("\n[ROBOT TRANSLATION]")
        for axis, (source, sign) in self.robot_translation_map.items():
            sign_str = "+" if sign > 0 else "-"
            info.append(f"  Robot {axis.upper()}: {sign_str}{source}")
        
        info.append("\n[ROBOT ROTATION]")
        for axis, (source, sign) in self.robot_rotation_map.items():
            sign_str = "+" if sign > 0 else "-"
            info.append(f"  Robot {axis.upper()}: {sign_str}{source}")
        
        info.append("\n[VISUALIZATION]")
        for axis, (source, sign) in self.visualization_map.items():
            sign_str = "+" if sign > 0 else "-"
            info.append(f"  Cube {axis.upper()}: {sign_str}{source}")
        
        info.append("="*70)
        return "\n".join(info)


# Global instance for easy access
frame_transform = CoordinateFrameTransform()