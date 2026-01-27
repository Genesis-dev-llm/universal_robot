"""
Universal Coordinate Frame Transform System
Handles all transformations between sensor, visualization, and robot frames

SENSOR BEHAVIOR (BNO085 on glove):
    - NOD forward/back (look up/down) → PITCH changes
    - TILT left/right (ear to shoulder) → ROLL changes
    - TWIST wrist (doorknob rotation) → YAW changes

COORDINATE SYSTEM:
    BNO085 Raw Data → Standard IMU Frame → Target Frames (Robot/Visualization)
    
Standard IMU Frame (Right-Handed):
    +X = Forward (fingers pointing direction)
    +Y = Left (thumb direction)
    +Z = Up (back of hand direction)

FIXED: All axis mappings now correctly match physical sensor behavior
"""

import numpy as np
import math
from typing import Dict, Tuple

class CoordinateFrameTransform:
    """
    Universal coordinate frame transformation system.
    Maps BNO085 sensor data to robot commands and visualization.
    
    All signs are +1 for natural "mirror" control - your movements
    directly map to robot/visualization movements in the same direction.
    """
    
    def __init__(self):
        """
        Initialize transform system with corrected BNO085 mappings.
        
        Physical Sensor Behavior:
        - User NODS hand up/down → BNO085 PITCH changes
        - User TILTS hand left/right → BNO085 ROLL changes
        - User TWISTS wrist → BNO085 YAW changes
        """
        
        # Sensor mounting configuration (currently 1:1 pass-through)
        # The BNO085 reports standard Euler angles that we use directly
        self.sensor_mounting = {
            'roll': 'roll',    # Tilt gesture
            'pitch': 'pitch',  # Nod gesture
            'yaw': 'yaw'       # Twist gesture
        }
        
        # ================================================================
        # ROBOT BASE FRAME MAPPING (UR Convention)
        # Maps hand gestures to robot translation commands
        # ================================================================
        self.robot_translation_map = {
            # Mode 1: NOD forward/back → TCP moves forward/back (relative to tool)
            # Mode 2: NOD up/down → Robot moves up/down vertically
            'x': ('pitch', 1),    # Nod controls forward/back movement
            
            # Mode 1: TILT left/right → Base rotates (crane swing)
            # Mode 3: TILT left/right → TCP moves left/right (relative to tool)
            'y': ('roll', 1),     # Tilt controls lateral movement / base rotation
            
            # Mode 2: NOD up/down → Vertical movement
            'z': ('pitch', 1)     # Nod controls vertical movement
        }
        
        # ================================================================
        # ROBOT ROTATION MAPPING (Joint Control)
        # Maps hand gestures to robot orientation/joint commands
        # ================================================================
        self.robot_rotation_map = {
            # Mode 4: TILT left/right → Wrist 2 rotates
            # Mode 5: TILT left/right → Wrist 3 rotates (screwdriver)
            'rx': ('roll', 1),    # Tilt controls wrist rotations
            
            # Future use: Twist controls orientation around Y-axis
            'ry': ('yaw', 1),     # Twist (not currently used in modes 1-5)
            
            # Mode 1: Base rotation uses this via robot_trans['y']
            # Mode 4: NOD up/down → Wrist 1 rotates
            'rz': ('pitch', 1)    # Nod controls base/wrist rotation
        }
        
        # ================================================================
        # VISUALIZATION (OpenGL Cube) FRAME MAPPING
        # Maps hand gestures to cube display rotations
        # "Mirror-style" control - cube faces away, mimics your hand
        # ================================================================
        self.visualization_map = {
            # NOD down → Cube nods away from you (mirrors your nod)
            'x': ('pitch', 1),    # Nod gesture → Cube pitch rotation
            
            # TWIST clockwise → Cube twists clockwise (from your view)
            'y': ('yaw', 1),      # Twist gesture → Cube yaw rotation
            
            # TILT right → Cube tilts right (mirrors your tilt)
            'z': ('roll', 1)      # Tilt gesture → Cube roll rotation
        }
    
    def sensor_to_standard(self, bno_euler: Dict[str, float]) -> Dict[str, float]:
        """
        Convert BNO085 sensor frame to Standard IMU frame.
        Currently 1:1 pass-through as sensor reports standard Euler angles.
        
        Args:
            bno_euler: Dict with 'roll', 'pitch', 'yaw' in degrees from BNO085
        
        Returns:
            Dict with 'roll', 'pitch', 'yaw' in Standard IMU frame
        """
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
                    Example: {'x': ('pitch', 1)} means target X = source pitch * 1
        
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
        Used by Modes 1, 2, 3 for movement commands.
        
        Args:
            imu_euler: Standard IMU Euler angles in degrees
        
        Returns:
            Dict with 'x', 'y', 'z' translation values for robot base frame
            
        Examples:
            - Nod forward → {'x': +10, 'y': 0, 'z': +10}
            - Tilt right → {'x': 0, 'y': +5, 'z': 0}
        """
        return self.apply_mapping(imu_euler, self.robot_translation_map)
    
    def to_robot_rotation(self, imu_euler: Dict[str, float]) -> Dict[str, float]:
        """
        Convert Standard IMU frame to Robot rotation deltas.
        Used by Modes 4, 5 for joint/orientation commands.
        
        Args:
            imu_euler: Standard IMU Euler angles in degrees
        
        Returns:
            Dict with 'rx', 'ry', 'rz' rotation values for robot orientation
            
        Examples:
            - Nod forward → {'rx': 0, 'ry': 0, 'rz': +10}
            - Tilt right → {'rx': +5, 'ry': 0, 'rz': 0}
        """
        return self.apply_mapping(imu_euler, self.robot_rotation_map)
    
    def to_visualization(self, imu_euler: Dict[str, float]) -> Dict[str, float]:
        """
        Convert Standard IMU frame to OpenGL visualization rotations.
        Creates "mirror-style" control for the cube display.
        
        Args:
            imu_euler: Standard IMU Euler angles in degrees
        
        Returns:
            Dict with 'x', 'y', 'z' rotation values for OpenGL cube
            
        Examples:
            - Nod down → Cube nods away from you
            - Twist right → Cube twists right (from your perspective)
        """
        return self.apply_mapping(imu_euler, self.visualization_map)
    
    def apply_quaternion_mapping(self, quaternion: np.ndarray) -> np.ndarray:
        """
        Apply coordinate transform to a quaternion for visualization.
        Converts quaternion to Euler, applies mapping, converts back.
        
        Args:
            quaternion: [qi, qj, qk, qr] from BNO085
        
        Returns:
            Transformed quaternion for OpenGL rendering
        """
        from core.math_utils import quat_to_euler, euler_to_quat
        
        # Extract Euler angles (in radians)
        rx, ry, rz = quat_to_euler(quaternion)
        
        # Convert to degrees for mapping
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
        """
        Return human-readable description of current frame mappings.
        Useful for debugging and verification.
        """
        info = []
        info.append("="*70)
        info.append("COORDINATE FRAME MAPPINGS")
        info.append("="*70)
        info.append("\n[SENSOR BEHAVIOR]")
        info.append("  NOD forward/back → PITCH changes")
        info.append("  TILT left/right → ROLL changes")
        info.append("  TWIST wrist → YAW changes")
        
        info.append("\n[ROBOT TRANSLATION]")
        for axis, (source, sign) in self.robot_translation_map.items():
            sign_str = "+" if sign > 0 else "-"
            gesture = "NOD" if source == "pitch" else "TILT" if source == "roll" else "TWIST"
            info.append(f"  Robot {axis.upper()}: {sign_str}{source} ({gesture})")
        
        info.append("\n[ROBOT ROTATION]")
        for axis, (source, sign) in self.robot_rotation_map.items():
            sign_str = "+" if sign > 0 else "-"
            gesture = "NOD" if source == "pitch" else "TILT" if source == "roll" else "TWIST"
            info.append(f"  Robot {axis.upper()}: {sign_str}{source} ({gesture})")
        
        info.append("\n[VISUALIZATION]")
        for axis, (source, sign) in self.visualization_map.items():
            sign_str = "+" if sign > 0 else "-"
            gesture = "NOD" if source == "pitch" else "TILT" if source == "roll" else "TWIST"
            info.append(f"  Cube {axis.upper()}: {sign_str}{source} ({gesture})")
        
        info.append("="*70)
        return "\n".join(info)


# Global instance for easy access throughout the codebase
frame_transform = CoordinateFrameTransform()