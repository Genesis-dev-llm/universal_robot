"""
Robot movement mode implementations - Maps axis values to position/orientation deltas
"""

from config.constants import CONTROL_PARAMS
from core.math_utils import quaternion_to_rotation_vector, rotation_vector_add
import numpy as np

class MovementModes:
    """Control mode movement calculations. All methods expect pre-mapped axis values."""
    
    @staticmethod
    def calculate_base_frame_xy(x_delta, y_delta, scale):
        """MODE 1: XY translation in base frame"""
        return (
            [x_delta * CONTROL_PARAMS['base_translation'] * scale,
             y_delta * CONTROL_PARAMS['base_translation'] * scale, 0],
            [0, 0, 0]
        )
    
    @staticmethod
    def calculate_vertical_z(z_delta, scale):
        """MODE 2: Vertical Z movement"""
        return [0, 0, z_delta * CONTROL_PARAMS['vertical'] * scale], [0, 0, 0]
    
    @staticmethod
    def calculate_base_frame_orientation(rx, ry, rz, scale):
        """MODE 3: Orientation in base frame (Rx, Ry, Rz)"""
        return [0, 0, 0], [
            rx * CONTROL_PARAMS['robot_orientation'] * scale,
            ry * CONTROL_PARAMS['robot_orientation'] * scale,
            rz * CONTROL_PARAMS['base_rotation'] * scale
        ]
    
    @staticmethod
    def calculate_tcp_xy(x_delta, y_delta, scale):
        """MODE 4: Fine TCP XY positioning"""
        return (
            [x_delta * CONTROL_PARAMS['tcp_translation'] * scale,
             y_delta * CONTROL_PARAMS['tcp_translation'] * scale, 0],
            [0, 0, 0]
        )
    
    @staticmethod
    def calculate_tcp_z(z_delta, scale):
        """MODE 5: Fine TCP Z positioning"""
        return [0, 0, z_delta * CONTROL_PARAMS['tcp_vertical'] * scale], [0, 0, 0]
    
    @staticmethod
    def calculate_tcp_orientation(target_quat, current_orientation, scale):
        """MODE 6: TCP orientation from quaternion"""
        target_rv = np.array(quaternion_to_rotation_vector(target_quat)) * scale
        orientation_delta = rotation_vector_add(target_rv.tolist(), [-x for x in current_orientation])
        return [0, 0, 0], orientation_delta