"""
Mathematical utilities for robot control
Quaternion operations, coordinate transforms, filtering
"""

import numpy as np
import math
from config.constants import UR_LIMITS

#==============================================================================
# FILTERING FUNCTIONS
#==============================================================================

def apply_deadzone_ramp(value, deadzone, ramp_width):
    """
    Apply deadzone with smooth ramping to avoid jerky motion
    
    Args:
        value: Input value (degrees)
        deadzone: Deadzone threshold (degrees)
        ramp_width: Ramp transition width (degrees)
    
    Returns:
        Filtered value with smooth ramping
    """
    abs_value = abs(value)
    
    if abs_value < deadzone:
        return 0.0  # Inside deadzone
    elif abs_value < deadzone + ramp_width:
        # In ramp zone - linear interpolation
        ramp_factor = (abs_value - deadzone) / ramp_width
        return value * ramp_factor
    else:
        # Outside ramp zone - full value
        return value

def get_mapped_axis(euler_dict, axis_name, invert=False):
    """
    Get the mapped IMU axis value based on configuration.
    
    Args:
        euler_dict: Dictionary with 'roll', 'pitch', 'yaw' values
        axis_name: Which axis to get ('roll', 'pitch', or 'yaw')
        invert: Whether to invert the value
    
    Returns:
        The mapped (and optionally inverted) value
    """
    value = euler_dict.get(axis_name, 0.0)
    return -value if invert else value

def apply_axis_mapping(roll, pitch, yaw, mapping):
    """
    Apply IMU axis mapping configuration to remap and invert axes.
    
    Args:
        roll, pitch, yaw: Raw IMU Euler angles
        mapping: IMU_AXIS_MAPPING dict from constants
    
    Returns:
        Dict with mapped values for x, y, z, rx, ry, rz movements
    """
    euler = {'roll': roll, 'pitch': pitch, 'yaw': yaw}
    
    return {
        'x': get_mapped_axis(euler, mapping['x_axis'], mapping['x_invert']),
        'y': get_mapped_axis(euler, mapping['y_axis'], mapping['y_invert']),
        'z': get_mapped_axis(euler, mapping['z_axis'], mapping['z_invert']),
        'rx': get_mapped_axis(euler, mapping['rx_axis'], mapping['rx_invert']),
        'ry': get_mapped_axis(euler, mapping['ry_axis'], mapping['ry_invert']),
        'rz': get_mapped_axis(euler, mapping['rz_axis'], mapping['rz_invert']),
    }

#==============================================================================
# QUATERNION OPERATIONS
#==============================================================================

def quaternion_normalize(q):
    """
    Normalize quaternion to unit length
    
    Args:
        q: Quaternion array [i, j, k, w] or [x, y, z, w]
    
    Returns:
        Normalized quaternion
    """
    norm = np.linalg.norm(q)
    return q / norm if norm > 0.0001 else np.array([0, 0, 0, 1])

def quaternion_to_rotation_vector(q):
    """
    Convert quaternion [i,j,k,w] to UR rotation vector [rx,ry,rz]
    UR uses axis-angle representation
    
    Args:
        q: Quaternion [i, j, k, w]
    
    Returns:
        Rotation vector [rx, ry, rz] in radians
    """
    q = quaternion_normalize(q)
    i, j, k, w = q[0], q[1], q[2], q[3]
    
    # Ensure w is in valid range
    w = max(-1.0, min(1.0, w))
    
    # Calculate angle
    angle = 2 * math.acos(w)
    
    if abs(angle) < 0.001:
        return [0.0, 0.0, 0.0]
    
    sin_half = math.sin(angle / 2)
    if abs(sin_half) < 0.001:
        return [0.0, 0.0, 0.0]
    
    # Axis vector (normalized i, j, k components) scaled by the angle
    return [i * angle / sin_half, j * angle / sin_half, k * angle / sin_half]

def rotation_vector_add(rv1, rv2):
    """
    Add two rotation vectors via quaternion multiplication.
    Converts to quaternions, multiplies, converts back.
    
    Args:
        rv1: First rotation vector [rx, ry, rz]
        rv2: Second rotation vector [rx, ry, rz]
    
    Returns:
        Combined rotation vector [rx, ry, rz]
    """
    def rv_to_quat(rv):
        """Convert rotation vector to quaternion"""
        angle = np.linalg.norm(rv)
        if angle < 0.001:
            return [0, 0, 0, 1]
        axis = np.array(rv) / angle
        half_angle = angle / 2
        sin_half = math.sin(half_angle)
        return [axis[0]*sin_half, axis[1]*sin_half, axis[2]*sin_half, math.cos(half_angle)]
    
    def quat_mult(q1, q2):
        """Multiply two quaternions"""
        # q = [x, y, z, w]
        x1, y1, z1, w1 = q1[0], q1[1], q1[2], q1[3]
        x2, y2, z2, w2 = q2[0], q2[1], q2[2], q2[3]
        return [
            w1*x2 + x1*w2 + y1*z2 - z1*y2,  # x
            w1*y2 - x1*z2 + y1*w2 + z1*x2,  # y
            w1*z2 + x1*y2 - y1*x2 + z1*w2,  # z
            w1*w2 - x1*x2 - y1*y2 - z1*z2   # w
        ]
    
    q1 = rv_to_quat(rv1)
    q2 = rv_to_quat(rv2)
    q_result = quat_mult(q1, q2)
    return quaternion_to_rotation_vector(q_result)

#==============================================================================
# COORDINATE TRANSFORMS
#==============================================================================

def clamp_position(pos):
    """
    Clamp position to workspace limits
    
    Args:
        pos: Position [x, y, z] in meters
    
    Returns:
        Clamped position within workspace
    """
    return [
        max(UR_LIMITS['x_min'], min(UR_LIMITS['x_max'], pos[0])),
        max(UR_LIMITS['y_min'], min(UR_LIMITS['y_max'], pos[1])),
        max(UR_LIMITS['z_min'], min(UR_LIMITS['z_max'], pos[2]))
    ]

#==============================================================================
# VALIDATION UTILITIES
#==============================================================================

def is_valid_pose(pose):
    """
    Check if pose data is valid (no NaN, Inf, correct length)
    
    Args:
        pose: Pose array [x, y, z, rx, ry, rz]
    
    Returns:
        True if valid, False otherwise
    """
    if len(pose) != 6:
        return False
    return all(math.isfinite(x) for x in pose)

def is_valid_quaternion(q):
    """
    Check if quaternion is valid
    
    Args:
        q: Quaternion [i, j, k, w]
    
    Returns:
        True if valid, False otherwise
    """
    if len(q) != 4:
        return False
    if not all(math.isfinite(x) for x in q):
        return False
    # Check if nearly unit length (within tolerance)
    norm = np.linalg.norm(q)
    return abs(norm - 1.0) < 0.1  # Allow some tolerance

def calculate_angular_velocity(roll, pitch, yaw):
    """
    Calculate total angular velocity magnitude
    
    Args:
        roll, pitch, yaw: Euler angles in degrees
    
    Returns:
        Angular velocity magnitude in degrees/s
    """
    return math.sqrt(roll**2 + pitch**2 + yaw**2)

def quaternion_multiply(q1, q2):
    """Multiply two quaternions [i, j, k, w]"""
    # q = [x, y, z, w]
    x1, y1, z1, w1 = q1[0], q1[1], q1[2], q1[3]
    x2, y2, z2, w2 = q2[0], q2[1], q2[2], q2[3]
    return np.array([
        w1*x2 + x1*w2 + y1*z2 - z1*y2,  # x
        w1*y2 - x1*z2 + y1*w2 + z1*x2,  # y
        w1*z2 + x1*y2 - y1*x2 + z1*w2,  # z
        w1*w2 - x1*x2 - y1*y2 - z1*z2   # w
    ])

def quaternion_inverse(q):
    """
    Calculate inverse (conjugate for unit quaternions) of quaternion [i,j,k,w].
    """
    q = np.array(q)
    return np.array([-q[0], -q[1], -q[2], q[3]])

def quaternion_difference(q1, q2):
    """
    Calculate rotation from q1 to q2 (delta quaternion).
    Formula: q_delta = q2 * inverse(q1)
    """
    q1_inv = quaternion_inverse(q1)
    return quaternion_multiply(q2, q1_inv)

def apply_axis_mapping_quat(q, mapping):
    """
    Apply IMU axis mapping (swaps and inversions) to a quaternion.
    This ensures absolute orientation (cube/mimic) follows same mapping as translations.
    
    Args:
        q: Absolute quaternion [i, j, k, w]
        mapping: IMU_AXIS_MAPPING dict
        
    Returns:
        Mapped quaternion [i, j, k, w]
    """
    # 1. Convert to Euler
    rx, ry, rz = quat_to_euler(q)
    
    # 2. Apply mapping logic similar to apply_axis_mapping
    euler = {'roll': math.degrees(rx), 'pitch': math.degrees(ry), 'yaw': math.degrees(rz)}
    
    # Extract mapped values in degrees, then back to radians
    m_rx = math.radians(get_mapped_axis(euler, mapping['rx_axis'], mapping['rx_invert']))
    m_ry = math.radians(get_mapped_axis(euler, mapping['ry_axis'], mapping['ry_invert']))
    m_rz = math.radians(get_mapped_axis(euler, mapping['rz_axis'], mapping['rz_invert']))
    
    # 3. Reconstruct mapped quaternion
    return euler_to_quat(m_rx, m_ry, m_rz)

def quat_to_euler(q):
    """
    Convert quaternion to Euler angles (rx, ry, rz) in radians.
    Uses ZYX convention (yaw-pitch-roll).
    """
    qi, qj, qk, qr = q[0], q[1], q[2], q[3]
    
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (qr * qi + qj * qk)
    cosr_cosp = 1 - 2 * (qi**2 + qj**2)
    rx = math.atan2(sinr_cosp, cosr_cosp)
    
    # Pitch (y-axis rotation)
    sinp = 2 * (qr * qj - qk * qi)
    if abs(sinp) >= 1:
        ry = math.copysign(math.pi / 2, sinp)
    else:
        ry = math.asin(sinp)
    
    # Yaw (z-axis rotation)
    siny_cosp = 2 * (qr * qk + qi * qj)
    cosy_cosp = 1 - 2 * (qj**2 + qk**2)
    rz = math.atan2(siny_cosp, cosy_cosp)
    
    return [rx, ry, rz]

def euler_to_quat(roll, pitch, yaw):
    """
    Convert Euler angles (rad) to quaternion [i, j, k, w].
    Uses ZYX convention (yaw-pitch-roll).
    """
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    return np.array([
        sr * cp * cy - cr * sp * sy, # i (x)
        cr * sp * cy + sr * cp * sy, # j (y)
        cr * cp * sy - sr * sp * cy, # k (z)
        cr * cp * cy + sr * sp * sy  # w (r)
    ])