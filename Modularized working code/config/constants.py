"""
Configuration constants for UR Robot RTDE + IMU Control System
All hardware settings, limits, and control parameters
"""

import math

#==============================================================================
# COMMUNICATION SETTINGS
#==============================================================================

SERIAL_PORT = '/dev/ttyUSB0'  # Linux/Mac: /dev/ttyUSB0, Windows: COM3
BAUD_RATE = 115200
WIFI_HOST = '192.168.4.1'
WIFI_PORT = 3333
USE_WIFI = False  # Set True for WiFi, False for USB

#==============================================================================
# UNIVERSAL ROBOT SETTINGS
#==============================================================================

UR_ROBOT_IP = "192.168.1.100"
UR_ENABLED = False  # Set True to enable robot control
UR_SIMULATE = True  # Set False for real robot commands

# UR Joint Limits (radians) - UR5/UR10 typical values
UR_JOINT_LIMITS = {
    'joint_0': (-2*math.pi, 2*math.pi),
    'joint_1': (-math.pi, math.pi),
    'joint_2': (-math.pi, math.pi),
    'joint_3': (-math.pi, math.pi),
    'joint_4': (-math.pi, math.pi),
    'joint_5': (-2*math.pi, 2*math.pi),
}

# UR Workspace limits (meters, relative to robot base)
UR_LIMITS = {
    'x_min': -0.85, 'x_max': 0.85,
    'y_min': -0.85, 'y_max': 0.85,
    'z_min': 0.05, 'z_max': 1.2
}

# UR Control parameters
UR_BASE_POSITION = [0.4, 0.0, 0.6]  # Starting TCP position [x, y, z] in meters
UR_BASE_ORIENTATION = [0.0, 0.0, 0.0]  # Starting orientation [rx, ry, rz] rotation vector

# Neutral "Home" Position (Bent arm, safe for restart)
# [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3] in radians
UR_NEUTRAL_JOINT_POSITIONS = [0, -1.57, -1.57, -1.57, 1.57, 0]

UR_MAX_VELOCITY = 0.25  # m/s
UR_MAX_ACCELERATION = 1.2  # m/s²
UR_JOINT_VELOCITY = 1.05  # rad/s
UR_JOINT_ACCELERATION = 1.4  # rad/s²

#==============================================================================
# CONTROL PARAMETERS
#==============================================================================

# Control parameters (meters per degree for translation, radians per degree for rotation)
CONTROL_PARAMS = {
    'base_translation': 0.002,
    'base_rotation': 0.01,
    'vertical': 0.002,
    'tcp_translation': 0.0005,
    'tcp_vertical': 0.0005,
    'tcp_orientation': 0.005,
    'robot_orientation': 0.008,
}

# Movement filtering parameters
MOVEMENT_DEADZONE = 5.0  # degrees - no movement below this
DEADZONE_RAMP_WIDTH = 5.0  # degrees - gradual ramp zone (5-10 deg)
VELOCITY_DECAY = 0.95  # visualization smoothing

#==============================================================================
# IMU AXIS MAPPING
# Maps IMU axes to movement axes. Adjust based on how BNO085 is mounted.
# 
# Your current setup (based on your description):
#   - BNO "pitch" = your hand's up/down tilt
#   - BNO "roll" = your hand's left/right tilt  
#   - BNO "yaw" = your hand's rotation (twist)
#
# Options for each: 'roll', 'pitch', 'yaw'
# Set invert to True to flip the direction
#==============================================================================

IMU_AXIS_MAPPING = {
    # For Mode 1 (BASE_FRAME_XY): which IMU axis moves X and Y?
    'x_axis': 'roll',       # User Tilt Fwd/Back (reads as Roll) -> Robot X (Fwd/Back)
    'x_invert': True,       # Inverted as requested (Fwd moves Fwd)
    'y_axis': 'pitch',      # User Tilt Left/Right (reads as Pitch) -> Robot Y (Left/Right)
    'y_invert': False,      # Inverted back (Left moves Left)
    
    # For Mode 2/5 (VERTICAL_Z): which IMU axis moves Z?
    'z_axis': 'pitch',      # Up/down movement
    'z_invert': False,      # Tilt-forward = move-up
    
    'rx_axis': 'pitch',     # User Tilt Left/Right -> Robot Roll (Tilt Sideways)
    'rx_invert': False,     # Aligned with Y_AXIS (Left/Right)
    'ry_axis': 'yaw',       # User Twist Wrist -> Robot Yaw (Shake Head)
    'ry_invert': True,      # Inverted as requested
    'rz_axis': 'roll',      # User Tilt Fwd/Back -> Robot Pitch (Nod Fwd/Back)
    'rz_invert': False,     # Inverted as requested
}

#==============================================================================
# CONTROL MODES
#==============================================================================

CONTROL_MODES = {
    0: "IDLE",
    1: "BASE_FRAME_XY",      # Remapped: XY translation in base frame
    2: "VERTICAL_Z",         # Vertical movement
    3: "BASE_FRAME_ORIENT",  # Remapped: Orientation control in base frame (Rx, Ry, Rz)
    4: "TCP_XY",             # Fine TCP XY positioning
    5: "TCP_Z",              # Fine TCP vertical
    6: "TCP_ORIENT"          # TCP orientation control
}

#==============================================================================
# SAFETY THRESHOLDS
#==============================================================================

# Singularity thresholds (radians)
WRIST_SINGULARITY_THRESHOLD = 0.15
SHOULDER_SINGULARITY_THRESHOLD = 0.15
ELBOW_SINGULARITY_THRESHOLD = 0.1

# Joint safety margin (radians)
JOINT_SAFETY_MARGIN = 0.05  # ~3 degrees

# Connection settings
MAX_RECONNECT_ATTEMPTS = 5
RECONNECT_COOLDOWN = 2.0  # seconds
STATE_UPDATE_INTERVAL = 0.1  # seconds
RTDE_MAX_FREQUENCY = 0.008  # 125Hz (8ms period)

#==============================================================================
# VISUALIZATION SETTINGS
#==============================================================================

DEFAULT_WINDOW_WIDTH = 1200
DEFAULT_WINDOW_HEIGHT = 800
DEFAULT_CAMERA_DISTANCE = 20.0
TARGET_FPS = 125  # Match RTDE frequency

#==============================================================================
# ERROR CODES
#==============================================================================

ERROR_CODES = {
    # Connection errors (E1xx)
    'E101': 'RTDE Connection Error',
    'E102': 'RTDE Connection Failed',
    'E103': 'Connection Recovery Failed',
    'E104': 'Serial Connection Failed',
    'E105': 'WiFi Connection Failed',
    
    # Robot state errors (E2xx)
    'E201': 'Robot Not in Running Mode',
    'E202': 'Protective Stop Active',
    'E203': 'Emergency Stop Active',
    'E204': 'Robot in Backdrive Mode',
    'E205': 'Safety System Error',
    
    # Motion safety errors (E3xx)
    'E301': 'Joint Limit Violation',
    'E302': 'Singularity Detected',
    'E303': 'Workspace Boundary Violation',
    'E304': 'Inverse Kinematics Failed',
    'E305': 'Velocity Limit Exceeded',
    
    # Data errors (E4xx)
    'E401': 'Invalid Pose Data',
    'E402': 'Invalid Joint Configuration',
    
    # IMU errors (E5xx)
    'E501': 'IMU Communication Error',
    'E502': 'IMU Data Parse Error',
    'E503': 'IMU Calibration Error',
}

# Robot mode names for diagnostics
ROBOT_MODE_NAMES = {
    -1: "Disconnected", 
    0: "Confirm Safety", 
    1: "Booting",
    2: "Power Off", 
    3: "Power On", 
    4: "Idle", 
    5: "Backdrive",
    6: "Running", 
    7: "Running", 
    8: "Updating Firmware"
}

# Safety mode names for diagnostics
SAFETY_MODE_NAMES = {
    0: "Undefined", 
    1: "Normal", 
    2: "Reduced",
    3: "Protective Stop", 
    4: "Recovery", 
    5: "Safeguard Stop",
    6: "System Emergency Stop", 
    7: "Robot Emergency Stop",
    8: "Violation", 
    9: "Fault"
}