"""
Configuration constants for UR5e Robot RTDE + IMU Control System
All hardware settings, limits, and control parameters

OPTIMIZED FOR UR5e:
- UR5e max reach: 850mm (0.85m)
- UR5e max speed: 1.0 m/s (much faster than UR10!)
- UR5e max acceleration: 5.0 m/s² (much faster!)
- Updated workspace limits for UR5e
- Faster control parameters taking advantage of UR5e speed

ADDED: Hand-E Gripper Integration
- Gripper control via flex sensor input
- Socket communication on port 63352
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
# UNIVERSAL ROBOT UR5e SETTINGS
#==============================================================================

UR_ROBOT_IP = "192.168.1.191"
UR_ENABLED = True  # Set True to enable robot control
UR_SIMULATE = False  # Set False for real robot commands

# UR5e Joint Limits (radians)
UR_JOINT_LIMITS = {
    'joint_0': (-2*math.pi, 2*math.pi),      # Base: ±360°
    'joint_1': (-2*math.pi, 2*math.pi),      # Shoulder: ±360°
    'joint_2': (-2*math.pi, 2*math.pi),      # Elbow: ±360°
    'joint_3': (-2*math.pi, 2*math.pi),      # Wrist 1: ±360°
    'joint_4': (-2*math.pi, 2*math.pi),      # Wrist 2: ±360°
    'joint_5': (-2*math.pi, 2*math.pi),      # Wrist 3: ±360°
}

# UR5e Workspace limits (meters, relative to robot base)
# UR5e reach: 850mm, but we use safety margins
UR_LIMITS = {
    'x_min': -0.80, 'x_max': 0.80,
    'y_min': -0.80, 'y_max': 0.80,
    'z_min': 0.00, 'z_max': 1.0      # UR5e can go lower and slightly higher
}

# UR5e Control parameters
UR_BASE_POSITION = [0.4, 0.0, 0.5]  # Starting TCP position [x, y, z] in meters
UR_BASE_ORIENTATION = [0.0, 0.0, 0.0]  # Starting orientation [rx, ry, rz] rotation vector

# Neutral "Home" Position (Bent arm, safe for restart)
# [Base, Shoulder, Elbow, Wrist1, Wrist2, Wrist3] in radians
UR_NEUTRAL_JOINT_POSITIONS = [0, -1.57, -1.57, -1.57, 1.57, 0]

# UR5e Performance Specifications (e-Series is MUCH faster!)
UR_MAX_VELOCITY = 1.0           # CHANGED: 0.25 -> 1.0 m/s (UR5e max speed!)
UR_MAX_ACCELERATION = 5.0       # CHANGED: 1.2 -> 5.0 m/s² (UR5e max accel!)
UR_JOINT_VELOCITY = 3.14        # CHANGED: 1.05 -> 3.14 rad/s (180°/s)
UR_JOINT_ACCELERATION = 6.28    # CHANGED: 1.4 -> 6.28 rad/s² (360°/s²)

#==============================================================================
# HAND-E GRIPPER SETTINGS
#==============================================================================

# Connection parameters
GRIPPER_ENABLED = True           # Enable gripper control
GRIPPER_PORT = 63352             # Socket port for Hand-E gripper
GRIPPER_TIMEOUT = 2.0            # Socket timeout in seconds
GRIPPER_RECONNECT_ATTEMPTS = 3   # Max reconnection attempts
GRIPPER_RECONNECT_COOLDOWN = 2.0 # Seconds between reconnect attempts

# Control parameters
GRIPPER_MIN_POSITION = 0         # Fully open (0-255 range)
GRIPPER_MAX_POSITION = 255       # Fully closed (0-255 range)
GRIPPER_DEFAULT_SPEED = 255      # Default speed (0-255)
GRIPPER_DEFAULT_FORCE = 150      # Default force (0-255)
GRIPPER_MIN_SPEED = 50           # Minimum allowed speed
GRIPPER_MAX_SPEED = 255          # Maximum allowed speed
GRIPPER_MIN_FORCE = 20           # Minimum allowed force
GRIPPER_MAX_FORCE = 255          # Maximum allowed force

# Flex sensor filtering
FLEX_DEADZONE = 3                # Percent - ignore changes smaller than this
FLEX_SMOOTH_WINDOW = 5           # Moving average window size (Arduino-side)

# Command rate limiting
GRIPPER_UPDATE_INTERVAL = 0.05   # Minimum seconds between gripper commands (20 Hz)

#==============================================================================
# CONTROL PARAMETERS - OPTIMIZED FOR UR5e HIGH PERFORMANCE
#==============================================================================

CONTROL_PARAMS = {
    'base_translation': 0.010,      # REDUCED: 0.020 -> 0.010 (User requested 50% sensitivity)
    'base_rotation': 0.02,          
    'vertical': 0.006,              
    'tcp_translation': 0.0025,      # REDUCED: 0.005 -> 0.0025 (50% sensitivity)
    'tcp_vertical': 0.005,          
    'tcp_orientation': 0.030,       
    'robot_orientation': 0.05,
    'wrist_rotation': 0.02,         # NEW: For wrist joint control (Modes 4 & 5)
}

# Movement filtering parameters
MOVEMENT_DEADZONE = 5.0  # degrees - no movement below this
DEADZONE_RAMP_WIDTH = 5.0  # degrees - gradual ramp zone (5-10 deg)
VELOCITY_DECAY = 0.95  # visualization smoothing

#==============================================================================
# IMU AXIS MAPPING (DEPRECATED - Now handled by core/coordinate_frames.py)
# 
# This configuration is kept for reference only.
# The new universal coordinate transform system in core/coordinate_frames.py
# now handles all axis mapping transparently.
#
# See core/coordinate_frames.py for the new mapping system.
#==============================================================================

IMU_AXIS_MAPPING = {
    # DEPRECATED: These values are no longer used by the system
    # Kept for documentation purposes only
    'x_axis': 'roll',
    'x_invert': True,
    'y_axis': 'pitch',
    'y_invert': False,
    'z_axis': 'pitch',
    'z_invert': False,
    'rx_axis': 'pitch',
    'rx_invert': False,
    'ry_axis': 'yaw',
    'ry_invert': True,
    'rz_axis': 'roll',
    'rz_invert': False,
}

#==============================================================================
# CONTROL MODES - REDESIGNED FOR CRANE + WRIST CONTROL
#==============================================================================

CONTROL_MODES = {
    0: "IDLE",
    1: "CRANE_MODE",           # Forward/back translation + base rotation
    2: "VERTICAL_Z",           # Pure vertical movement
    3: "LATERAL_PRECISE",      # Precise left/right translation
    4: "WRIST_ORIENT",         # Wrist 1 & 2 orientation control
    5: "WRIST_SCREW",          # Wrist 3 rotation (screwdriver motion)
    6: "TCP_ORIENT_MIMIC"      # Absolute orientation mimic mode
}

#==============================================================================
# SAFETY THRESHOLDS
#==============================================================================

# Singularity thresholds (radians) - UR5e has better singularity handling
WRIST_SINGULARITY_THRESHOLD = 0.10   # CHANGED: 0.15 -> 0.10 (UR5e can get closer)
SHOULDER_SINGULARITY_THRESHOLD = 0.10
ELBOW_SINGULARITY_THRESHOLD = 0.08   # CHANGED: 0.10 -> 0.08

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
    
    # Gripper errors (E6xx)
    'E601': 'Gripper Connection Error',
    'E602': 'Gripper Activation Failed',
    'E603': 'Gripper Command Timeout',
    'E604': 'Flex Sensor Error',
    'E605': 'Gripper Position Error',
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