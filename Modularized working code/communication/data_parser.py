"""
IMU data parsing and validation
Parses ESP32 data packets into usable format

UPDATED: Now handles 9-value CSV format including flex sensor data
Format: relQI,relQJ,relQK,relQR,relRoll,relPitch,relYaw,mode,flex
"""

import numpy as np
import math
from core.error_handler import RobotError

class IMUDataParser:
    """
    Parses and validates IMU data packets from ESP32
    Now includes flex sensor data (9th value)
    """
    
    @staticmethod
    def parse_line(line, log_file=None):
        """
        Parse IMU data line into structured format
        
        Expected format: relQI,relQJ,relQK,relQR,relRoll,relPitch,relYaw,mode,flex
        
        Args:
            line: Raw data string from IMU
            log_file: Optional log file for error logging
        
        Returns:
            Dictionary with parsed data or None if invalid:
            {
                'quaternion': [qi, qj, qk, qr],
                'euler': [roll, pitch, yaw],
                'mode': int,
                'flex': int (0-100 percentage)
            }
        """
        # Skip comment lines and empty lines
        if not line or line.startswith('#') or ',' not in line:
            return None
        
        try:
            parts = line.split(',')
            if len(parts) != 9:  # CHANGED: 8 -> 9 (added flex)
                if log_file:
                    RobotError.log_error(log_file, 'E502',
                        details=f"Expected 9 values, got {len(parts)}",
                        context=f"Line: {line}")
                return None
            
            # Parse floats and ints
            float_parts = [float(p) for p in parts[:7]]
            mode_part = int(parts[7])
            flex_part = int(parts[8])  # NEW: 9th value is flex sensor percentage
            
            # Validate finite values
            if not all(math.isfinite(x) for x in float_parts):
                if log_file:
                    RobotError.log_error(log_file, 'E502',
                        details="NaN or Inf values detected",
                        context=f"Line: {line}")
                return None
            
            # Extract components
            rel_qi, rel_qj, rel_qk, rel_qr = float_parts[:4]
            rel_roll, rel_pitch, rel_yaw = float_parts[4:7]
            
            # Validate flex sensor range (0-100)
            if not (0 <= flex_part <= 100):
                if log_file:
                    RobotError.log_error(log_file, 'E604',
                        details=f"Flex value out of range: {flex_part}",
                        context=f"Expected 0-100, got {flex_part}")
                # Clamp to valid range instead of rejecting
                flex_part = max(0, min(100, flex_part))
            
            return {
                'quaternion': np.array([rel_qi, rel_qj, rel_qk, rel_qr]),
                'euler': np.array([rel_roll, rel_pitch, rel_yaw]),
                'mode': mode_part,
                'flex': flex_part  # NEW: flex sensor percentage
            }
            
        except (ValueError, IndexError) as e:
            if log_file:
                RobotError.log_error(log_file, 'E502',
                    details=f"Parse error: {str(e)}",
                    context=f"Line: {line}")
            return None
    
    @staticmethod
    def validate_quaternion(quat):
        """
        Validate quaternion data
        
        Args:
            quat: Quaternion array [qi, qj, qk, qr]
        
        Returns:
            True if valid, False otherwise
        """
        if len(quat) != 4:
            return False
        
        if not all(math.isfinite(x) for x in quat):
            return False
        
        # Check if roughly unit length (allow some tolerance)
        norm = np.linalg.norm(quat)
        return abs(norm - 1.0) < 0.2  # Allow 20% tolerance for transmission errors
    
    @staticmethod
    def validate_euler(euler):
        """
        Validate Euler angle data
        
        Args:
            euler: Euler angles array [roll, pitch, yaw] in degrees
        
        Returns:
            True if valid, False otherwise
        """
        if len(euler) != 3:
            return False
        
        if not all(math.isfinite(x) for x in euler):
            return False
        
        # Check reasonable ranges (±180 degrees)
        return all(abs(x) < 360.0 for x in euler)
    
    @staticmethod
    def validate_flex(flex_value):
        """
        Validate flex sensor data
        
        Args:
            flex_value: Flex sensor percentage (0-100)
        
        Returns:
            True if valid, False otherwise
        """
        if not isinstance(flex_value, (int, float)):
            return False
        
        if not math.isfinite(flex_value):
            return False
        
        # Check range 0-100
        return 0 <= flex_value <= 100