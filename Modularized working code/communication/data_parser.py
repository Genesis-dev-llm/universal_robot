"""
IMU data parsing and validation
Parses ESP32 data packets into usable format
"""

import numpy as np
import math
from core.error_handler import RobotError

class IMUDataParser:
    """
    Parses and validates IMU data packets from ESP32
    """
    
    @staticmethod
    def parse_line(line, log_file=None):
        """
        Parse IMU data line into structured format
        
        Expected format: relQI,relQJ,relQK,relQR,relRoll,relPitch,relYaw,mode
        
        Args:
            line: Raw data string from IMU
            log_file: Optional log file for error logging
        
        Returns:
            Dictionary with parsed data or None if invalid:
            {
                'quaternion': [qi, qj, qk, qr],
                'euler': [roll, pitch, yaw],
                'mode': int
            }
        """
        # Skip comment lines and empty lines
        if not line or line.startswith('#') or ',' not in line:
            return None
        
        try:
            parts = line.split(',')
            if len(parts) != 8:
                if log_file:
                    RobotError.log_error(log_file, 'E502',
                        details=f"Expected 8 values, got {len(parts)}",
                        context=f"Line: {line}")
                return None
            
            # Parse floats
            float_parts = [float(p) for p in parts[:-1]]
            mode_part = int(parts[-1])
            
            # Validate finite values
            if not all(math.isfinite(x) for x in float_parts):
                if log_file:
                    RobotError.log_error(log_file, 'E502',
                        details="NaN or Inf values detected",
                        context=f"Line: {line}")
                return None
            
            # Validate mode range
            if not (0 <= mode_part <= 6):
                if log_file:
                    RobotError.log_error(log_file, 'E502',
                        details=f"Mode {mode_part} out of range [0-6]",
                        context=f"Line: {line}")
                return None
            
            # Extract components
            rel_qi, rel_qj, rel_qk, rel_qr = float_parts[:4]
            rel_roll, rel_pitch, rel_yaw = float_parts[4:7]
            
            return {
                'quaternion': np.array([rel_qi, rel_qj, rel_qk, rel_qr]),
                'euler': np.array([rel_roll, rel_pitch, rel_yaw]),
                'mode': mode_part
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