"""
IMU calibration system
Handles calibration and zero-point offset management
"""

import math
import numpy as np
from core.math_utils import euler_to_quat, quaternion_multiply, quaternion_inverse

class IMUCalibration:
    """
    Handles IMU calibration and zero-point offset management
    Provides an interactive calibration wizard for accurate zero-point setup
    """
    
    def __init__(self, config_manager):
        """
        Initialize IMU calibration system
        
        Args:
            config_manager: ConfigManager instance for persistent storage
        """
        self.config_manager = config_manager
        self.zero_offsets = {
            'roll': config_manager.get('imu_calibration', 'roll_offset', 0.0),
            'pitch': config_manager.get('imu_calibration', 'pitch_offset', 0.0),
            'yaw': config_manager.get('imu_calibration', 'yaw_offset', 0.0)
        }
        self.is_calibrated = config_manager.get('imu_calibration', 'is_calibrated', False)
        
        # Pre-calculate calibration quaternion (inverse of the zero offset position)
        if self.is_calibrated:
            # Convert degree offsets to radians
            r = math.radians(self.zero_offsets['roll'])
            p = math.radians(self.zero_offsets['pitch'])
            y = math.radians(self.zero_offsets['yaw'])
            # Create quaternion representing the offset position
            self.zero_quat = euler_to_quat(r, p, y)
            # We will multiply by the inverse to "subtract" this rotation
            self.zero_quat_inv = quaternion_inverse(self.zero_quat)
        else:
            self.zero_quat = np.array([0.0, 0.0, 0.0, 1.0])
            self.zero_quat_inv = np.array([0.0, 0.0, 0.0, 1.0])
        
        # Calibration wizard state
        self.calibration_active = False
        self.calibration_step = 0
        self.calibration_samples = []
        self.samples_per_step = 100  # Number of samples to average
    
    def apply_calibration(self, roll, pitch, yaw):
        """
        Apply calibration offsets to raw IMU data
        
        Args:
            roll, pitch, yaw: Raw IMU angles in degrees
        
        Returns:
            Tuple of (calibrated_roll, calibrated_pitch, calibrated_yaw)
        """
        if self.is_calibrated:
            roll -= self.zero_offsets['roll']
            pitch -= self.zero_offsets['pitch']
            yaw -= self.zero_offsets['yaw']
        return roll, pitch, yaw

    def apply_calibration_quat(self, q):
        """
        Apply calibration to a raw quaternion.
        Rotates the raw quaternion by the inverse of the zero-offset rotation.
        
        Args:
            q: Raw quaternion [i, j, k, w]
        
        Returns:
            Calibrated quaternion
        """
        if self.is_calibrated:
            # Result = ZeroInv * Raw
            return quaternion_multiply(self.zero_quat_inv, q)
        return q
    
    def start_calibration_wizard(self):
        """Start interactive calibration process"""
        print("\n" + "="*70)
        print("IMU CALIBRATION WIZARD")
        print("="*70)
        print("Step 1: Place IMU in neutral position")
        print("        (flat surface, facing forward)")
        print("        Hold still and press SPACE to capture zero position...")
        print("="*70 + "\n")
        
        self.calibration_active = True
        self.calibration_step = 0
        self.calibration_samples = []
    
    def capture_calibration_sample(self, roll, pitch, yaw):
        """
        Capture calibration data during wizard
        
        Args:
            roll, pitch, yaw: Current IMU angles in degrees
        """
        if not self.calibration_active:
            return
        
        if self.calibration_step == 0:
            # Capturing zero position
            self.calibration_samples.append((roll, pitch, yaw))
            
            # Show progress
            if len(self.calibration_samples) % 20 == 0:
                progress = (len(self.calibration_samples) / self.samples_per_step) * 100
                print(f"Capturing... {progress:.0f}%")
            
            if len(self.calibration_samples) >= self.samples_per_step:
                # Calculate average offsets
                avg_roll = sum(s[0] for s in self.calibration_samples) / len(self.calibration_samples)
                avg_pitch = sum(s[1] for s in self.calibration_samples) / len(self.calibration_samples)
                avg_yaw = sum(s[2] for s in self.calibration_samples) / len(self.calibration_samples)
                
                # Calculate standard deviation to check stability
                std_roll = math.sqrt(sum((s[0] - avg_roll)**2 for s in self.calibration_samples) / len(self.calibration_samples))
                std_pitch = math.sqrt(sum((s[1] - avg_pitch)**2 for s in self.calibration_samples) / len(self.calibration_samples))
                std_yaw = math.sqrt(sum((s[2] - avg_yaw)**2 for s in self.calibration_samples) / len(self.calibration_samples))
                
                self.zero_offsets = {
                    'roll': avg_roll,
                    'pitch': avg_pitch,
                    'yaw': avg_yaw
                }
                
                print(f"\nZero position captured:")
                print(f"  Roll offset: {avg_roll:.2f}° (std: {std_roll:.2f}°)")
                print(f"  Pitch offset: {avg_pitch:.2f}° (std: {std_pitch:.2f}°)")
                print(f"  Yaw offset: {avg_yaw:.2f}° (std: {std_yaw:.2f}°)")
                
                if std_roll > 2.0 or std_pitch > 2.0 or std_yaw > 2.0:
                    print("\nWARNING: High variation detected. IMU may have been moving.")
                    print("         Consider recalibrating for better accuracy.")
                
                print("\nStep 2: Press SPACE to complete calibration...")
                
                self.calibration_step = 1
                self.calibration_samples = []
    
    def complete_calibration(self):
        """Finalize calibration and save to config"""
        self.is_calibrated = True
        self.calibration_active = False
        
        # Update zero quaternion
        r = math.radians(self.zero_offsets['roll'])
        p = math.radians(self.zero_offsets['pitch'])
        y = math.radians(self.zero_offsets['yaw'])
        self.zero_quat = euler_to_quat(r, p, y)
        self.zero_quat_inv = quaternion_inverse(self.zero_quat)
        
        # Save to config
        self.config_manager.set(self.zero_offsets['roll'], 'imu_calibration', 'roll_offset')
        self.config_manager.set(self.zero_offsets['pitch'], 'imu_calibration', 'pitch_offset')
        self.config_manager.set(self.zero_offsets['yaw'], 'imu_calibration', 'yaw_offset')
        self.config_manager.set(True, 'imu_calibration', 'is_calibrated')
        self.config_manager.save_config()
        
        print("\n" + "="*70)
        print("CALIBRATION COMPLETE")
        print("="*70)
        print("IMU is now calibrated. Offsets saved to configuration file.")
        print("="*70 + "\n")
    
    def reset_calibration(self):
        """Reset calibration to defaults"""
        self.zero_offsets = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.is_calibrated = False
        self.calibration_active = False
        self.zero_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.zero_quat_inv = np.array([0.0, 0.0, 0.0, 1.0])
        
        self.config_manager.set(0.0, 'imu_calibration', 'roll_offset')
        self.config_manager.set(0.0, 'imu_calibration', 'pitch_offset')
        self.config_manager.set(0.0, 'imu_calibration', 'yaw_offset')
        self.config_manager.set(False, 'imu_calibration', 'is_calibrated')
        self.config_manager.save_config()
        
        print("IMU calibration reset to defaults")
    
    def get_status(self):
        """Get calibration status string"""
        if self.is_calibrated:
            return "Calibrated"
        else:
            return "Not Calibrated"