"""
IMU calibration system
Handles calibration and zero-point offset management

CHANGES:
- Improved calibration wizard with progress tracking
- Automatic stability detection
- Better visual feedback
- Quality rating system
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
        
        # IMPROVED: Calibration wizard state
        self.calibration_active = False
        self.calibration_step = 0
        self.calibration_samples = []
        self.samples_per_step = 50  # CHANGED: From 100 to 50 (faster)
        self.stability_threshold = 0.5  # ADDED: Max deviation for "stable" reading
    
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
        print("\nSTEP 1: Position Setup")
        print("  1. Place glove on a FLAT surface")
        print("  2. Orient fingers POINTING AWAY from you")
        print("  3. Ensure glove is COMPLETELY STILL")
        print("\nPress SPACE when ready to capture zero position...")
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
            # Waiting for user to press SPACE
            return
        
        elif self.calibration_step == 1:
            # Collecting samples
            self.calibration_samples.append((roll, pitch, yaw))
            
            # ADDED: Check stability every 10 samples
            if len(self.calibration_samples) > 10:
                recent_samples = self.calibration_samples[-10:]
                rolls = [s[0] for s in recent_samples]
                pitches = [s[1] for s in recent_samples]
                yaws = [s[2] for s in recent_samples]
                
                max_std = max(np.std(rolls), np.std(pitches), np.std(yaws))
                
                if max_std > self.stability_threshold:
                    # Show warning but continue
                    if len(self.calibration_samples) % 10 == 0:
                        print(f"⚠️  UNSTABLE! Movement detected: {max_std:.2f}° (Keep glove still!)")
            
            # Show progress
            progress = (len(self.calibration_samples) / self.samples_per_step) * 100
            if len(self.calibration_samples) % 10 == 0:
                print(f"Collecting samples: {len(self.calibration_samples)}/{self.samples_per_step} ({progress:.0f}%)")
            
            if len(self.calibration_samples) >= self.samples_per_step:
                # Automatically complete when enough samples collected
                self.complete_calibration()
    
    def complete_calibration(self):
        """Finalize calibration and compute offsets"""
        if len(self.calibration_samples) == 0:
            print("ERROR: No samples collected!")
            self.calibration_active = False
            return
        
        # Calculate average offsets
        avg_roll = sum(s[0] for s in self.calibration_samples) / len(self.calibration_samples)
        avg_pitch = sum(s[1] for s in self.calibration_samples) / len(self.calibration_samples)
        avg_yaw = sum(s[2] for s in self.calibration_samples) / len(self.calibration_samples)
        
        # Calculate standard deviations (stability check)
        std_roll = math.sqrt(sum((s[0] - avg_roll)**2 for s in self.calibration_samples) / len(self.calibration_samples))
        std_pitch = math.sqrt(sum((s[1] - avg_pitch)**2 for s in self.calibration_samples) / len(self.calibration_samples))
        std_yaw = math.sqrt(sum((s[2] - avg_yaw)**2 for s in self.calibration_samples) / len(self.calibration_samples))
        
        max_std = max(std_roll, std_pitch, std_yaw)
        
        self.zero_offsets = {
            'roll': avg_roll,
            'pitch': avg_pitch,
            'yaw': avg_yaw
        }
        
        # Update zero quaternion
        r = math.radians(self.zero_offsets['roll'])
        p = math.radians(self.zero_offsets['pitch'])
        y = math.radians(self.zero_offsets['yaw'])
        self.zero_quat = euler_to_quat(r, p, y)
        self.zero_quat_inv = quaternion_inverse(self.zero_quat)
        
        # Mark as calibrated
        self.is_calibrated = True
        self.calibration_active = False
        
        # Save to config
        self.config_manager.set(self.zero_offsets['roll'], 'imu_calibration', 'roll_offset')
        self.config_manager.set(self.zero_offsets['pitch'], 'imu_calibration', 'pitch_offset')
        self.config_manager.set(self.zero_offsets['yaw'], 'imu_calibration', 'yaw_offset')
        self.config_manager.set(True, 'imu_calibration', 'is_calibrated')
        self.config_manager.save_config()
        
        # Determine quality rating
        if max_std < 0.3:
            quality = "EXCELLENT"
        elif max_std < 0.5:
            quality = "GOOD"
        else:
            quality = "ACCEPTABLE"
        
        print("\n" + "="*70)
        print("✅ CALIBRATION COMPLETE!")
        print("="*70)
        print("\nZero Position Offsets:")
        print(f"  Roll:  {avg_roll:+7.2f}° (±{std_roll:.2f}°)")
        print(f"  Pitch: {avg_pitch:+7.2f}° (±{std_pitch:.2f}°)")
        print(f"  Yaw:   {avg_yaw:+7.2f}° (±{std_yaw:.2f}°)")
        print(f"\nStability Quality: {quality}")
        
        if max_std > 1.0:
            print("\n⚠️  WARNING: High variance detected!")
            print("    Glove may have been moving during calibration.")
            print("    Consider recalibrating (Shift+C) for better accuracy.")
        
        print("\nCalibration saved to robot_config.json")
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
        if self.calibration_active:
            if self.calibration_step == 0:
                return "CALIBRATING: Press SPACE to start"
            else:
                progress = (len(self.calibration_samples) / self.samples_per_step) * 100
                return f"CALIBRATING: {len(self.calibration_samples)}/{self.samples_per_step} ({progress:.0f}%)"
        
        if self.is_calibrated:
            return f"Calibrated (R:{self.zero_offsets['roll']:+.1f}° P:{self.zero_offsets['pitch']:+.1f}° Y:{self.zero_offsets['yaw']:+.1f}°)"
        else:
            return "Not Calibrated (Press Shift+C)"