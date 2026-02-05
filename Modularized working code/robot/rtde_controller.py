"""
RTDE Robot Controller - Cleaned Up Version
CHANGES:
- Removed singularity checks (UR handles internally)
- Removed joint limit checks (IK solver handles)
- Cleaned up redundant validations
- Streamlined command execution paths
- Added velocity data validation
"""

import time
import math
from datetime import datetime
from collections import deque

from core.error_handler import RobotError
from core.math_utils import clamp_position, rotation_vector_add, calculate_angular_velocity
from robot.safety_checker import SafetyChecker
from robot.gripper_controller import GripperController
from config.constants import (
    UR_BASE_POSITION, UR_BASE_ORIENTATION,
    UR_MAX_VELOCITY, UR_MAX_ACCELERATION,
    UR_JOINT_VELOCITY, UR_JOINT_ACCELERATION,
    RTDE_MAX_FREQUENCY, GRIPPER_ENABLED
)

# Try to import RTDE
try:
    import rtde_control
    import rtde_receive
    RTDE_AVAILABLE = True
except ImportError:
    RTDE_AVAILABLE = False
    print("WARNING: ur-rtde not installed. Robot control disabled.")


class RTDEController:
    """
    Universal Robot RTDE control interface
    Simplified safety model - relies on UR's internal safety systems
    """
    
    def __init__(self, robot_ip, enabled=False, simulate=True, config_mgr=None):
        """
        Initialize RTDE controller
        
        Args:
            robot_ip: Robot IP address
            enabled: Enable robot control
            simulate: Simulation mode (no real commands)
            config_mgr: Configuration manager
        """
        self.robot_ip = robot_ip
        self.enabled = enabled and RTDE_AVAILABLE
        self.simulate = simulate
        self.config_mgr = config_mgr
        
        # RTDE interfaces
        self.rtde_c = None
        self.rtde_r = None
        
        # Safety checker (simplified)
        self.safety_checker = SafetyChecker(log_file=None)
        
        # Gripper controller
        self.gripper = None
        self.gripper_enabled = (GRIPPER_ENABLED and 
            config_mgr.get('gripper', 'enabled', True) if config_mgr else GRIPPER_ENABLED)
        
        # Control state
        self.command_queue = deque(maxlen=100)
        self.last_command_time = 0
        
        # Command thresholding (prevent jittery updates)
        self.last_sent_pose = None
        self.last_sent_velocity = None
        self.last_sent_joint_velocity = None
        self.POSE_DELTA_THRESHOLD = 0.0005  # 0.5mm
        self.ORIENT_DELTA_THRESHOLD = 0.005 # ~0.3 degrees
        self.VEL_DELTA_THRESHOLD = 0.001    # 1mm/s or 0.001 rad/s
        self.JOINT_VEL_DELTA_THRESHOLD = 0.001  # 0.001 rad/s for joints
        
        # Smoothing parameters
        self.smoothing_factor = (config_mgr.get('safety', 'smoothing_factor', 0.15) 
            if config_mgr else 0.15)
        self.smoothed_roll = None
        self.smoothed_pitch = None
        self.smoothed_yaw = None
        
        # Velocity ramping
        self.current_velocity_scale = 0.0
        self.target_velocity_scale = 1.0
        self.velocity_ramp_rate = (config_mgr.get('safety', 'velocity_ramp_rate', 0.1) 
            if config_mgr else 0.1)
        
        # Robot state
        self.current_tcp_pose = UR_BASE_POSITION + UR_BASE_ORIENTATION
        self.target_tcp_pose = UR_BASE_POSITION + UR_BASE_ORIENTATION
        self.last_joint_positions = [0, -1.57, 1.57, -1.57, -1.57, 0]
        self.robot_status_text = "Disconnected"
        self.last_state_update = 0
        self.state_update_interval = 0.1
        
        # Connection recovery
        self.connection_lost = False
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 5
        self.last_reconnect_attempt = 0
        self.reconnect_cooldown = 2.0
        
        # Statistics
        self.total_commands_sent = 0
        self.successful_commands = 0
        self.failed_commands = 0
        
        # Initialize log file
        import os
        os.makedirs("logs", exist_ok=True)
        log_filename = "logs/latest_session.log"
        try:
            self.log_file = open(log_filename, "w")
            self.log_file.write(f"# RTDE Control Log - {datetime.now()}\n")
            self.log_file.write(f"# Mode: {'SIMULATE' if simulate else 'REAL'}\n")
            self.log_file.write(f"# Robot IP: {robot_ip}\n")
            self.log_file.write(f"# Gripper: {'ENABLED' if self.gripper_enabled else 'DISABLED'}\n\n")
            print(f"Logging to: {log_filename}")
            
            self.safety_checker.log_file = self.log_file
        except Exception as e:
            print(f"Warning: Could not create log file: {e}")
            self.log_file = None
        
        if self.enabled and not self.simulate:
            self.connect()
    
    def connect(self):
        """Connect to UR robot via RTDE and gripper"""
        try:
            print(f"Connecting to UR robot at {self.robot_ip}...")
            self.rtde_c = rtde_control.RTDEControlInterface(self.robot_ip)
            self.rtde_r = rtde_receive.RTDEReceiveInterface(self.robot_ip)
            
            self.safety_checker.set_rtde_receive(self.rtde_r)
            
            # Get current robot state
            current_pose = self.rtde_r.getActualTCPPose()
            if current_pose:
                self.current_tcp_pose = current_pose
                self.target_tcp_pose = current_pose[:]
                print(f"Current TCP: [{current_pose[0]:.3f}, {current_pose[1]:.3f}, {current_pose[2]:.3f}]")
            
            print(f"RTDE connected successfully")
            self.log_command(f"# Connected to robot at {self.robot_ip}")
            
            self.connection_lost = False
            self.reconnect_attempts = 0
            
            # Initialize gripper if enabled
            if self.gripper_enabled:
                self._initialize_gripper()
            
            return True
            
        except RuntimeError as e:
            print(f"\n[WARNING] Could not connect to robot at {self.robot_ip}")
            print(f"Reason: {e}")
            print(">> FALLING BACK TO SIMULATION MODE (Visualization Only) <<\n")
            
            self.simulate = True
            self.enabled = False
            self.rtde_c = None
            self.rtde_r = None
            
            self.log_command(f"# Connection failed: {e}. Fallback to SIMULATION.")
            return False
            
        except Exception as e:
            error_msg = RobotError.format_error('E102', str(e), f"IP: {self.robot_ip}")
            print(error_msg)
            RobotError.log_error(self.log_file, 'E102', str(e), f"IP: {self.robot_ip}")
            self.rtde_c = None
            self.rtde_r = None
            self.connection_lost = True
            return False
    
    def _initialize_gripper(self):
        """Initialize gripper controller"""
        try:
            print("Initializing Hand-E gripper...")
            self.gripper = GripperController(self.robot_ip, self.config_mgr, self.log_file)
            
            if self.gripper.connect():
                if self.gripper.activate():
                    self.gripper.start_worker_thread()
                    print("✓ Gripper ready")
                else:
                    print("✗ Gripper activation failed")
                    self.gripper = None
            else:
                print("✗ Gripper connection failed - continuing without gripper")
                self.gripper = None
                
        except Exception as e:
            print(f"✗ Gripper initialization error: {e}")
            self.gripper = None
    
    def attempt_reconnect(self):
        """Attempt reconnection after connection loss"""
        current_time = time.time()
        
        if current_time - self.last_reconnect_attempt < self.reconnect_cooldown:
            return False
        
        if self.reconnect_attempts >= self.max_reconnect_attempts:
            error_msg = RobotError.format_error('E103',
                f"Maximum attempts ({self.max_reconnect_attempts}) reached",
                "Connection recovery failed")
            print(error_msg)
            RobotError.log_error(self.log_file, 'E103',
                f"Max attempts: {self.max_reconnect_attempts}", "Recovery failed")
            return False
        
        self.last_reconnect_attempt = current_time
        self.reconnect_attempts += 1
        
        print(f"Reconnecting ({self.reconnect_attempts}/{self.max_reconnect_attempts})...")
        self.log_command(f"# Reconnection attempt {self.reconnect_attempts}")
        
        # Close existing connections
        try:
            if self.rtde_c:
                self.rtde_c.disconnect()
            if self.rtde_r:
                self.rtde_r.disconnect()
        except:
            pass
        
        return self.connect()
    
    def log_command(self, command):
        """Log command with timestamp"""
        if self.log_file and not self.log_file.closed:
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            self.log_file.write(f"[{timestamp}] {command}\n")
            self.log_file.flush()
    
    def update_robot_status(self):
        """Update robot status periodically"""
        current_time = time.time()
        if current_time - self.last_state_update < self.state_update_interval:
            return
        
        self.last_state_update = current_time
        
        if not self.enabled or not self.rtde_r:
            self.robot_status_text = "Robot: DISABLED"
            return
        
        try:
            # Get current TCP pose
            tcp_pose = self.rtde_r.getActualTCPPose()
            if tcp_pose:
                self.current_tcp_pose = tcp_pose
            
            # Get joint positions
            joint_pos = self.rtde_r.getActualQ()
            if joint_pos:
                self.last_joint_positions = joint_pos
            
            # Check status
            robot_mode = self.rtde_r.getRobotMode()
            safety_mode = self.rtde_r.getSafetyMode()
            
            if robot_mode == 7 and safety_mode == 1:
                self.robot_status_text = f"Robot: READY | TCP: [{tcp_pose[0]:.3f}, {tcp_pose[1]:.3f}, {tcp_pose[2]:.3f}]"
            else:
                self.robot_status_text = f"Robot: NOT READY (Mode: {robot_mode}, Safety: {safety_mode})"
                
        except Exception as e:
            self.robot_status_text = f"Robot: ERROR - {str(e)[:50]}"
            self.connection_lost = True
    
    def apply_smoothing(self, roll, pitch, yaw):
        """
        Apply exponential smoothing (Low Pass Filter)
        smoothed = alpha * new + (1 - alpha) * old
        """
        if self.smoothed_roll is None:
            self.smoothed_roll = roll
            self.smoothed_pitch = pitch
            self.smoothed_yaw = yaw
            return roll, pitch, yaw
            
        alpha = self.smoothing_factor
        
        self.smoothed_roll = (1 - alpha) * self.smoothed_roll + alpha * roll
        self.smoothed_pitch = (1 - alpha) * self.smoothed_pitch + alpha * pitch
        self.smoothed_yaw = (1 - alpha) * self.smoothed_yaw + alpha * yaw
        
        return self.smoothed_roll, self.smoothed_pitch, self.smoothed_yaw
    
    def calculate_velocity_scale(self, roll, pitch, yaw):
        """Scale velocity based on IMU movement speed"""
        angular_velocity = calculate_angular_velocity(roll, pitch, yaw)
        
        # Slow down if moving IMU too fast
        if angular_velocity > 30.0:
            scale = 30.0 / angular_velocity
            target_scale = max(0.1, min(1.0, scale))
        else:
            target_scale = 1.0
        
        # Apply smooth ramping
        if target_scale > self.current_velocity_scale:
            self.current_velocity_scale = min(target_scale,
                self.current_velocity_scale + self.velocity_ramp_rate)
        else:
            self.current_velocity_scale = max(target_scale,
                self.current_velocity_scale - self.velocity_ramp_rate)
        
        return self.current_velocity_scale
    
    def move_speed(self, linear_velocity, angular_velocity, acceleration=0.5, mode_name=None):
        """
        Execute velocity control command (speedL)
        
        Args:
            linear_velocity: [vx, vy, vz] in m/s
            angular_velocity: [rx, ry, rz] in rad/s
            acceleration: Acceleration in m/s^2
            mode_name: Current control mode name (for logging)
        
        Returns:
            True if successful, False otherwise
        """
        current_time = time.time()
        
        # Rate limiting
        if current_time - self.last_command_time < RTDE_MAX_FREQUENCY:
            return False
        
        # Validate robot state
        state_ok, state_msg = self.safety_checker.validate_robot_state()
        if not state_ok:
            if self.total_commands_sent % 100 == 0:
                print(state_msg)
            return False
        
        # Combine into 6D vector
        velocity_vector = list(linear_velocity) + list(angular_velocity)
        
        # Validate velocity data
        vel_ok, vel_msg = self.safety_checker.validate_velocity_data(velocity_vector)
        if not vel_ok:
            if self.total_commands_sent % 100 == 0:
                print(vel_msg)
            self.failed_commands += 1
            return False
        
        # Threshold check (suppression)
        is_zero = all(v == 0 for v in velocity_vector)
        was_moving = (self.last_sent_velocity is not None and 
                     any(v != 0 for v in self.last_sent_velocity))
        
        if self.last_sent_velocity is not None and not (is_zero and was_moving):
            max_delta = max(abs(v1 - v2) for v1, v2 in 
                          zip(velocity_vector, self.last_sent_velocity))
            if max_delta < self.VEL_DELTA_THRESHOLD:
                return True
        
        # Update state
        self.last_sent_velocity = velocity_vector[:]
        self.last_command_time = current_time
        self.total_commands_sent += 1
        
        # Log occasionally
        if self.total_commands_sent % 50 == 0:
            vel_str = "[" + ", ".join([f"{x:.4f}" for x in velocity_vector]) + "]"
            self.log_command(f"speedL({vel_str}, acc={acceleration:.2f})")
        
        if self.simulate:
            self.successful_commands += 1
            return True
        
        if not self.enabled or not self.rtde_c:
            self.failed_commands += 1
            return False
        
        try:
            success = self.rtde_c.speedL(velocity_vector, acceleration, 0.008)
            
            if success:
                self.successful_commands += 1
            else:
                self.failed_commands += 1
            
            return success
            
        except Exception as e:
            self.log_command(f"# ERROR: {e}")
            print(f"RTDE speedL error: {e}")
            self.connection_lost = True
            self.failed_commands += 1
            return False
    
    def move_speed_joint(self, joint_velocities, acceleration=1.0, mode_name=None):
        """
        Execute joint velocity control command (speedJ)
        
        Args:
            joint_velocities: [q0, q1, q2, q3, q4, q5] in rad/s
            acceleration: Joint acceleration in rad/s^2
            mode_name: Current control mode name (for logging)
        
        Returns:
            True if successful, False otherwise
        """
        current_time = time.time()
        
        # Rate limiting
        if current_time - self.last_command_time < RTDE_MAX_FREQUENCY:
            return False
        
        # Validate robot state
        state_ok, state_msg = self.safety_checker.validate_robot_state()
        if not state_ok:
            if self.total_commands_sent % 100 == 0:
                print(state_msg)
            return False
        
        # Validate joint velocity data
        joint_ok, joint_msg = self.safety_checker.validate_joint_velocity_data(joint_velocities)
        if not joint_ok:
            if self.total_commands_sent % 100 == 0:
                print(joint_msg)
            self.failed_commands += 1
            return False
        
        # Threshold check (suppression)
        is_zero = all(v == 0 for v in joint_velocities)
        was_moving = (self.last_sent_joint_velocity is not None and 
                     any(v != 0 for v in self.last_sent_joint_velocity))
        
        if self.last_sent_joint_velocity is not None and not (is_zero and was_moving):
            max_delta = max(abs(v1 - v2) for v1, v2 in 
                          zip(joint_velocities, self.last_sent_joint_velocity))
            if max_delta < self.JOINT_VEL_DELTA_THRESHOLD:
                return True
        
        # Update state
        self.last_sent_joint_velocity = joint_velocities[:]
        self.last_command_time = current_time
        self.total_commands_sent += 1
        
        # Log occasionally
        if self.total_commands_sent % 50 == 0:
            vel_str = "[" + ", ".join([f"{x:.4f}" for x in joint_velocities]) + "]"
            self.log_command(f"speedJ({vel_str}, acc={acceleration:.2f})")
        
        if self.simulate:
            self.successful_commands += 1
            return True
        
        if not self.enabled or not self.rtde_c:
            self.failed_commands += 1
            return False
        
        try:
            success = self.rtde_c.speedJ(joint_velocities, acceleration, 0.008)
            
            if success:
                self.successful_commands += 1
            else:
                self.failed_commands += 1
            
            return success
            
        except Exception as e:
            self.log_command(f"# ERROR: {e}")
            print(f"RTDE speedJ error: {e}")
            self.connection_lost = True
            self.failed_commands += 1
            return False
    
    def move_servo(self, target_pose, velocity=0.5, acceleration=0.5, 
                   lookahead_time=0.1, gain=300, mode_name=None):
        """
        Execute servo command (servoL) to specific target pose
        
        Args:
            target_pose: Target pose [x, y, z, rx, ry, rz]
            velocity: Velocity limit
            acceleration: Acceleration limit
            lookahead_time: Lookahead time
            gain: Proportional gain
            mode_name: Current control mode name (for logging)
        
        Returns:
            True if successful, False otherwise
        """
        current_time = time.time()
        
        # Validate robot state
        state_ok, state_msg = self.safety_checker.validate_robot_state()
        if not state_ok:
            if self.total_commands_sent % 100 == 0:
                print(state_msg)
            return False
        
        # Validate pose data
        pose_ok, pose_msg = self.safety_checker.validate_pose_data(target_pose)
        if not pose_ok:
            if self.total_commands_sent % 100 == 0:
                print(pose_msg)
            self.failed_commands += 1
            return False
        
        # Threshold check (suppression)
        if self.last_sent_pose is not None:
            target_pose = list(target_pose)
            pos_delta = math.sqrt(sum((a - b)**2 for a, b in 
                                    zip(target_pose[:3], self.last_sent_pose[:3])))
            orient_delta = math.sqrt(sum((a - b)**2 for a, b in 
                                       zip(target_pose[3:6], self.last_sent_pose[3:6])))
            
            if (pos_delta < self.POSE_DELTA_THRESHOLD and 
                orient_delta < self.ORIENT_DELTA_THRESHOLD):
                return True
        
        # Update state
        self.last_sent_pose = list(target_pose)
        self.last_command_time = current_time
        self.total_commands_sent += 1
        self.target_tcp_pose = target_pose
        
        # Log occasionally
        if self.total_commands_sent % 50 == 0:
            pose_str = "[" + ", ".join([f"{x:.3f}" for x in target_pose]) + "]"
            self.log_command(f"servoL({pose_str}, t={lookahead_time}, g={gain})")
        
        if self.simulate:
            self.successful_commands += 1
            return True
        
        if not self.enabled or not self.rtde_c:
            self.failed_commands += 1
            return False
        
        try:
            success = self.rtde_c.servoL(target_pose, velocity, acceleration, 
                                        0.008, lookahead_time, gain)
            
            if success:
                self.successful_commands += 1
            else:
                self.failed_commands += 1
            
            return success
            
        except Exception as e:
            self.log_command(f"# ERROR: {e}")
            print(f"RTDE servoL error: {e}")
            self.connection_lost = True
            self.failed_commands += 1
            return False
    
    def emergency_stop(self):
        """Emergency stop robot and open gripper"""
        if self.enabled and self.rtde_c:
            try:
                self.rtde_c.stopL(10.0)
                print("EMERGENCY STOP EXECUTED")
            except Exception as e:
                print(f"Emergency stop error: {e}")
        
        # Open gripper on emergency stop
        if self.gripper and self.gripper.is_activated():
            try:
                self.gripper.open_gripper()
                print("Gripper opened (emergency)")
            except:
                pass
        
        self.log_command("# EMERGENCY STOP")
    
    def reset_to_home(self):
        """Reset robot to home position"""
        if self.enabled and not self.simulate and self.rtde_c:
            try:
                from config.constants import UR_NEUTRAL_JOINT_POSITIONS
                self.rtde_c.moveJ(UR_NEUTRAL_JOINT_POSITIONS, 0.6, 0.6)
                print("Robot moving to neutral home position")
            except Exception as e:
                print(f"Reset to home error: {e}")
        
        self.log_command("# Reset to home")
    
    def get_status_display(self):
        """Get formatted status for display"""
        status_dict = {
            'robot_status': self.robot_status_text,
            'tcp_pose': self.current_tcp_pose,
            'joint_angles': [math.degrees(j) for j in self.last_joint_positions],
            'target_velocity': self.current_velocity_scale,
            'connection_lost': self.connection_lost,
            'reconnect_attempts': self.reconnect_attempts,
            'total_commands': self.total_commands_sent,
            'success_rate': ((self.successful_commands / self.total_commands_sent * 100) 
                           if self.total_commands_sent > 0 else 0)
        }
        
        # Add gripper status
        if self.gripper:
            gripper_status = self.gripper.get_status()
            status_dict['gripper_status'] = gripper_status['status']
            status_dict['gripper_position'] = gripper_status['position']
            status_dict['gripper_speed'] = gripper_status['speed']
            status_dict['gripper_force'] = gripper_status['force']
        else:
            status_dict['gripper_status'] = "DISABLED"
            status_dict['gripper_position'] = 0
            status_dict['gripper_speed'] = 0
            status_dict['gripper_force'] = 0
        
        return status_dict
    
    def get_statistics(self):
        """Get performance statistics"""
        success_rate = ((self.successful_commands / self.total_commands_sent * 100) 
                       if self.total_commands_sent > 0 else 0)
        return {
            'total': self.total_commands_sent,
            'successful': self.successful_commands,
            'failed': self.failed_commands,
            'success_rate': success_rate
        }
    
    def close(self):
        """Close connections and log file"""
        # Close gripper first
        if self.gripper:
            try:
                self.gripper.close()
            except:
                pass
        
        # Close RTDE connections
        if self.rtde_c:
            try:
                self.rtde_c.disconnect()
            except:
                pass
        if self.rtde_r:
            try:
                self.rtde_r.disconnect()
            except:
                pass
        
        # Close log file
        if self.log_file and not self.log_file.closed:
            stats = self.get_statistics()
            self.log_file.write(f"\n# Session Statistics:\n")
            self.log_file.write(f"# Total commands: {stats['total']}\n")
            self.log_file.write(f"# Successful: {stats['successful']}\n")
            self.log_file.write(f"# Failed: {stats['failed']}\n")
            self.log_file.write(f"# Success rate: {stats['success_rate']:.1f}%\n")
            
            if self.gripper:
                gripper_stats = self.gripper.get_status()
                self.log_file.write(f"\n# Gripper Statistics:\n")
                self.log_file.write(f"# Total commands: {gripper_stats['total_commands']}\n")
                self.log_file.write(f"# Success rate: {gripper_stats['success_rate']:.1f}%\n")
            
            self.log_file.close()
            print(f"Session stats: {stats['total']} commands, {stats['success_rate']:.1f}% success rate")