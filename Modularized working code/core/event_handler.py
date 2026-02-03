"""
Keyboard event handling
Processes user input and keyboard shortcuts

UPDATED: Added gripper control hotkeys
"""

import pygame
from pygame.locals import *

class EventHandler:
    """
    Handles keyboard events and user input
    Now includes gripper controls
    """
    
    def __init__(self, config_manager, runtime_config, imu_calibration, rtde_controller, vis_state=None, control_dispatcher=None):
        """
        Initialize event handler
        
        Args:
            config_manager: ConfigManager instance
            runtime_config: RuntimeConfig instance
            imu_calibration: IMUCalibration instance
            rtde_controller: RTDEController instance
            vis_state: VisualizationState instance (optional)
            control_dispatcher: ControlModeDispatcher instance (optional)
        """
        self.config_manager = config_manager
        self.runtime_config = runtime_config
        self.imu_calibration = imu_calibration
        self.rtde_controller = rtde_controller
        self.vis_state = vis_state
        self.control_dispatcher = control_dispatcher
    
    def handle_events(self):
        """
        Process pygame events
        
        Returns:
            True to continue running, False to quit
        """
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                return False
            
            elif event.type == pygame.KEYDOWN:
                if not self.handle_keydown(event.key):
                    return False
        
        return True
    
    def handle_keydown(self, key):
        """
        Handle keydown event
        
        Args:
            key: Pygame key constant
        
        Returns:
            True to continue, False to quit
        """
        if key == pygame.K_ESCAPE:
            return False
        
        elif key == pygame.K_r:
            # Reset to home
            print("Reset to home position")
            self.rtde_controller.reset_to_home()
            if self.vis_state:
                self.vis_state.reset()
        
        elif key == pygame.K_u:
            # Toggle robot control (Safety Interlock: Shift + U)
            keys_pressed = pygame.key.get_mods()
            if keys_pressed & pygame.KMOD_SHIFT:
                if self.rtde_controller.connection_lost and not self.rtde_controller.enabled:
                   print("Attempting to reconnect...")
                   if self.rtde_controller.attempt_reconnect():
                       self.rtde_controller.enabled = True
                       print("Robot control: ENABLED")
                       self.rtde_controller.log_command("# Robot control: ENABLED")
                   else:
                       print("Reconnection failed. Staying in Simulation Mode.")
                else:
                    self.rtde_controller.enabled = not self.rtde_controller.enabled
                    status = "ENABLED" if self.rtde_controller.enabled else "DISABLED"
                    print(f"Robot control: {status}")
                    self.rtde_controller.log_command(f"# Robot control: {status}")
            else:
                print("[SAFETY] Robot control toggle requires SHIFT + U")

        elif key == pygame.K_s:
            # Emergency stop
            self.rtde_controller.emergency_stop()

        elif key == pygame.K_TAB:
            # Toggle visualizer mode
            if self.vis_state:
                if self.vis_state.visualizer_mode == 'CUBE':
                    self.vis_state.visualizer_mode = 'ROBOT'
                else:
                    self.vis_state.visualizer_mode = 'CUBE'
                print(f"Visualizer Mode: {self.vis_state.visualizer_mode}")
        
        # ====================================================================
        # ROBOT SPEED CONTROL KEYS
        # ====================================================================
        elif key == pygame.K_UP:
            self.runtime_config.LINEAR_SPEED_SCALE = min(
                self.runtime_config.MAX_SCALE, 
                self.runtime_config.LINEAR_SPEED_SCALE + self.runtime_config.STEP_SCALE)
            print(f"Linear Speed Scale: {self.runtime_config.LINEAR_SPEED_SCALE:.1f}")
            self.config_manager.set(self.runtime_config.LINEAR_SPEED_SCALE, 
                                   'speed_scaling', 'linear_scale')
        
        elif key == pygame.K_DOWN:
            self.runtime_config.LINEAR_SPEED_SCALE = max(
                self.runtime_config.MIN_SCALE, 
                self.runtime_config.LINEAR_SPEED_SCALE - self.runtime_config.STEP_SCALE)
            print(f"Linear Speed Scale: {self.runtime_config.LINEAR_SPEED_SCALE:.1f}")
            self.config_manager.set(self.runtime_config.LINEAR_SPEED_SCALE, 
                                   'speed_scaling', 'linear_scale')
        
        elif key == pygame.K_RIGHT:
            self.runtime_config.ANGULAR_SPEED_SCALE = min(
                self.runtime_config.MAX_SCALE, 
                self.runtime_config.ANGULAR_SPEED_SCALE + self.runtime_config.STEP_SCALE)
            print(f"Angular Speed Scale: {self.runtime_config.ANGULAR_SPEED_SCALE:.1f}")
            self.config_manager.set(self.runtime_config.ANGULAR_SPEED_SCALE, 
                                   'speed_scaling', 'angular_scale')
        
        elif key == pygame.K_LEFT:
            self.runtime_config.ANGULAR_SPEED_SCALE = max(
                self.runtime_config.MIN_SCALE, 
                self.runtime_config.ANGULAR_SPEED_SCALE - self.runtime_config.STEP_SCALE)
            print(f"Angular Speed Scale: {self.runtime_config.ANGULAR_SPEED_SCALE:.1f}")
            self.config_manager.set(self.runtime_config.ANGULAR_SPEED_SCALE, 
                                   'speed_scaling', 'angular_scale')
        
        # ====================================================================
        # GRIPPER CONTROL KEYS
        # ====================================================================
        elif key == pygame.K_g:
            # Toggle gripper enable/disable (Safety Interlock: Shift + G)
            keys_pressed = pygame.key.get_mods()
            if keys_pressed & pygame.KMOD_SHIFT:
                self.runtime_config.GRIPPER_ENABLED = not self.runtime_config.GRIPPER_ENABLED
                status = "ENABLED" if self.runtime_config.GRIPPER_ENABLED else "DISABLED"
                print(f"Gripper control: {status}")
                self.config_manager.set(self.runtime_config.GRIPPER_ENABLED, 
                                       'gripper', 'enabled')
            else:
                print("[SAFETY] Gripper toggle requires SHIFT + G")
        
        elif key == pygame.K_o:
            # Manual open gripper
            if self.rtde_controller.gripper:
                self.rtde_controller.gripper.open_gripper()
            else:
                print("Gripper not available")
        
        elif key == pygame.K_c and not (pygame.key.get_mods() & pygame.KMOD_SHIFT):
            # Manual close gripper (only if NOT Shift+C which is calibration)
            if self.rtde_controller.gripper:
                self.rtde_controller.gripper.close_gripper()
            else:
                print("Gripper not available")
        
        elif key == pygame.K_LEFTBRACKET:
            # Decrease gripper speed
            if self.rtde_controller.gripper:
                new_speed = max(50, self.runtime_config.GRIPPER_SPEED - 10)
                self.runtime_config.GRIPPER_SPEED = new_speed
                self.rtde_controller.gripper.set_speed(new_speed)
            else:
                print("Gripper not available")
        
        elif key == pygame.K_RIGHTBRACKET:
            # Increase gripper speed
            if self.rtde_controller.gripper:
                new_speed = min(255, self.runtime_config.GRIPPER_SPEED + 10)
                self.runtime_config.GRIPPER_SPEED = new_speed
                self.rtde_controller.gripper.set_speed(new_speed)
            else:
                print("Gripper not available")
        
        elif key == pygame.K_SEMICOLON:
            # Decrease gripper force
            if self.rtde_controller.gripper:
                new_force = max(20, self.runtime_config.GRIPPER_FORCE - 10)
                self.runtime_config.GRIPPER_FORCE = new_force
                self.rtde_controller.gripper.set_force(new_force)
            else:
                print("Gripper not available")
        
        elif key == pygame.K_QUOTE:
            # Increase gripper force
            if self.rtde_controller.gripper:
                new_force = min(255, self.runtime_config.GRIPPER_FORCE + 10)
                self.runtime_config.GRIPPER_FORCE = new_force
                self.rtde_controller.gripper.set_force(new_force)
            else:
                print("Gripper not available")
        
        # ====================================================================
        # IMU CALIBRATION KEYS
        # ====================================================================
        elif key == pygame.K_c and (pygame.key.get_mods() & pygame.KMOD_SHIFT):
            # Shift+C to start calibration
            self.imu_calibration.start_calibration_wizard()
        
        elif key == pygame.K_SPACE:
            # Only used for advancing calibration steps
            if self.imu_calibration.calibration_active:
                if self.imu_calibration.calibration_step == 0:
                    print("Capturing zero position...")
                elif self.imu_calibration.calibration_step == 1:
                    self.imu_calibration.complete_calibration()
                    # Reset mimic references after calibration to avoid jumps
                    if self.control_dispatcher:
                        self.control_dispatcher.reset_mimic_references()
        
        return True