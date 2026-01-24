"""
Universal Robot RTDE Control with Wearable IMU
Main Application Entry Point - Redesigned Mode System

CHANGES:
- Fixed cube rotation for linear modes (stays upright)
- Cube only rotates in orientation modes (4, 5, 6)
- Terminal spam eliminated (mode changes only)

Author: Professional Refactored Version
Date: January 2025
"""

import pygame
from pygame.locals import *
from OpenGL.GL import *
from OpenGL.GLU import *
import numpy as np
import time
import sys
import traceback
import math

# Import configuration
from config.constants import *
from config.config_manager import ConfigManager, RuntimeConfig

# Import core modules
from core.error_handler import RobotError
from core.imu_calibration import IMUCalibration
from core.event_handler import EventHandler
from core.math_utils import apply_deadzone_ramp, quaternion_normalize
from core.coordinate_frames import frame_transform

# Import communication
from communication.imu_interface import IMUInterface
from communication.data_parser import IMUDataParser

# Import robot control
from robot.rtde_controller import RTDEController
from robot.control_modes import ControlModeDispatcher

# Import visualization
from visualization.scene_renderer import SceneRenderer
from visualization.gui_overlay import GUIOverlay
from visualization.graph_renderer import GraphRenderer

#==============================================================================
# VISUALIZATION STATE
#==============================================================================

class VisualizationState:
    """
    Manages 3D visualization state (cube position, velocity, rotation)
    
    REDESIGNED: Cube rotation now mode-dependent
    - Linear modes (1, 2, 3): Cube stays upright, position updates only
    - Orientation modes (4, 5, 6): Cube rotates to match hand orientation
    """
    
    def __init__(self):
        self.position = np.array([0.0, -1.0, 0.0])
        self.rotation_quat = np.array([0.0, 0.0, 0.0, 1.0])  # Identity (upright)
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.is_active = False
        self.current_mode = 0
        self.last_mode = -1
        self.visualizer_mode = 'CUBE'  # 'CUBE' or 'ROBOT'
        
        # NEW: Control whether cube should rotate based on mode
        self.should_show_rotation = False
    
    def update_from_imu(self, imu_data, runtime_config):
        """Update visualization based on IMU data"""
        self.is_active = True
        self.current_mode = imu_data['mode']
        
        # Determine if this mode should show rotation
        # Modes 1, 2, 3: Linear only (no rotation)
        # Modes 4, 5, 6: Orientation modes (show rotation)
        if self.current_mode in [1, 2, 3]:
            self.should_show_rotation = False
        elif self.current_mode in [4, 5, 6]:
            self.should_show_rotation = True
        else:  # Mode 0 (IDLE)
            self.should_show_rotation = False
        
        # Update rotation quaternion (only if rotation mode)
        if self.should_show_rotation:
            raw_quat = quaternion_normalize(imu_data['quaternion'])
            self.rotation_quat = frame_transform.apply_quaternion_mapping(raw_quat)
        else:
            # Keep cube upright (identity quaternion)
            self.rotation_quat = np.array([0.0, 0.0, 0.0, 1.0])
        
        # Extract Euler angles for position updates
        rel_roll, rel_pitch, rel_yaw = imu_data['euler']
        
        # Apply deadzone to all axes
        ramped_roll = apply_deadzone_ramp(rel_roll, MOVEMENT_DEADZONE, DEADZONE_RAMP_WIDTH)
        ramped_pitch = apply_deadzone_ramp(rel_pitch, MOVEMENT_DEADZONE, DEADZONE_RAMP_WIDTH)
        ramped_yaw = apply_deadzone_ramp(rel_yaw, MOVEMENT_DEADZONE, DEADZONE_RAMP_WIDTH)
        
        # Create IMU euler dict for transform
        imu_euler = {
            'roll': ramped_roll,
            'pitch': ramped_pitch,
            'yaw': ramped_yaw
        }
        
        # Get visualization-frame deltas using coordinate transform
        viz_trans = frame_transform.to_robot_translation(imu_euler)
        
        # Reset velocity for pure rotation modes (4, 5, 6)
        if self.current_mode in [4, 5, 6]:
            if self.last_mode not in [4, 5, 6]:
                self.velocity = np.array([0.0, 0.0, 0.0])
            self.velocity *= (VELOCITY_DECAY - 0.15)
        
        # Update velocity based on mode (position changes for linear modes only)
        if self.current_mode == 1:  # CRANE MODE
            self.velocity[0] += viz_trans['x'] * 0.001 * runtime_config.LINEAR_SPEED_SCALE
            # Note: Base rotation doesn't affect cube position in visualization
        elif self.current_mode == 2:  # VERTICAL_Z
            self.velocity[1] += viz_trans['z'] * 0.001 * runtime_config.LINEAR_SPEED_SCALE
        elif self.current_mode == 3:  # LATERAL_PRECISE
            self.velocity[2] += viz_trans['y'] * 0.001 * runtime_config.LINEAR_SPEED_SCALE
        # Modes 4, 5, 6: Position locked, only rotation updates (already handled above)
    
    def update_physics(self):
        """Update physics simulation"""
        self.position += self.velocity
        self.velocity *= VELOCITY_DECAY
    
    def reset(self):
        """Reset to initial state"""
        self.position = np.array([0.0, -1.0, 0.0])
        self.rotation_quat = np.array([0.0, 0.0, 0.0, 1.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.is_active = False
        self.current_mode = 0
        self.last_mode = -1
        self.should_show_rotation = False
    
    def get_color_multiplier(self, runtime_config, rtde_enabled):
        """Get color multiplier based on mode and state"""
        if not self.is_active or self.current_mode == 0:
            return (0.5, 0.5, 0.5)  # Gray for idle
        
        # Updated mode colors for new system
        mode_colors = {
            1: (1.0, 0.3, 0.3),  # Red - CRANE MODE
            2: (0.3, 1.0, 0.3),  # Green - VERTICAL_Z
            3: (0.3, 0.3, 1.0),  # Blue - LATERAL_PRECISE
            4: (1.0, 1.0, 0.3),  # Yellow - WRIST_ORIENT
            5: (1.0, 0.3, 1.0),  # Magenta - WRIST_SCREW
            6: (0.3, 1.0, 1.0)   # Cyan - TCP_ORIENT_MIMIC
        }
        
        base_color = mode_colors.get(self.current_mode, (1.0, 1.0, 1.0))
        
        if rtde_enabled:
            return base_color
        else:
            return tuple(c * 0.6 for c in base_color)  # Dim if robot disabled

#==============================================================================
# STATUS BUILDER
#==============================================================================

class StatusBuilder:
    """Builds status text for display"""
    
    @staticmethod
    def build_status_lines(vis_state, rtde_controller, imu_calibration, 
                          runtime_config, control_dispatcher, imu_connected):
        """Build list of status lines for display"""
        status = rtde_controller.get_status_display()
        remap_status = "ENABLED" if runtime_config.ENABLE_REMAPPED_MODES else "DISABLED"
        mode_name = control_dispatcher.get_mode_name(vis_state.current_mode)
        
        # IMU connection status
        imu_status = "Connected" if imu_connected else "DISCONNECTED"
        
        status_lines = [
            f"Mode: {mode_name} | {status.get('robot_status', 'N/A')} | IMU: {imu_status}",
            f"Speed: {status.get('target_velocity', 0.0) * 100:.0f}% | Lin Scale: {runtime_config.LINEAR_SPEED_SCALE:.1f}x | Ang Scale: {runtime_config.ANGULAR_SPEED_SCALE:.1f}x"
        ]
        
        if status.get('connection_lost'):
            status_lines.append(f"WARNING: Connection Lost - Reconnect: {status.get('reconnect_attempts', 0)}/5")
        
        if not imu_connected:
            status_lines.append("⚠️  IMU NOT CONNECTED - Visualization only mode")
        
        if imu_calibration.calibration_active:
            status_lines.append(f"CALIBRATION ACTIVE - Step {imu_calibration.calibration_step + 1}/2")
        
        return status_lines

#==============================================================================
# MAIN APPLICATION
#==============================================================================

def print_startup_info(config_manager, imu_calibration, imu_interface, runtime_config):
    """Print startup information"""
    config_manager.print_summary()
    
    print("\nIMU Calibration Status:")
    print(f"  Status: {imu_calibration.get_status()}")
    if imu_calibration.is_calibrated:
        print(f"  Roll offset: {imu_calibration.zero_offsets['roll']:.2f}°")
        print(f"  Pitch offset: {imu_calibration.zero_offsets['pitch']:.2f}°")
        print(f"  Yaw offset: {imu_calibration.zero_offsets['yaw']:.2f}°")
    print()
    
    # Print coordinate frame info
    print(frame_transform.get_info())
    print()
    
    print("="*70)
    print("UNIVERSAL ROBOT RTDE + IMU CONTROL SYSTEM")
    print("="*70)
    
    # IMU connection status
    if imu_interface.is_connected():
        print(f"IMU Communication: {imu_interface.get_mode()} ✓")
    else:
        print(f"IMU Communication: NOT CONNECTED (Visualization only)")
    
    print(f"Robot IP: {UR_ROBOT_IP}")
    print(f"Robot Control: {'ENABLED' if UR_ENABLED else 'DISABLED'}")
    print(f"Simulation Mode: {'ON' if UR_SIMULATE else 'OFF (REAL ROBOT)'}")
    print(f"Remapped Modes: {'ENABLED' if runtime_config.ENABLE_REMAPPED_MODES else 'DISABLED (SAFETY)'}")
    print("\nControl Modes (Redesigned):")
    for mode_num, mode_name in CONTROL_MODES.items():
        if mode_num == 0:
            continue
        print(f"  Mode {mode_num}: {mode_name}")
    print("\nControls:")
    print("  [TAB] Switch View | [R] Reset | [U] Robot Power | [S] Emergency Stop")
    print("  [↑↓] Linear Speed | [←→] Angular Speed | [C] Save Config")
    print("  [Shift+C] Calibrate IMU | [ESC] Quit")
    print("="*70 + "\n")

def main():
    """Main application loop"""
    
    # Initialize configuration
    print("Initializing configuration...")
    config_manager = ConfigManager()
    runtime_config = RuntimeConfig(config_manager)
    imu_calibration = IMUCalibration(config_manager)
    
    # Initialize PyGame and OpenGL
    print("Initializing graphics...")
    pygame.init()
    display_width = config_manager.get('visualization', 'window_width', DEFAULT_WINDOW_WIDTH)
    display_height = config_manager.get('visualization', 'window_height', DEFAULT_WINDOW_HEIGHT)
    display = (display_width, display_height)
    
    try:
        screen = pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    except Exception as e:
        print(f"ERROR: Could not initialize display: {e}")
        print("Make sure you have OpenGL support installed.")
        return
    
    pygame.display.set_caption("UR Robot RTDE + IMU Control")
    glEnable(GL_DEPTH_TEST)
    glClearColor(0.1, 0.1, 0.15, 1.0)
    
    # Setup camera
    camera_distance = config_manager.get('visualization', 'camera_distance', DEFAULT_CAMERA_DISTANCE)
    gluPerspective(45, (display[0] / display[1]), 0.1, 50.0)
    glTranslatef(0.0, 0.0, -camera_distance)
    
    # Initialize visualization
    vis_state = VisualizationState()
    SceneRenderer.initialize()
    scene_renderer = SceneRenderer()
    gui_overlay = GUIOverlay()
    
    # Initialize RTDE Controller
    print("Initializing RTDE controller...")
    rtde_controller = RTDEController(UR_ROBOT_IP, UR_ENABLED, UR_SIMULATE, config_manager)
    
    # Initialize control mode dispatcher
    control_dispatcher = ControlModeDispatcher(rtde_controller)
    
    # Initialize Graphs
    graph_width = 210
    graph_height = 80
    graph_x = display_width - 220
    graph_y_start = 180
    
    lin_vel_graph = GraphRenderer("Linear Vel (m/s)", graph_x, graph_y_start, 
                                 graph_width, graph_height, color=(0, 255, 0), y_range=(0, 0.5))
    
    ang_vel_graph = GraphRenderer("Angular Vel (rad/s)", graph_x, graph_y_start + graph_height + 20, 
                                 graph_width, graph_height, color=(255, 255, 0), y_range=(0, 1.0))
    
    # Initialize IMU communication (GRACEFUL FAILURE)
    print("Connecting to IMU...")
    imu_interface = IMUInterface(USE_WIFI, SERIAL_PORT, BAUD_RATE, WIFI_HOST, WIFI_PORT)
    imu_connected = imu_interface.connect()
    
    if not imu_connected:
        print("⚠️  IMU connection failed - continuing in visualization-only mode")
        print("    The application will start, but no IMU data will be received.")
    
    # Initialize event handler
    event_handler = EventHandler(config_manager, runtime_config, imu_calibration, 
                                 rtde_controller, vis_state, control_dispatcher)
    
    # Print startup info
    print_startup_info(config_manager, imu_calibration, imu_interface, runtime_config)
    
    # Main loop
    clock = pygame.time.Clock()
    frame_count = 0
    last_fps_print = time.time()
    
    try:
        running = True
        while running:
            frame_count += 1
            
            # Print FPS every 5 seconds
            if frame_count % 625 == 0:
                current_time = time.time()
                elapsed = current_time - last_fps_print
                fps = 625 / elapsed if elapsed > 0 else 0
                stats = rtde_controller.get_statistics()
                print(f"FPS: {fps:.1f} | Commands: {rtde_controller.total_commands_sent} | Success: {stats['success_rate']:.1f}%")
                last_fps_print = current_time
            
            # Handle events
            running = event_handler.handle_events()
            if not running:
                break
            
            # Read IMU data (only if connected)
            imu_data = None
            if imu_connected:
                line = imu_interface.read_line()
                imu_data = IMUDataParser.parse_line(line, rtde_controller.log_file)
            
            # Process IMU data
            if imu_data:
                # Apply calibration to IMU data
                calibrated_euler = imu_calibration.apply_calibration(*imu_data['euler'])
                imu_data['euler'] = np.array(calibrated_euler)
                imu_data['quaternion'] = imu_calibration.apply_calibration_quat(imu_data['quaternion'])
                
                # Capture calibration samples if active
                if imu_calibration.calibration_active:
                    imu_calibration.capture_calibration_sample(*calibrated_euler)
                
                # Update visualization
                vis_state.update_from_imu(imu_data, runtime_config)
                
                # Mode transition handling
                if imu_data['mode'] != vis_state.last_mode:
                    if vis_state.last_mode > 0 and imu_data['mode'] > 0:
                        # Only stop if we have a real connection and aren't simulating
                        if rtde_controller.enabled and rtde_controller.rtde_c and not rtde_controller.simulate:
                            try:
                                rtde_controller.rtde_c.stopL(2.0)
                                time.sleep(0.05)
                            except:
                                pass
                    
                    # Mode change message handled in control_dispatcher now
                    vis_state.last_mode = imu_data['mode']
                
                # Execute robot control
                if rtde_controller.enabled:
                    control_dispatcher.execute_mode(imu_data['mode'], imu_data, runtime_config)
            
            else:
                # No IMU data - decay velocity
                vis_state.is_active = False
                if vis_state.last_mode > 0:
                    vis_state.velocity *= (VELOCITY_DECAY - 0.1)
                    vis_state.last_mode = 0
            
            # Update physics
            vis_state.update_physics()
            
            # Update robot status
            rtde_controller.update_robot_status()
            
            # Check connection health
            if rtde_controller.enabled and rtde_controller.connection_lost:
                if rtde_controller.attempt_reconnect():
                    print("Robot reconnected successfully")
            
            # 3D rendering
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
            
            # Draw scene
            glPushMatrix()
            scene_renderer.draw_grid()
            scene_renderer.draw_axes()
            scene_renderer.draw_workspace_limit()
            glPopMatrix()
            
            # Dual-mode rendering
            if vis_state.visualizer_mode == 'ROBOT':
                # Draw Robot Model
                joint_angles = rtde_controller.last_joint_positions
                glPushMatrix()
                scene_renderer.draw_robot(joint_angles)
                glPopMatrix()
                
                # Draw ghost TCP for debugging
                scene_renderer.render_cube_at_pose(
                    rtde_controller.current_tcp_pose[:3],
                    [0, 0, 0, 1],
                    color_multiplier=(0.2, 1.0, 0.2))
            else:
                # Draw Cube (IMU orientation)
                color_multiplier = vis_state.get_color_multiplier(runtime_config, rtde_controller.enabled)
                scene_renderer.render_cube_at_pose(vis_state.position, vis_state.rotation_quat, color_multiplier)
            
            # 2D GUI overlay
            gui_overlay.setup_2d_projection(display)
            
            # Draw status text
            status_lines = StatusBuilder.build_status_lines(
                vis_state, rtde_controller, imu_calibration, runtime_config, 
                control_dispatcher, imu_connected)
            gui_overlay.draw_status_text(status_lines, display)
            
            # Render and update Graphs
            current_lin_speed = np.linalg.norm(control_dispatcher.current_velocity)
            current_ang_speed = np.linalg.norm(control_dispatcher.current_angular_velocity)
            
            lin_vel_graph.add_value(current_lin_speed)
            ang_vel_graph.add_value(current_ang_speed)
            
            lin_vel_graph.render()
            ang_vel_graph.render()
            
            gui_overlay.restore_3d_projection()
            
            pygame.display.flip()
            clock.tick(TARGET_FPS)
    
    except KeyboardInterrupt:
        print("\n\nShutdown requested by user...")
    except Exception as e:
        print(f"\n\nFATAL ERROR: {e}")
        traceback.print_exc()
    finally:
        print("\nClosing connections...")
        # Save final speed settings
        config_manager.set(runtime_config.LINEAR_SPEED_SCALE, 'speed_scaling', 'linear_scale')
        config_manager.set(runtime_config.ANGULAR_SPEED_SCALE, 'speed_scaling', 'angular_scale')
        config_manager.save_config()
        
        rtde_controller.close()
        imu_interface.close()
        pygame.quit()
        
        print("System shutdown complete.")
        print("\nSession Summary:")
        stats = rtde_controller.get_statistics()
        print(f"  Total commands: {stats['total']}")
        print(f"  Successful: {stats['successful']}")
        print(f"  Failed: {stats['failed']}")
        print(f"  Success rate: {stats['success_rate']:.1f}%")

#==============================================================================
# PROGRAM ENTRY POINT
#==============================================================================

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nProgram terminated by user (Ctrl+C)")
        sys.exit(0)
    except Exception as e:
        print(f"\n\nFATAL ERROR: {e}")
        traceback.print_exc()
        print("\nCheck log files for details.")
        sys.exit(1)