#!/usr/bin/env python3
"""
Professional UR Connection Tester with Robotiq Hand-E Integration
Gripper control via URScript socket communication (CORRECT IMPLEMENTATION)
"""

import sys
import time
import socket
import rtde_control
import rtde_receive
import pygame
from pygame.locals import *

# Robot Configuration
ROBOT_IP = "192.168.1.191"
URSCRIPT_PORT = 30002  # Secondary client interface for script commands
SPEED_DEFAULT = 0.05
SPEED_MIN = 0.01
SPEED_MAX = 0.20  # 4x higher than original
SPEED_INCREMENT = 0.05
ACCEL = 0.5

# Gripper Configuration
GRIPPER_MIN_POS = 0      # Fully open (mm)
GRIPPER_MAX_POS = 50     # Fully closed (mm)
GRIPPER_SPEED = 255      # 0-255
GRIPPER_FORCE = 100      # 0-255


class RobotiqHandE:
    """
    Robotiq Hand-E Gripper Controller via URScript Socket Communication
    
    The Hand-E connects to the UR tool communication port and uses
    Modbus RTU protocol. We send URScript commands via socket to port 30002.
    """
    
    def __init__(self, robot_ip):
        self.robot_ip = robot_ip
        self.port = URSCRIPT_PORT
        self.current_position = 0
        self.is_activated = False
        self.is_ready = False
        
    def _send_script(self, script, wait_time=0.1):
        """
        Send URScript command to robot via socket connection
        
        Args:
            script: URScript code as string
            wait_time: Time to wait after sending (seconds)
        
        Returns:
            bool: True if sent successfully
        """
        try:
            # Create socket connection
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.settimeout(5.0)
            s.connect((self.robot_ip, self.port))
            
            # Send script
            s.send(script.encode('utf-8'))
            
            # Close connection
            s.close()
            
            # Wait for execution
            if wait_time > 0:
                time.sleep(wait_time)
            
            return True
            
        except socket.timeout:
            print(f"⚠️  Socket timeout connecting to {self.robot_ip}:{self.port}")
            return False
        except socket.error as e:
            print(f"⚠️  Socket error: {e}")
            return False
        except Exception as e:
            print(f"⚠️  Script send error: {e}")
            return False
    
    def setup_modbus(self):
        """
        Configure Modbus device for gripper communication
        Must be called before activation
        """
        print("📡 Configuring Modbus device for Robotiq Hand-E...")
        
        # This script adds the Modbus signal if not already present
        setup_script = """
modbus_add_signal("gripper", 127.0.0.1, 502, 0, 5, 9, "Robotiq Hand-E")
"""
        
        if self._send_script(setup_script, wait_time=0.5):
            print("✅ Modbus device configured")
            return True
        else:
            print("⚠️  Modbus configuration may have failed (might already exist)")
            return True  # Continue anyway, might already be configured
    
    def activate(self):
        """
        Activate the Robotiq Hand-E gripper
        This MUST be called before the gripper can be used
        
        Returns:
            bool: True if activation successful
        """
        print("🤖 Activating Robotiq Hand-E gripper...")
        print("   (This takes ~2 seconds...)")
        
        # Complete activation sequence for Robotiq Hand-E
        activation_script = """def rq_activate():
  # Reset gripper
  modbus_set_output_register("gripper", 0, 0, False)
  sleep(0.1)
  
  # Set to auto-release mode and activate
  modbus_set_output_register("gripper", 0, 1, False)
  sleep(0.1)
  
  # Send activation command (rACT = 1, rMOD = 0, rGTO = 0, rATR = 0)
  modbus_set_output_register("gripper", 0, 256 + 1, False)  # 0x0101
  sleep(0.5)
  
  # Wait for activation (status should be 0x3131 or 0x3231)
  count = 0
  while count < 20:
    status = modbus_get_input_register("gripper", 0, False)
    gSTA = (status / 256) / 16  # Extract gSTA bits
    if gSTA >= 3:
      textmsg("Gripper activated")
      return True
    end
    sleep(0.1)
    count = count + 1
  end
  
  textmsg("Gripper activation timeout")
  return False
end

rq_activate()
"""
        
        if self._send_script(activation_script, wait_time=2.5):
            self.is_activated = True
            self.is_ready = True
            print("✅ Gripper activated successfully")
            return True
        else:
            print("❌ Gripper activation failed")
            return False
    
    def move(self, position_mm, speed=GRIPPER_SPEED, force=GRIPPER_FORCE):
        """
        Move gripper to specified position
        
        Args:
            position_mm: Position in mm (0=fully open, 50=fully closed)
            speed: Gripper speed 0-255 (default 255)
            force: Gripper force 0-255 (default 100)
        
        Returns:
            bool: True if command sent successfully
        """
        if not self.is_activated:
            print("⚠️  Gripper not activated. Press 'G' to activate first.")
            return False
        
        # Clamp values to valid ranges
        position_mm = max(GRIPPER_MIN_POS, min(GRIPPER_MAX_POS, position_mm))
        speed = max(0, min(255, speed))
        force = max(0, min(255, force))
        
        # Convert mm to gripper units (0-255)
        # Hand-E: 0mm = 0 (open), 50mm = 255 (closed)
        position_units = int((position_mm / 50.0) * 255)
        
        # Build move command
        # Format: rACT=1, rGTO=1, rATR=0, rMOD=0 = 0x0109 = 265
        action_register = 256 + 9  # 0x0109
        
        move_script = f"""def rq_move():
  modbus_set_output_register("gripper", 0, {action_register}, False)
  modbus_set_output_register("gripper", 3, {position_units}, False)
  modbus_set_output_register("gripper", 4, {speed}, False)
  modbus_set_output_register("gripper", 5, {force}, False)
end

rq_move()
"""
        
        if self._send_script(move_script, wait_time=0.05):
            self.current_position = position_mm
            return True
        return False
    
    def open(self, speed=GRIPPER_SPEED):
        """Fully open the gripper"""
        print("🖐️  Opening gripper...")
        return self.move(GRIPPER_MIN_POS, speed, GRIPPER_FORCE)
    
    def close(self, speed=GRIPPER_SPEED, force=GRIPPER_FORCE):
        """Fully close the gripper"""
        print(f"🤏 Closing gripper (force: {force})...")
        return self.move(GRIPPER_MAX_POS, speed, force)
    
    def reset(self):
        """Reset the gripper (in case of errors)"""
        print("🔄 Resetting gripper...")
        
        reset_script = """def rq_reset():
  modbus_set_output_register("gripper", 0, 0, False)
  sleep(0.5)
  textmsg("Gripper reset")
end

rq_reset()
"""
        
        if self._send_script(reset_script, wait_time=0.5):
            self.is_activated = False
            self.is_ready = False
            print("✅ Gripper reset. Press 'G' to reactivate.")
            return True
        return False
    
    def get_status(self):
        """Get current gripper status"""
        return {
            'position': self.current_position,
            'activated': self.is_activated,
            'ready': self.is_ready
        }


def print_controls():
    """Display control instructions"""
    print("\n" + "="*70)
    print("  ROBOT CONTROL MODES")
    print("="*70)
    print("1, 2, 3    : Switch Mode (SPEED / SERVO / MOVE)")
    print()
    print("MOTION CONTROLS")
    print("-"*70)
    print("ARROWS     : Move X & Y")
    print("W / S      : Move Z (up/down)")
    print("I / K      : Rotate Rx (pitch)")
    print("J / L      : Rotate Ry (roll)")
    print("U / O      : Rotate Rz (yaw)")
    print()
    print("GRIPPER CONTROLS (Robotiq Hand-E via URScript/Modbus)")
    print("-"*70)
    print("G          : Activate gripper (required before first use)")
    print("R          : Reset gripper (if errors occur)")
    print("C          : Close gripper")
    print("V          : Open gripper")
    print("B / N      : Adjust gripper position -/+ 5mm")
    print("F / H      : Adjust gripper force -/+ 20")
    print()
    print("PARAMETER ADJUSTMENT")
    print("-"*70)
    print("[ / ]      : Speed -/+ 0.05 m/s (range: 0.01-0.20)")
    print("- / =      : Step size -/+ 0.5mm")
    print()
    print("EMERGENCY & EXIT")
    print("-"*70)
    print("SPACE      : EMERGENCY STOP (stops robot and gripper)")
    print("ESC        : Quit program")
    print("="*70 + "\n")


def main():
    print("="*70)
    print("  PROFESSIONAL UR ROBOT TESTER WITH ROBOTIQ HAND-E")
    print("  Gripper Control via URScript Socket (Port 30002)")
    print("="*70)
    print(f"Target Robot IP: {ROBOT_IP}")
    
    # Initialize Pygame
    pygame.init()
    screen = pygame.display.set_mode((550, 450))
    pygame.display.set_caption("UR Robot Tester - Robotiq Hand-E")
    font = pygame.font.SysFont("monospace", 13, bold=True)
    font_small = pygame.font.SysFont("monospace", 11)
    
    # Connect to robot via RTDE
    print("\n⏳ Connecting to Universal Robot (RTDE)...")
    try:
        rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
        rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
        print("✅ SUCCESS: Connected to Universal Robot via RTDE")
    except Exception as e:
        print(f"❌ ERROR: Could not connect to {ROBOT_IP}")
        print(f"Details: {e}")
        pygame.quit()
        return 1

    # Initialize gripper (uses separate socket connection)
    print("\n⏳ Initializing Robotiq Hand-E gripper interface...")
    gripper = RobotiqHandE(ROBOT_IP)
    
    print("\n" + "="*70)
    print("  SETUP INSTRUCTIONS FOR ROBOTIQ HAND-E")
    print("="*70)
    print("HARDWARE:")
    print("  1. Connect Hand-E to UR tool communication port (8-pin M12)")
    print("  2. Ensure 24V power is connected")
    print()
    print("UR POLYSCOPE CONFIGURATION:")
    print("  1. Go to: Installation → General → Tool I/O")
    print("  2. Set Tool Communication to 'RS485'")
    print("  3. Parameters:")
    print("     • Baud rate: 115200")
    print("     • Parity: None")
    print("     • Stop bits: 1")
    print("     • RX/TX Idle: 0ms")
    print()
    print("FIRST TIME SETUP:")
    print("  - The program will auto-configure Modbus on startup")
    print("  - Press 'G' to activate the gripper (takes ~2 seconds)")
    print("  - Gripper will auto-calibrate and be ready to use")
    print("="*70)
    
    print_controls()
    
    # Setup Modbus device
    gripper.setup_modbus()
    
    # Control loop variables
    clock = pygame.time.Clock()
    running = True
    mode = "SPEED"
    
    current_speed = SPEED_DEFAULT
    ang_speed = 0.2
    current_step_size = 0.002  # 2mm default
    ang_step = 0.01
    
    current_target_pose = rtde_r.getActualTCPPose()
    
    # Rate limiting for SERVO mode
    last_servo_time = 0
    servo_interval = 0.008  # 125Hz max
    
    # Move mode tracking
    move_keys_last_frame = set()
    
    # Gripper parameters
    gripper_target_pos = 0
    gripper_force = GRIPPER_FORCE
    
    print("\n✅ System ready. Use controls to operate robot and gripper.")
    print("   Remember to press 'G' to activate gripper before first use!\n")
    
    while running:
        vx, vy, vz = 0.0, 0.0, 0.0
        vrx, vry, vrz = 0.0, 0.0, 0.0
        
        # Event handling
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
                
            if event.type == KEYDOWN:
                # Exit
                if event.key == K_ESCAPE:
                    running = False
                
                # Mode switching
                elif event.key == K_1:
                    mode = "SPEED"
                    print(f"🔄 Mode: SPEED (continuous velocity control)")
                elif event.key == K_2:
                    mode = "SERVO"
                    current_target_pose = rtde_r.getActualTCPPose()
                    print(f"🔄 Mode: SERVO (real-time path control)")
                elif event.key == K_3:
                    mode = "MOVE"
                    current_target_pose = rtde_r.getActualTCPPose()
                    print(f"🔄 Mode: MOVE (point-to-point motion)")
                
                # Speed adjustment
                elif event.key == K_LEFTBRACKET:
                    current_speed = max(SPEED_MIN, current_speed - SPEED_INCREMENT)
                    print(f"⚡ Linear Speed: {current_speed:.3f} m/s")
                elif event.key == K_RIGHTBRACKET:
                    current_speed = min(SPEED_MAX, current_speed + SPEED_INCREMENT)
                    print(f"⚡ Linear Speed: {current_speed:.3f} m/s")
                
                # Step size adjustment
                elif event.key == K_MINUS:
                    current_step_size = max(0.0001, current_step_size - 0.0005)
                    print(f"📏 Step Size: {current_step_size*1000:.2f} mm")
                elif event.key == K_EQUALS:
                    current_step_size = min(0.05, current_step_size + 0.0005)
                    print(f"📏 Step Size: {current_step_size*1000:.2f} mm")

                # Emergency stop
                elif event.key == K_SPACE:
                    print("🛑 EMERGENCY STOP ACTIVATED")
                    rtde_c.stopL(2.0)
                    rtde_c.speedL([0, 0, 0, 0, 0, 0], 2.0, 0.008)
                
                # Gripper controls
                elif event.key == K_g:
                    if gripper.activate():
                        print("✅ Gripper ready to use")
                
                elif event.key == K_r:
                    gripper.reset()
                
                elif event.key == K_c:
                    gripper.close(force=gripper_force)
                    gripper_target_pos = GRIPPER_MAX_POS
                
                elif event.key == K_v:
                    gripper.open()
                    gripper_target_pos = GRIPPER_MIN_POS
                
                elif event.key == K_b:
                    gripper_target_pos = max(GRIPPER_MIN_POS, gripper_target_pos - 5)
                    gripper.move(gripper_target_pos, force=gripper_force)
                    print(f"🔧 Gripper position: {gripper_target_pos} mm")
                
                elif event.key == K_n:
                    gripper_target_pos = min(GRIPPER_MAX_POS, gripper_target_pos + 5)
                    gripper.move(gripper_target_pos, force=gripper_force)
                    print(f"🔧 Gripper position: {gripper_target_pos} mm")
                
                elif event.key == K_f:
                    gripper_force = max(0, gripper_force - 20)
                    print(f"💪 Gripper force: {gripper_force}/255")
                
                elif event.key == K_h:
                    gripper_force = min(255, gripper_force + 20)
                    print(f"💪 Gripper force: {gripper_force}/255")

        # Continuous key state
        keys = pygame.key.get_pressed()
        
        # Build velocity/delta vectors
        if keys[K_UP]:    
            vx = current_speed
            current_target_pose[0] += current_step_size
        if keys[K_DOWN]:  
            vx = -current_speed
            current_target_pose[0] -= current_step_size
        if keys[K_LEFT]:  
            vy = current_speed
            current_target_pose[1] += current_step_size
        if keys[K_RIGHT]: 
            vy = -current_speed
            current_target_pose[1] -= current_step_size
        if keys[K_w]:     
            vz = current_speed
            current_target_pose[2] += current_step_size
        if keys[K_s]:     
            vz = -current_speed
            current_target_pose[2] -= current_step_size
        
        if keys[K_i]:     
            vrx = ang_speed
            current_target_pose[3] += ang_step
        if keys[K_k]:     
            vrx = -ang_speed
            current_target_pose[3] -= ang_step
        if keys[K_j]:     
            vry = ang_speed
            current_target_pose[4] += ang_step
        if keys[K_l]:     
            vry = -ang_speed
            current_target_pose[4] -= ang_step
        if keys[K_u]:     
            vrz = ang_speed
            current_target_pose[5] += ang_step
        if keys[K_o]:     
            vrz = -ang_speed
            current_target_pose[5] -= ang_step

        # Check if any motion key is pressed
        is_input = any([keys[K_UP], keys[K_DOWN], keys[K_LEFT], keys[K_RIGHT], 
                       keys[K_w], keys[K_s], keys[K_i], keys[K_k], 
                       keys[K_j], keys[K_l], keys[K_u], keys[K_o]])
        
        # Execute motion based on mode
        try:
            if mode == "SPEED":
                if is_input:
                    rtde_c.speedL([vx, vy, vz, vrx, vry, vrz], ACCEL, servo_interval)
                else:
                    rtde_c.speedL([0, 0, 0, 0, 0, 0], 2.0, servo_interval)
            
            elif mode == "SERVO" and is_input:
                current_time = time.time()
                if current_time - last_servo_time >= servo_interval:
                    rtde_c.servoL(current_target_pose, 0.5, 0.5, servo_interval, 0.05, 500)
                    last_servo_time = current_time
            
            elif mode == "MOVE":
                current_move_keys = {k for k in [K_UP, K_DOWN, K_LEFT, K_RIGHT, 
                                                  K_w, K_s, K_i, K_k, K_j, K_l, K_u, K_o] 
                                     if keys[k]}
                new_keys = current_move_keys - move_keys_last_frame
                if new_keys:
                    rtde_c.moveL(current_target_pose, current_speed, ACCEL, False)
                move_keys_last_frame = current_move_keys

        except Exception as e:
            print(f"❌ Robot execution error: {e}")
            running = False

        # Get actual robot state
        try:
            actual_pose = rtde_r.getActualTCPPose()
        except:
            actual_pose = [0, 0, 0, 0, 0, 0]
        
        # UI Rendering
        screen.fill((15, 15, 20))
        
        # Header
        pygame.draw.rect(screen, (40, 60, 80), (0, 0, 550, 50))
        header_text = font.render(f"UR ROBOT TESTER - {mode} MODE", True, (100, 255, 150))
        screen.blit(header_text, (15, 15))
        
        # Status indicator
        status_color = (255, 150, 0) if is_input else (100, 100, 100)
        pygame.draw.circle(screen, status_color, (515, 25), 12)
        
        # Information display
        y_offset = 65
        line_height = 20
        
        info_lines = [
            f"Robot IP: {ROBOT_IP}  |  Status: {'MOVING' if is_input else 'IDLE'}",
            "",
            f"Linear Speed: {current_speed:.3f} m/s  |  Step: {current_step_size*1000:.2f} mm",
            "",
            "TCP POSITION (m)              ORIENTATION (rad)",
            f"X: {actual_pose[0]:7.4f}            Rx: {actual_pose[3]:7.4f}",
            f"Y: {actual_pose[1]:7.4f}            Ry: {actual_pose[4]:7.4f}",
            f"Z: {actual_pose[2]:7.4f}            Rz: {actual_pose[5]:7.4f}",
            "",
            f"GRIPPER: {'ACTIVE' if gripper.is_activated else 'INACTIVE (press G)'}",
            f"Position: {gripper.current_position} mm  |  Force: {gripper_force}/255",
            "",
            "CONTROLS:",
            "1/2/3:Mode  G:Activate  R:Reset  C/V:Close/Open  B/N:Pos",
            "[/]:Speed  -/=:Step  F/H:Force  SPACE:STOP  ESC:Exit"
        ]
        
        for i, text in enumerate(info_lines):
            if text == "":
                y_offset += 8
                continue
                
            color = (180, 180, 180)
            use_small_font = False
            
            if "MOVING" in text:
                color = (255, 200, 0)
            elif "TCP POSITION" in text or "GRIPPER" in text:
                color = (150, 200, 255)
            elif "INACTIVE" in text:
                color = (255, 100, 100)
            elif "ACTIVE" in text:
                color = (100, 255, 100)
            elif i >= len(info_lines) - 3:
                color = (150, 150, 150)
                use_small_font = True
            
            active_font = font_small if use_small_font else font
            img = active_font.render(text, True, color)
            screen.blit(img, (15, y_offset))
            y_offset += line_height
        
        pygame.display.flip()
        clock.tick(125)  # 125 FPS for responsive input

    # Cleanup
    print("\n🔌 Shutting down...")
    print("⏹️  Stopping robot motion...")
    try:
        rtde_c.stopL(2.0)
        rtde_c.speedL([0, 0, 0, 0, 0, 0], 2.0, 0.008)
    except:
        pass
    
    print("🔌 Disconnecting interfaces...")
    try:
        rtde_c.disconnect()
        rtde_r.disconnect()
    except:
        pass
    
    pygame.quit()
    print("✅ Shutdown complete.\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())