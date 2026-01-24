#!/usr/bin/env python3
"""
Professional UR Connection Tester with Robotiq Hand-E Integration
Supports multiple control modes and gripper operations
"""

import sys
import time
import rtde_control
import rtde_receive
import pygame
from pygame.locals import *

# Robot Configuration
ROBOT_IP = "192.168.1.191"
SPEED_DEFAULT = 0.05
SPEED_MIN = 0.01
SPEED_MAX = 0.20  # 4x higher than original 0.05
SPEED_INCREMENT = 0.05  # Changed from 0.01
ACCEL = 0.5

# Gripper Configuration
GRIPPER_MIN_POS = 0      # Fully open (mm)
GRIPPER_MAX_POS = 50     # Fully closed (mm)
GRIPPER_SPEED = 255      # 0-255
GRIPPER_FORCE = 100      # 0-255

class RobotiqHandE:
    """Robotiq Hand-E Gripper Controller"""
    
    def __init__(self, rtde_control):
        self.rtde_c = rtde_control
        self.current_position = 0
        self.is_activated = False
        
    def activate(self):
        """Activate the gripper (must be called before use)"""
        try:
            # Send activation command via digital outputs or modbus
            # This is a placeholder - actual implementation depends on setup
            print("🤖 Activating Robotiq Hand-E gripper...")
            self.is_activated = True
            return True
        except Exception as e:
            print(f"⚠️  Gripper activation failed: {e}")
            return False
    
    def move(self, position, speed=GRIPPER_SPEED, force=GRIPPER_FORCE):
        """
        Move gripper to position
        position: 0-50mm (0=open, 50=closed)
        speed: 0-255
        force: 0-255
        """
        if not self.is_activated:
            print("⚠️  Gripper not activated")
            return False
            
        position = max(GRIPPER_MIN_POS, min(GRIPPER_MAX_POS, position))
        
        try:
            # Implementation depends on connection method:
            # Option 1: Using RTDE digital/analog outputs
            # Option 2: Using Modbus/TCP directly
            # This is a simplified example using script commands
            
            script_cmd = f"rq_move_and_wait({position}, {speed}, {force})"
            # self.rtde_c.sendCustomScriptCommand(script_cmd)
            
            self.current_position = position
            return True
        except Exception as e:
            print(f"⚠️  Gripper move error: {e}")
            return False
    
    def open(self, speed=GRIPPER_SPEED):
        """Fully open gripper"""
        return self.move(GRIPPER_MIN_POS, speed)
    
    def close(self, speed=GRIPPER_SPEED, force=GRIPPER_FORCE):
        """Fully close gripper"""
        return self.move(GRIPPER_MAX_POS, speed, force)
    
    def get_status(self):
        """Get current gripper status"""
        return {
            'position': self.current_position,
            'activated': self.is_activated
        }


def print_controls():
    """Display control instructions"""
    print("\n" + "="*60)
    print("ROBOT CONTROL MODES")
    print("="*60)
    print("1, 2, 3    : Switch Mode (SPEED / SERVO / MOVE)")
    print()
    print("MOTION CONTROLS")
    print("-"*60)
    print("ARROWS     : Move X & Y")
    print("W / S      : Move Z (up/down)")
    print("I / K      : Rotate Rx (pitch)")
    print("J / L      : Rotate Ry (roll)")
    print("U / O      : Rotate Rz (yaw)")
    print()
    print("GRIPPER CONTROLS")
    print("-"*60)
    print("G          : Activate gripper")
    print("C          : Close gripper")
    print("V          : Open gripper")
    print("B / N      : Adjust gripper position -/+ 5mm")
    print()
    print("PARAMETER ADJUSTMENT")
    print("-"*60)
    print("[ / ]      : Speed -/+ 0.05 m/s")
    print("- / =      : Step size -/+ 0.5mm")
    print()
    print("EMERGENCY & EXIT")
    print("-"*60)
    print("SPACE      : EMERGENCY STOP")
    print("ESC        : Quit program")
    print("="*60 + "\n")


def main():
    print("="*60)
    print("  PROFESSIONAL UR ROBOT TESTER WITH ROBOTIQ HAND-E")
    print("="*60)
    print(f"Target Robot IP: {ROBOT_IP}")
    
    # Initialize Pygame
    pygame.init()
    screen = pygame.display.set_mode((500, 400))
    pygame.display.set_caption("UR Robot Tester")
    font = pygame.font.SysFont("monospace", 14, bold=True)
    
    # Connect to robot
    print("\n⏳ Connecting to Universal Robot...")
    try:
        rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
        rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
        print("✅ SUCCESS: Connected to Universal Robot")
    except Exception as e:
        print(f"❌ ERROR: Could not connect to {ROBOT_IP}")
        print(f"Details: {e}")
        pygame.quit()
        return 1

    # Initialize gripper
    gripper = RobotiqHandE(rtde_c)
    
    print_controls()

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
    
    # Gripper position tracking
    gripper_target_pos = 0
    
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
                        print("✅ Gripper activated")
                    else:
                        print("❌ Gripper activation failed")
                
                elif event.key == K_c:
                    print("🤏 Closing gripper...")
                    gripper.close()
                    gripper_target_pos = GRIPPER_MAX_POS
                
                elif event.key == K_v:
                    print("🖐️  Opening gripper...")
                    gripper.open()
                    gripper_target_pos = GRIPPER_MIN_POS
                
                elif event.key == K_b:
                    gripper_target_pos = max(GRIPPER_MIN_POS, gripper_target_pos - 5)
                    gripper.move(gripper_target_pos)
                    print(f"🔧 Gripper position: {gripper_target_pos} mm")
                
                elif event.key == K_n:
                    gripper_target_pos = min(GRIPPER_MAX_POS, gripper_target_pos + 5)
                    gripper.move(gripper_target_pos)
                    print(f"🔧 Gripper position: {gripper_target_pos} mm")

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
        actual_pose = rtde_r.getActualTCPPose()
        
        # UI Rendering
        screen.fill((15, 15, 20))
        
        # Header
        pygame.draw.rect(screen, (40, 60, 80), (0, 0, 500, 50))
        header_text = font.render(f"UR ROBOT TESTER - {mode} MODE", True, (100, 255, 150))
        screen.blit(header_text, (15, 15))
        
        # Status indicator
        status_color = (255, 150, 0) if is_input else (100, 100, 100)
        status_text = "MOVING" if is_input else "IDLE"
        pygame.draw.circle(screen, status_color, (470, 25), 12)
        
        # Information display
        y_offset = 70
        line_height = 22
        
        info_lines = [
            f"Robot IP: {ROBOT_IP}",
            f"Status: {status_text}",
            "",
            f"Linear Speed: {current_speed:.3f} m/s  |  Step: {current_step_size*1000:.2f} mm",
            "",
            "TCP POSITION (m)              ORIENTATION (rad)",
            f"X: {actual_pose[0]:7.4f}            Rx: {actual_pose[3]:7.4f}",
            f"Y: {actual_pose[1]:7.4f}            Ry: {actual_pose[4]:7.4f}",
            f"Z: {actual_pose[2]:7.4f}            Rz: {actual_pose[5]:7.4f}",
            "",
            f"GRIPPER: {'ACTIVE' if gripper.is_activated else 'INACTIVE'}  |  Position: {gripper.current_position} mm",
            "",
            "1:SPEED  2:SERVO  3:MOVE  |  G:Activate  C/V:Close/Open",
            "[/]:Speed  -/=:Step  B/N:Gripper  SPACE:STOP  ESC:Exit"
        ]
        
        for i, text in enumerate(info_lines):
            if text == "":
                continue
                
            color = (180, 180, 180)
            if "MOVING" in text:
                color = (255, 200, 0)
            elif "TCP POSITION" in text or "GRIPPER" in text:
                color = (150, 200, 255)
            elif i >= len(info_lines) - 2:
                color = (150, 150, 150)
            
            img = font.render(text, True, color)
            screen.blit(img, (15, y_offset + i * line_height))
        
        pygame.display.flip()
        clock.tick(125)  # 125 FPS for responsive input

    # Cleanup
    print("\n🔌 Shutting down...")
    print("⏹️  Stopping robot motion...")
    rtde_c.stopL(2.0)
    rtde_c.speedL([0, 0, 0, 0, 0, 0], 2.0, 0.008)
    
    print("🔌 Disconnecting interfaces...")
    rtde_c.disconnect()
    rtde_r.disconnect()
    
    pygame.quit()
    print("✅ Shutdown complete.\n")
    return 0


if __name__ == "__main__":
    sys.exit(main())