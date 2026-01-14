#!/usr/bin/env python3
"""
Standalone UR Connection Tester
Verifies network connectivity and basic movement via Keyboard.
Bypasses all IMU and Visualization logic.
"""

import sys
import time
import rtde_control
import rtde_receive
import pygame
from pygame.locals import *

# CONFIGURATION - Update these if needed
ROBOT_IP = "192.168.1.100"
SPEED = 0.05       # 5 cm/s
ACCEL = 0.5        # 0.5 m/s^2

def main():
    print(f"--- UR Connection Tester ---")
    print(f"Target IP: {ROBOT_IP}")
    
    # 1. Initialize Pygame for Keyboard
    pygame.init()
    screen = pygame.display.set_mode((400, 300))
    pygame.display.set_caption("UR Connection Tester")
    font = pygame.font.SysFont("Arial", 18)
    
    print("Connecting to robot...")
    try:
        # 2. Try to connect
        rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
        rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
        print("✅ SUCCESS: Connected to Universal Robot")
    except Exception as e:
        print(f"❌ ERROR: Could not connect to {ROBOT_IP}")
        print(f"Details: {e}")
        pygame.quit()
        return

    print("\n--- CONTROLS ---")
    print("1, 2, 3: Switch Mode (SPEED, SERVO, MOVE)")
    print("ARROWS : Move X & Y    | W / S  : Move Z")
    print("I / K  : Rotate Rx     | J / L  : Rotate Ry | U / O  : Rotate Rz")
    print("[ / ]  : Speed +/-     | - / =  : Step +/-")
    print("SPACE  : EMERGENCY STOP | ESC    : Quit")
    print("----------------\n")

    clock = pygame.time.Clock()
    running = True
    mode = "SPEED" # SPEED, SERVO, MOVE
    
    # Adjustable parameters
    current_speed = SPEED
    ang_speed = 0.2 # 0.2 rad/s for rotation
    current_step_size = 0.002 # 2mm per tick
    ang_step = 0.01 # 0.01 rad per tick (~0.5 deg)
    
    # For Position-based modes (SERVO, MOVE)
    current_target_pose = rtde_r.getActualTCPPose()
    
    while running:
        vx, vy, vz = 0.0, 0.0, 0.0
        vrx, vry, vrz = 0.0, 0.0, 0.0
        
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False
            if event.type == KEYDOWN:
                if event.key == K_ESCAPE:
                    running = False
                elif event.key == K_1:
                    mode = "SPEED"
                    print("Mode: SPEED (speedL)")
                elif event.key == K_2:
                    mode = "SERVO"
                    current_target_pose = rtde_r.getActualTCPPose()
                    print("Mode: SERVO (servoL)")
                elif event.key == K_3:
                    mode = "MOVE"
                    current_target_pose = rtde_r.getActualTCPPose()
                    print("Mode: MOVE (moveL)")
                
                # Adjust Speed
                elif event.key == K_LEFTBRACKET: # [
                    current_speed = max(0.01, current_speed - 0.01)
                elif event.key == K_RIGHTBRACKET: # ]
                    current_speed = min(0.5, current_speed + 0.01)
                
                # Adjust Step Size
                elif event.key == K_MINUS: # -
                    current_step_size = max(0.0001, current_step_size - 0.0005)
                elif event.key == K_EQUALS: # = (+ key)
                    current_step_size = min(0.05, current_step_size + 0.0005)

                elif event.key == K_SPACE:
                    print("🛑 EMERGENCY STOP pressed")
                    rtde_c.stopL(2.0)

        # Get continuous key states
        keys = pygame.key.get_pressed()
        
        # Translation
        if keys[K_UP]:    vx = current_speed; current_target_pose[0] += current_step_size
        if keys[K_DOWN]:  vx = -current_speed; current_target_pose[0] -= current_step_size
        if keys[K_LEFT]:  vy = current_speed; current_target_pose[1] += current_step_size
        if keys[K_RIGHT]: vy = -current_speed; current_target_pose[1] -= current_step_size
        if keys[K_w]:     vz = current_speed; current_target_pose[2] += current_step_size
        if keys[K_s]:     vz = -current_speed; current_target_pose[2] -= current_step_size
        
        # Rotation
        if keys[K_i]:     vrx = ang_speed; current_target_pose[3] += ang_step
        if keys[K_k]:     vrx = -ang_speed; current_target_pose[3] -= ang_step
        if keys[K_j]:     vry = ang_speed; current_target_pose[4] += ang_step
        if keys[K_l]:     vry = -ang_speed; current_target_pose[4] -= ang_step
        if keys[K_u]:     vrz = ang_speed; current_target_pose[5] += ang_step
        if keys[K_o]:     vrz = -ang_speed; current_target_pose[5] -= ang_step

        # Execution Logic
        try:
            is_input = any([keys[K_UP], keys[K_DOWN], keys[K_LEFT], keys[K_RIGHT], keys[K_w], keys[K_s],
                           keys[K_i], keys[K_k], keys[K_j], keys[K_l], keys[K_u], keys[K_o]])
            
            if mode == "SPEED":
                if is_input:
                    rtde_c.speedL([vx, vy, vz, vrx, vry, vrz], ACCEL, 0.008)
                else:
                    rtde_c.speedL([0, 0, 0, 0, 0, 0], 2.0, 0.008)
            
            elif mode == "SERVO" and is_input:
                rtde_c.servoL(current_target_pose, 0.5, 0.5, 0.008, 0.05, 500)
            
            elif mode == "MOVE" and is_input:
                rtde_c.moveL(current_target_pose, 0.1, 0.5, True)

        except Exception as e:
            print(f"Robot execution error: {e}")
            break

        # UI Rendering
        screen.fill((20, 20, 25)) # Sleek dark blue
        
        # Header
        pygame.draw.rect(screen, (50, 50, 70), (0, 0, 400, 40))
        header_text = font.render(f"UR TESTER - {mode} MODE", True, (0, 255, 100))
        screen.blit(header_text, (20, 10))
        
        # Status
        lines = [
            f"IP: {ROBOT_IP}",
            f"Status: {'[ MOVING ]' if is_input else '[ IDLE ]'}",
            "",
            f"LINEAR SPEED: {current_speed:.2f} m/s",
            f"STEP SIZE: {current_step_size*1000:.1f} mm",
            "",
            f"X: {current_target_pose[0]:.3f}   Rx: {current_target_pose[3]:.3f}",
            f"Y: {current_target_pose[1]:.3f}   Ry: {current_target_pose[4]:.3f}",
            f"Z: {current_target_pose[2]:.3f}   Rz: {current_target_pose[5]:.3f}",
            "",
            "1:SPEED 2:SERVO 3:MOVE | ESC:Exit"
        ]
        
        for i, text in enumerate(lines):
            color = (200, 200, 200)
            if "MOVING" in text: color = (255, 200, 0)
            img = font.render(text, True, color)
            screen.blit(img, (20, 60 + i*22))
        
        pygame.display.flip()
        clock.tick(125)

    print("Closing connection...")
    rtde_c.stopL(2.0)
    rtde_c.disconnect()
    pygame.quit()
    print("Done.")

if __name__ == "__main__":
    main()
