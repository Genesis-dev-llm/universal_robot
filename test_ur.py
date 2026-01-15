#!/usr/bin/env python3
"""
Enhanced UR Connection Tester with Detailed Diagnostics
Verifies network connectivity and basic movement via Keyboard
"""

import sys
import time
import socket
import pygame
from pygame.locals import *

# Try importing RTDE
try:
    import rtde_control
    import rtde_receive
    RTDE_AVAILABLE = True
except ImportError:
    print("ERROR: ur-rtde not installed!")
    print("Install with: pip3 install ur-rtde")
    sys.exit(1)

# CONFIGURATION
ROBOT_IP = "192.168.1.191"
SPEED = 0.05       # 5 cm/s
ACCEL = 0.5        # 0.5 m/s^2

def test_network_connection():
    """Test basic network connectivity"""
    print("\n[1/5] Testing network connection...")
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.settimeout(3)
        sock.connect((ROBOT_IP, 30004))
        sock.close()
        print("  ✅ Network connection OK")
        return True
    except socket.timeout:
        print(f"  ❌ Connection timeout - robot not responding")
        print(f"     - Check robot is powered on")
        print(f"     - Check IP address: {ROBOT_IP}")
        return False
    except ConnectionRefusedError:
        print(f"  ❌ Connection refused - RTDE port closed")
        print(f"     - Enable RTDE on teach pendant")
        print(f"     - Check robot in Remote Control mode")
        return False
    except Exception as e:
        print(f"  ❌ Connection failed: {e}")
        return False

def connect_rtde():
    """Connect to RTDE interfaces with detailed error reporting"""
    print("\n[2/5] Connecting to RTDE Control Interface...")
    try:
        rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
        print("  ✅ RTDE Control connected")
    except Exception as e:
        print(f"  ❌ RTDE Control failed: {e}")
        print(f"\n  Possible causes:")
        print(f"  - Robot not in Remote Control mode")
        print(f"  - Another program already connected")
        print(f"  - PolyScope program running")
        return None, None
    
    print("\n[3/5] Connecting to RTDE Receive Interface...")
    try:
        rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
        print("  ✅ RTDE Receive connected")
    except Exception as e:
        print(f"  ❌ RTDE Receive failed: {e}")
        if rtde_c:
            rtde_c.disconnect()
        return None, None
    
    return rtde_c, rtde_r

def check_robot_status(rtde_r):
    """Check robot operational status"""
    print("\n[4/5] Checking robot status...")
    try:
        robot_mode = rtde_r.getRobotMode()
        safety_mode = rtde_r.getSafetyMode()
        
        mode_names = {
            -1: "Disconnected", 0: "Confirm Safety", 1: "Booting",
            2: "Power Off", 3: "Power On", 4: "Idle", 5: "Backdrive",
            6: "Running", 7: "Running"
        }
        
        safety_names = {
            0: "Undefined", 1: "Normal", 2: "Reduced",
            3: "Protective Stop", 4: "Recovery", 5: "Safeguard Stop",
            6: "System Emergency Stop", 7: "Robot Emergency Stop"
        }
        
        print(f"  Robot Mode: {robot_mode} ({mode_names.get(robot_mode, 'Unknown')})")
        print(f"  Safety Mode: {safety_mode} ({safety_names.get(safety_mode, 'Unknown')})")
        
        if robot_mode == 7 and safety_mode == 1:
            print("  ✅ Robot ready for commands")
            return True
        else:
            print("  ⚠️  Robot not ready")
            if robot_mode != 7:
                print(f"     - Robot mode must be 7 (Running), currently {robot_mode}")
                print(f"     - Power on robot and release brakes")
            if safety_mode != 1:
                print(f"     - Safety mode must be 1 (Normal), currently {safety_mode}")
                print(f"     - Clear protective stops or emergency stops")
            
            # Ask user if they want to continue anyway
            response = input("\n  Continue anyway? (y/n): ")
            return response.lower() == 'y'
            
    except Exception as e:
        print(f"  ❌ Status check failed: {e}")
        return False

def main():
    print("="*70)
    print("UR ROBOT CONNECTION TESTER - Enhanced Diagnostics")
    print("="*70)
    print(f"Target IP: {ROBOT_IP}")
    
    # Test network
    if not test_network_connection():
        print("\n❌ Network test failed. Cannot proceed.")
        return
    
    # Connect to RTDE
    rtde_c, rtde_r = connect_rtde()
    if not rtde_c or not rtde_r:
        print("\n❌ RTDE connection failed. Cannot proceed.")
        return
    
    # Check status
    if not check_robot_status(rtde_r):
        print("\n❌ Robot not ready. Exiting.")
        rtde_c.disconnect()
        rtde_r.disconnect()
        return
    
    # Initialize Pygame
    print("\n[5/5] Initializing control interface...")
    try:
        pygame.init()
        screen = pygame.display.set_mode((500, 400))
        pygame.display.set_caption("UR Connection Tester - CONNECTED")
        font = pygame.font.SysFont("Arial", 16)
        print("  ✅ Interface ready")
    except Exception as e:
        print(f"  ❌ Pygame init failed: {e}")
        rtde_c.disconnect()
        rtde_r.disconnect()
        return
    
    print("\n" + "="*70)
    print("✅ ALL SYSTEMS READY - CONNECTION SUCCESSFUL!")
    print("="*70)
    print("\n--- CONTROLS ---")
    print("1, 2, 3: Switch Mode (SPEED, SERVO, MOVE)")
    print("ARROWS : Move X & Y    | W / S  : Move Z")
    print("I / K  : Rotate Rx     | J / L  : Rotate Ry | U / O  : Rotate Rz")
    print("[ / ]  : Speed +/-     | - / =  : Step +/-")
    print("SPACE  : EMERGENCY STOP | ESC    : Quit")
    print("="*70 + "\n")

    clock = pygame.time.Clock()
    running = True
    mode = "SPEED"
    
    # Adjustable parameters
    current_speed = SPEED
    ang_speed = 0.2
    current_step_size = 0.002
    ang_step = 0.01
    
    # For position-based modes
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
                elif event.key == K_LEFTBRACKET:
                    current_speed = max(0.01, current_speed - 0.01)
                    print(f"Speed: {current_speed:.2f} m/s")
                elif event.key == K_RIGHTBRACKET:
                    current_speed = min(0.5, current_speed + 0.01)
                    print(f"Speed: {current_speed:.2f} m/s")
                elif event.key == K_MINUS:
                    current_step_size = max(0.0001, current_step_size - 0.0005)
                    print(f"Step: {current_step_size*1000:.1f} mm")
                elif event.key == K_EQUALS:
                    current_step_size = min(0.05, current_step_size + 0.0005)
                    print(f"Step: {current_step_size*1000:.1f} mm")
                elif event.key == K_SPACE:
                    print("🛑 EMERGENCY STOP")
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

        # Execution
        is_input = any([keys[K_UP], keys[K_DOWN], keys[K_LEFT], keys[K_RIGHT], 
                        keys[K_w], keys[K_s], keys[K_i], keys[K_k], 
                        keys[K_j], keys[K_l], keys[K_u], keys[K_o]])
        
        try:
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
            print(f"❌ Command error: {e}")
            break

        # UI Rendering
        screen.fill((20, 25, 30))
        
        # Header
        pygame.draw.rect(screen, (40, 60, 80), (0, 0, 500, 50))
        header = font.render(f"UR TESTER - {mode} MODE - CONNECTED", True, (100, 255, 150))
        screen.blit(header, (20, 15))
        
        # Status
        try:
            tcp = rtde_r.getActualTCPPose()
            lines = [
                f"IP: {ROBOT_IP} ✓",
                f"Status: {'[MOVING]' if is_input else '[IDLE]'}",
                "",
                f"Speed: {current_speed:.2f} m/s | Step: {current_step_size*1000:.1f} mm",
                "",
                f"TCP Position:",
                f"  X: {tcp[0]:+.4f} m    Rx: {tcp[3]:+.3f} rad",
                f"  Y: {tcp[1]:+.4f} m    Ry: {tcp[4]:+.3f} rad",
                f"  Z: {tcp[2]:+.4f} m    Rz: {tcp[5]:+.3f} rad",
                "",
                "Controls: 1=SPEED 2=SERVO 3=MOVE",
                "          Arrows=XY  W/S=Z  IJKL/UO=Rotate",
                "          SPACE=STOP  ESC=Quit"
            ]
        except:
            lines = ["ERROR: Lost connection to robot!"]
        
        for i, text in enumerate(lines):
            color = (255, 200, 100) if "MOVING" in text else (200, 200, 200)
            if "ERROR" in text:
                color = (255, 100, 100)
            img = font.render(text, True, color)
            screen.blit(img, (20, 70 + i*24))
        
        pygame.display.flip()
        clock.tick(60)

    print("\nClosing connections...")
    try:
        rtde_c.stopL(2.0)
        rtde_c.disconnect()
        rtde_r.disconnect()
    except:
        pass
    pygame.quit()
    print("Done.")

if __name__ == "__main__":
    main()