#!/usr/bin/env python3
"""
Professional UR Connection Tester with Robotiq Hand-E (URCap) Integration.
Non-blocking URCap usage that won't crash RTDE loops.

Dependencies:
    pip install ur_rtde pygame
"""

import sys
import time
import math
import pygame
from pygame.locals import *

# Try importing rtde libraries, provide clear error if missing
try:
    import rtde_control
    import rtde_receive
except ImportError:
    print("❌ Critical Error: 'ur_rtde' library not found.")
    print("   Please install it using: pip install ur_rtde")
    sys.exit(1)

# ==========================================
# CONFIGURATION
# ==========================================
ROBOT_IP = "192.168.1.191"

# Motion Parameters
SPEED_DEFAULT = 0.05
SPEED_MIN = 0.01
SPEED_MAX = 0.25
SPEED_INCREMENT = 0.05
ACCEL = 0.5
SERVO_FREQ = 125  # Hz
DT = 1.0 / SERVO_FREQ

# Gripper Configuration (Hand-E)
# Note: Robotiq internal logic uses 0-255 (0=Open, 255=Closed)
GRIPPER_STROKE_MM = 50.0  # Max stroke of Hand-E
GRIPPER_MIN_MM = 0.0      # User definition: 0mm
GRIPPER_MAX_MM = 50.0     # User definition: 50mm
GRIPPER_SPEED_DEFAULT = 255  # Max speed (0-255)
GRIPPER_FORCE_DEFAULT = 150  # Medium force (0-255)

# Estimated time to traverse full stroke at speed=255 (seconds).
# This is an estimate — for exact timing you should read gripper status via Modbus.
GRIPPER_FULL_STROKE_BASE_TIME = 0.6  # seconds at speed=255 for full stroke

# Minimum cooldown after issuing a gripper command (prevents rapid commands)
GRIPPER_MIN_COOLDOWN = 0.1  # seconds


class RobotiqHandE:
    """
    Controller for Robotiq Hand-E using URCap Script Functions (non-blocking).
    Uses rq_activate(), rq_move(), rq_open(), rq_close() which are provided by
    the Robotiq URCap installed on the robot controller.
    """

    def __init__(self, rtde_c: rtde_control.RTDEControlInterface):
        self.rtde_c = rtde_c
        self.current_mm = 0.0
        self.is_activated = False
        self._last_move_end_time = 0.0
        self._last_cmd_time = 0.0

    def _mm_to_bit(self, mm: float) -> int:
        """Convert 0-50mm range to 0-255 byte range (clamped)."""
        mm = max(GRIPPER_MIN_MM, min(GRIPPER_MAX_MM, mm))
        return int((mm / GRIPPER_STROKE_MM) * 255)

    def _send_script_safe(self, script: str) -> bool:
        """
        Wrap sendCustomScript with try/except. Return True if script issued.
        """
        try:
            # RTDEControlInterface provides sendCustomScript()
            # It returns None; exceptions are thrown on failure.
            self.rtde_c.sendCustomScript(script)
            self._last_cmd_time = time.time()
            return True
        except Exception as e:
            print(f"⚠️  sendCustomScript failed: {e}")
            return False

    def activate(self) -> bool:
        """
        Sends non-blocking activation (rq_activate) rather than rq_activate_and_wait.
        If you require verification, add Modbus-based read of status registers.
        """
        print("🤖 Gripper: Sending non-blocking activation command...")
        script = """
def gripper_activate():
    rq_activate()
end
gripper_activate()
"""
        ok = self._send_script_safe(script)
        if ok:
            # Assume activation begins — mark as activated so UI can use it.
            # For absolute certainty, poll gripper via Modbus (not implemented here).
            self.is_activated = True
            # small grace time
            self._last_move_end_time = time.time() + 0.4
            return True
        return False

    def _estimate_move_time(self, from_mm: float, to_mm: float, speed_byte: int) -> float:
        """
        Estimate how long a move will take based on distance and speed.
        This is a heuristic: real device timing may vary; for robust apps use Modbus status.
        """
        dist = abs(to_mm - from_mm)
        if dist <= 0.0:
            return 0.0
        proportion = dist / GRIPPER_STROKE_MM
        # Lower speed (smaller byte) -> slower. We scale by (255/speed).
        speed = max(1, min(255, int(speed_byte)))
        estimated = GRIPPER_FULL_STROKE_BASE_TIME * proportion * (255.0 / speed)
        # Clamp estimates to reasonable bounds
        return max(0.05, min(10.0, estimated))

    def move(self, position_mm: float, speed=GRIPPER_SPEED_DEFAULT, force=GRIPPER_FORCE_DEFAULT) -> bool:
        """
        Non-blocking move using URCap rq_move(pos, speed, force)
        - position_mm in mm (0..50)
        - speed & force in 0..255
        Sets a local busy timer so the UI won't spam commands.
        """
        if not self.is_activated:
            print("⚠️  Gripper not activated. Press 'G' first.")
            return False

        # throttling: prevent issuing new command too rapidly
        if time.time() < self._last_cmd_time + GRIPPER_MIN_COOLDOWN:
            # Still in minimal cooldown
            print("ℹ️  Gripper command ignored (cooldown).")
            return False

        target_bit = self._mm_to_bit(position_mm)
        speed = max(0, min(255, int(speed)))
        force = max(0, min(255, int(force)))

        # Build non-blocking URScript call
        script = f"""
def gripper_move():
    rq_move({target_bit}, {speed}, {force})
end
gripper_move()
"""
        ok = self._send_script_safe(script)
        if not ok:
            return False

        # Estimate move time and set busy window
        est = self._estimate_move_time(self.current_mm, position_mm, speed)
        self._last_move_end_time = time.time() + est
        # Immediately reflect target for UI; if you want conservative behavior, update when motion ends
        self.current_mm = position_mm
        print(f"🤖 Gripper: move issued -> {position_mm:.1f} mm | est {est:.2f}s")
        return True

    def open(self) -> bool:
        """Non-blocking open using rq_open()"""
        if not self.is_activated:
            print("⚠️  Gripper not activated.")
            return False

        script = """
def gripper_open():
    rq_open()
end
gripper_open()
"""
        ok = self._send_script_safe(script)
        if ok:
            est = self._estimate_move_time(self.current_mm, GRIPPER_MIN_MM, GRIPPER_SPEED_DEFAULT)
            self._last_move_end_time = time.time() + est
            self.current_mm = GRIPPER_MIN_MM
            print(f"🤖 Gripper: open issued | est {est:.2f}s")
            return True
        return False

    def close(self, force=GRIPPER_FORCE_DEFAULT) -> bool:
        """Non-blocking close using rq_close()"""
        if not self.is_activated:
            print("⚠️  Gripper not activated.")
            return False

        script = """
def gripper_close():
    rq_close()
end
gripper_close()
"""
        ok = self._send_script_safe(script)
        if ok:
            est = self._estimate_move_time(self.current_mm, GRIPPER_MAX_MM, GRIPPER_SPEED_DEFAULT)
            self._last_move_end_time = time.time() + est
            self.current_mm = GRIPPER_MAX_MM
            print(f"🤖 Gripper: close issued | est {est:.2f}s")
            return True
        return False

    def is_busy(self) -> bool:
        """Return True while the last commanded move is expected to be in progress."""
        return time.time() < self._last_move_end_time

    def get_status(self) -> dict:
        """Return a small status dict (UI friendly)."""
        return {
            "activated": self.is_activated,
            "current_mm": self.current_mm,
            "busy": self.is_busy(),
            "last_cmd": self._last_cmd_time
        }


def print_controls():
    """Display control layout to console."""
    print("\n" + "=" * 60)
    print("🎮  UR ROBOT CONTROL INTERFACE")
    print("=" * 60)
    print(" MODE SELECTION")
    print(" 1: SPEED (Velocity) | 2: SERVO (Position) | 3: MOVE (Path)")
    print("-" * 60)
    print(" MOTION (TCP)")
    print(" Arrows : X / Y Plane")
    print(" W / S  : Z Axis (Up/Down)")
    print(" I / K  : Rx (Pitch)")
    print(" J / L  : Ry (Roll)")
    print(" U / O  : Rz (Yaw)")
    print("-" * 60)
    print(" GRIPPER (Robotiq Hand-E)")
    print(" G      : ACTIVATE (Required at start)")
    print(" C      : CLOSE (non-blocking)")
    print(" V      : OPEN (non-blocking)")
    print(" B / N  : Nudge -/+ 5mm (non-blocking)")
    print("-" * 60)
    print(" SETTINGS")
    print(" [ / ]  : Speed +/-")
    print(" - / =  : Step Size +/-")
    print("-" * 60)
    print(" SAFETY")
    print(" SPACE  : EMERGENCY STOP")
    print(" ESC    : QUIT")
    print("=" * 60 + "\n")


def main():
    print("=" * 60)
    print("  PROFESSIONAL UR ROBOT TESTER + ROBOTIQ URCAP (NON-BLOCKING)")
    print("=" * 60)
    print(f"Target Robot IP: {ROBOT_IP}")

    # Initialize Pygame
    pygame.init()
    screen = pygame.display.set_mode((600, 450))  # Slightly larger window
    pygame.display.set_caption("UR Robot Control Center")
    font = pygame.font.SysFont("consolas", 14, bold=True)
    header_font = pygame.font.SysFont("consolas", 18, bold=True)

    # Connect to robot
    print("\n⏳ Connecting to Universal Robot interfaces...")
    try:
        rtde_c = rtde_control.RTDEControlInterface(ROBOT_IP)
        rtde_r = rtde_receive.RTDEReceiveInterface(ROBOT_IP)
        print("✅ SUCCESS: Connected to RTDE Control & Receive")
    except RuntimeError as e:
        print(f"❌ CONNECTION ERROR: {e}")
        print("   Check IP address and ensure Robot is in Remote Control mode.")
        pygame.quit()
        return 1

    # Initialize gripper wrapper
    gripper = RobotiqHandE(rtde_c)

    print_controls()

    # State Variables
    clock = pygame.time.Clock()
    running = True
    mode = "SPEED"

    # Motion State
    current_speed = SPEED_DEFAULT
    ang_speed = 0.3
    current_step_size = 0.002  # 2mm default
    ang_step = 0.015

    # Target Tracking
    try:
        current_target_pose = rtde_r.getActualTCPPose()
    except Exception:
        current_target_pose = [0.4, 0.0, 0.5, 0.0, 0.0, 0.0]
    last_servo_time = 0
    move_keys_last_frame = set()

    # Gripper Tracking
    gripper_target_mm = 0.0

    while running:
        # Reset velocity vectors every frame
        vx, vy, vz = 0.0, 0.0, 0.0
        vrx, vry, vrz = 0.0, 0.0, 0.0

        # 1. Event Handling
        for event in pygame.event.get():
            if event.type == QUIT:
                running = False

            if event.type == KEYDOWN:
                # Exit
                if event.key == K_ESCAPE:
                    running = False

                # Mode Switching
                elif event.key == K_1:
                    mode = "SPEED"
                    print(f"🔄 Mode Switched: SPEED")
                elif event.key == K_2:
                    mode = "SERVO"
                    try:
                        current_target_pose = rtde_r.getActualTCPPose()
                    except Exception:
                        pass
                    print(f"🔄 Mode Switched: SERVO")
                elif event.key == K_3:
                    mode = "MOVE"
                    try:
                        current_target_pose = rtde_r.getActualTCPPose()
                    except Exception:
                        pass
                    print(f"🔄 Mode Switched: MOVE")

                # Speed / Step Adjustment
                elif event.key == K_LEFTBRACKET:
                    current_speed = max(SPEED_MIN, current_speed - SPEED_INCREMENT)
                elif event.key == K_RIGHTBRACKET:
                    current_speed = min(SPEED_MAX, current_speed + SPEED_INCREMENT)
                elif event.key == K_MINUS:
                    current_step_size = max(0.0001, current_step_size - 0.0005)
                elif event.key == K_EQUALS:
                    current_step_size = min(0.05, current_step_size + 0.0005)

                # Emergency Stop
                elif event.key == K_SPACE:
                    print("🛑 EMERGENCY STOP TRIGGERED")
                    try:
                        rtde_c.stopL(2.0)
                        rtde_c.stopScript()
                    except Exception as e:
                        print(f"⚠️ stop error: {e}")

                # --- Gripper Logic ---
                # Use non-blocking URCap commands so RTDE loops are not interrupted.
                elif event.key == K_g:
                    if gripper.activate():
                        print("✅ Gripper activation requested")
                    else:
                        print("❌ Gripper activation failed")

                elif event.key == K_c:
                    if gripper.is_busy():
                        print("ℹ️  Gripper busy; close ignored.")
                    else:
                        gripper.close()
                        gripper_target_mm = GRIPPER_MAX_MM
                        # Sync pose to prevent servo jump after gripper move
                        try:
                            current_target_pose = rtde_r.getActualTCPPose()
                        except Exception:
                            pass

                elif event.key == K_v:
                    if gripper.is_busy():
                        print("ℹ️  Gripper busy; open ignored.")
                    else:
                        gripper.open()
                        gripper_target_mm = GRIPPER_MIN_MM
                        try:
                            current_target_pose = rtde_r.getActualTCPPose()
                        except Exception:
                            pass

                elif event.key == K_b:
                    # nudge -5mm
                    if gripper.is_busy():
                        print("ℹ️  Gripper busy; nudge ignored.")
                    else:
                        gripper_target_mm = max(GRIPPER_MIN_MM, gripper_target_mm - 5.0)
                        gripper.move(gripper_target_mm)
                        try:
                            current_target_pose = rtde_r.getActualTCPPose()
                        except Exception:
                            pass

                elif event.key == K_n:
                    # nudge +5mm
                    if gripper.is_busy():
                        print("ℹ️  Gripper busy; nudge ignored.")
                    else:
                        gripper_target_mm = min(GRIPPER_MAX_MM, gripper_target_mm + 5.0)
                        gripper.move(gripper_target_mm)
                        try:
                            current_target_pose = rtde_r.getActualTCPPose()
                        except Exception:
                            pass

        # 2. Continuous Input Processing
        keys = pygame.key.get_pressed()

        # Linear Mapping
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

        # Angular Mapping
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

        is_moving = any([keys[K_UP], keys[K_DOWN], keys[K_LEFT], keys[K_RIGHT],
                        keys[K_w], keys[K_s], keys[K_i], keys[K_k],
                        keys[K_j], keys[K_l], keys[K_u], keys[K_o]])

        # 3. Robot Execution
        try:
            if mode == "SPEED":
                if is_moving:
                    # speedL(vel, accel, time)
                    rtde_c.speedL([vx, vy, vz, vrx, vry, vrz], ACCEL, DT)
                else:
                    # Soft stop to keep robot engaged but still
                    rtde_c.speedL([0, 0, 0, 0, 0, 0], 5.0, DT)

            elif mode == "SERVO" and is_moving:
                t_now = time.time()
                if t_now - last_servo_time >= DT:
                    # servoL(pose, vel, accel, time, lookahead, gain)
                    # Vel/accel set to reasonable defaults; tune as needed.
                    rtde_c.servoL(current_target_pose, 0.3, 0.8, DT, 0.03, 2000)
                    last_servo_time = t_now

            elif mode == "MOVE":
                current_move_keys = {k for k in [K_UP, K_DOWN, K_LEFT, K_RIGHT,
                                                K_w, K_s, K_i, K_k, K_j, K_l, K_u, K_o] if keys[k]}
                new_keys = current_move_keys - move_keys_last_frame
                if new_keys:
                    # Discrete moveL command (async move)
                    rtde_c.moveL(current_target_pose, current_speed, ACCEL, True)
                move_keys_last_frame = current_move_keys

        except Exception as e:
            # Often happens if gripper script interrupts control or RTDE temporarily errors.
            print(f"⚠️  Execution Warning: {e}")
            # Re-sync target pose to actual to prevent jump on next loop.
            try:
                current_target_pose = rtde_r.getActualTCPPose()
            except Exception:
                pass
            # If severe, continue loop so user can press emergency stop / recover.

        # 4. Data Refresh & Rendering
        try:
            actual_pose = rtde_r.getActualTCPPose()
        except Exception:
            actual_pose = current_target_pose

        screen.fill((20, 24, 28))  # Dark background

        # UI Header
        pygame.draw.rect(screen, (30, 40, 50), (0, 0, 600, 60))
        title_srf = header_font.render(f"UR CONTROL: {mode} MODE", True, (0, 255, 128))
        screen.blit(title_srf, (20, 20))

        # Activity Indicator
        ind_color = (0, 255, 0) if is_moving else (80, 80, 80)
        pygame.draw.circle(screen, ind_color, (570, 30), 10)

        # Text Block
        y_pos = 80
        lh = 24  # Line Height

        gr_status = gripper.get_status()
        busy_str = "BUSY" if gr_status["busy"] else "READY"
        lines = [
            ("IP Address", ROBOT_IP, (200, 200, 200)),
            ("Status", "MOVING" if is_moving else "HOLDING", (255, 200, 50) if is_moving else (150, 150, 150)),
            ("", "", None),
            (f"Speed: {current_speed:.3f} m/s", f"Step: {current_step_size*1000:.1f} mm", (100, 200, 255)),
            ("", "", None),
            ("TCP POSITION (meters)", "ORIENTATION (radians)", (150, 255, 150)),
            (f"X: {actual_pose[0]:7.4f}", f"Rx: {actual_pose[3]:7.4f}", (255, 255, 255)),
            (f"Y: {actual_pose[1]:7.4f}", f"Ry: {actual_pose[4]:7.4f}", (255, 255, 255)),
            (f"Z: {actual_pose[2]:7.4f}", f"Rz: {actual_pose[5]:7.4f}", (255, 255, 255)),
            ("", "", None),
            ("ROBOTIQ HAND-E", "", (255, 100, 100)),
            (f"State: {'ACTIVE' if gr_status['activated'] else 'NEEDS ACTIVATION'}", "", (255, 255, 255)),
            (f"Last Cmd Pos: {gr_status['current_mm']:.1f} mm", f"Busy: {busy_str}", (255, 255, 255)),
        ]

        for label, val, color in lines:
            if label == "" and val == "":
                y_pos += 10
                continue

            full_str = f"{label:<30} {val}"
            txt_srf = font.render(full_str, True, color if color else (200, 200, 200))
            screen.blit(txt_srf, (20, y_pos))
            y_pos += lh

        # Footer instructions
        footer = "G:Activate | C:Close V:Open | B/N:Adj | SPACE:STOP | ESC:Exit"
        foot_srf = font.render(footer, True, (100, 100, 100))
        screen.blit(foot_srf, (20, 420))

        pygame.display.flip()
        clock.tick(SERVO_FREQ)  # Sync loop to robot servo rate

    # Clean Exit
    print("\n🔻 Shutting down...")
    try:
        rtde_c.stopL(2.0)
        rtde_c.stopScript()
        rtde_c.disconnect()
        rtde_r.disconnect()
    except Exception:
        pass

    pygame.quit()
    print("✅ System Halted.")
    return 0


if __name__ == "__main__":
    sys.exit(main())