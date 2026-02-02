"""
FLEX SENSOR GRIPPER CONTROL - DEAR PYGUI VERSION
=================================================
Modern, GPU-accelerated GUI for Hand-E gripper control

Requirements:
    pip install dearpygui pyserial

Usage:
    1. Upload flex sensor code to Arduino/ESP32
    2. Update ARDUINO_PORT and ROBOT_IP below
    3. Run: python flex_gripper_dearpygui.py
"""

import dearpygui.dearpygui as dpg
import socket
import serial
import time
import threading
import sys
import os

# ========== CONFIGURATION ==========
ARDUINO_PORT = "/dev/ttyUSB0"  # Change to your port
ARDUINO_BAUD = 115200
ROBOT_IP = "192.168.1.191"     # Your robot IP

# Control settings
DEAD_ZONE = 3           # percent


# ========== GRIPPER CONTROLLER ==========
class HandEGripper:
    """Hand-E gripper controller via socket port 63352"""
    
    def __init__(self, robot_ip):
        self.robot_ip = robot_ip
        self.socket = None
        
    def connect(self):
        """Connect to gripper socket server"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.robot_ip, 63352))
            self.socket.settimeout(2.0)
            return True
        except Exception as e:
            print(f"✗ Gripper connection failed: {e}")
            return False
        
    def send_command(self, cmd):
        """Send command to gripper"""
        if self.socket:
            try:
                self.socket.send(f"{cmd}\n".encode('utf-8'))
                time.sleep(0.01)
            except Exception as e:
                print(f"Command error: {e}")
        
    def activate(self):
        """Activate the gripper"""
        self.send_command("SET ACT 1")
        time.sleep(0.5)
        self.send_command("SET GTO 1")
        time.sleep(1.5)
        
    def move_to(self, position, speed=255, force=255):
        """Move gripper to position (0-255)"""
        self.send_command(f"SET SPE {speed}")
        self.send_command(f"SET FOR {force}")
        self.send_command(f"SET POS {position}")
        
    def open(self, speed=255, force=255):
        """Open gripper"""
        self.move_to(0, speed, force)
        
    def disconnect(self):
        """Close connection"""
        if self.socket:
            self.socket.close()


# ========== APPLICATION STATE ==========
class AppState:
    def __init__(self):
        self.arduino = None
        self.gripper = None
        self.running = False
        self.connected = False
        
        # Control state
        self.current_percentage = 0
        self.last_sent_percentage = -999
        
        # Settings
        self.speed = 255
        self.force = 150
        
        # Statistics
        self.total_updates = 0


app_state = AppState()


# ========== GUI CALLBACKS ==========
def connect_callback():
    """Connect to Arduino and gripper"""
    dpg.set_value("status_text", "Connecting...")
    dpg.configure_item("status_text", color=(255, 165, 0))  # Orange
    
    # Connect to Arduino
    try:
        app_state.arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)
        time.sleep(2)
        print("✓ Arduino connected")
        
        # Wait for READY signal
        timeout = time.time() + 5
        while time.time() < timeout:
            line = app_state.arduino.readline().decode('utf-8').strip()
            if line == "READY":
                print("✓ Arduino ready")
                break
            time.sleep(0.1)
                
    except Exception as e:
        dpg.set_value("status_text", f"Arduino Error: {e}")
        dpg.configure_item("status_text", color=(255, 0, 0))
        return
    
    # Connect to gripper
    app_state.gripper = HandEGripper(ROBOT_IP)
    if not app_state.gripper.connect():
        dpg.set_value("status_text", "Gripper Connection Failed")
        dpg.configure_item("status_text", color=(255, 0, 0))
        app_state.arduino.close()
        return
    
    print("✓ Gripper connected")
    app_state.gripper.activate()
    print("✓ Gripper activated")
    
    # Start control loop
    app_state.running = True
    app_state.connected = True
    
    control_thread = threading.Thread(target=control_loop, daemon=True)
    control_thread.start()
    
    # Update UI
    dpg.set_value("status_text", "CONNECTED & ACTIVE")
    dpg.configure_item("status_text", color=(0, 255, 0))  # Green
    dpg.configure_item("connect_btn", enabled=False)
    dpg.configure_item("disconnect_btn", enabled=True)


def disconnect_callback():
    """Disconnect from devices"""
    app_state.running = False
    time.sleep(0.5)
    
    try:
        if app_state.gripper:
            app_state.gripper.open()
            time.sleep(1)
            app_state.gripper.disconnect()
    except Exception as e:
        print(f"Error during gripper disconnect: {e}")
        
    try:
        if app_state.arduino:
            app_state.arduino.close()
    except Exception as e:
        print(f"Error during Arduino disconnect: {e}")
        
    app_state.connected = False
    
    dpg.set_value("status_text", "DISCONNECTED")
    dpg.configure_item("status_text", color=(255, 0, 0))
    dpg.configure_item("connect_btn", enabled=True)
    dpg.configure_item("disconnect_btn", enabled=False)


def speed_callback(sender, value):
    """Update speed value"""
    app_state.speed = int(value)
    dpg.set_value("speed_value", f"{int(value)}")


def force_callback(sender, value):
    """Update force value"""
    app_state.force = int(value)
    dpg.set_value("force_value", f"{int(value)}")


# ========== CONTROL LOOP ==========
def control_loop():
    """Main control loop - runs in separate thread"""
    while app_state.running:
        try:
            # Read from Arduino
            line = app_state.arduino.readline().decode('utf-8', errors='ignore').strip()
            
            if line.startswith("ANGLE:"):
                try:
                    percentage = int(line.split(":")[1])
                    app_state.total_updates += 1
                    app_state.current_percentage = percentage
                    
                    # Update GUI
                    update_display(percentage)
                    
                    # Check dead zone
                    if abs(percentage - app_state.last_sent_percentage) < DEAD_ZONE:
                        continue
                    
                    app_state.last_sent_percentage = percentage
                    
                    # Map to gripper position (0-255)
                    gripper_pos = int((percentage / 100.0) * 255)
                    
                    # Send to gripper
                    app_state.gripper.move_to(gripper_pos, app_state.speed, app_state.force)
                
                except (ValueError, IndexError) as e:
                    # Malformed data - ignore
                    print(f"⚠ Bad data: {line}")
                    continue
                
        except Exception as e:
            if app_state.running:  # Only print error if still supposed to be running
                print(f"Control loop error: {e}")
            time.sleep(0.1)


def update_display(percentage):
    """Update the percentage display"""
    try:
        # Update percentage text
        dpg.set_value("percentage_text", f"{percentage}%")
        
        # Update progress bar
        dpg.set_value("progress_bar", percentage / 100.0)
        
        # Color coding
        if percentage < 33:
            color = (39, 174, 96)  # Green
        elif percentage < 66:
            color = (243, 156, 18)  # Orange
        else:
            color = (231, 76, 60)  # Red
        
        dpg.configure_item("percentage_text", color=color)
        
        # Update stats
        dpg.set_value("updates_text", f"Updates: {app_state.total_updates}")
    except:
        # GUI not ready yet, skip update
        pass


# ========== GUI SETUP ==========
def create_gui():
    """Create the Dear PyGui interface"""
    
    dpg.create_context()
    
    # Main window
    with dpg.window(label="Flex Sensor Gripper Control", tag="main_window"):
        
        # Title
        dpg.add_text("FLEX SENSOR → HAND-E GRIPPER", tag="title_text")
        dpg.bind_item_theme("title_text", "title_theme")
        dpg.add_spacer(height=10)
        
        # Status
        dpg.add_text("Status: NOT CONNECTED", tag="status_text", color=(255, 0, 0))
        dpg.add_spacer(height=10)
        dpg.add_separator()
        dpg.add_spacer(height=10)
        
        # Percentage display
        with dpg.group(horizontal=False):
            dpg.add_text("Gripper Position")
            dpg.add_text("0%", tag="percentage_text")
            dpg.bind_item_theme("percentage_text", "percentage_theme")
            
            dpg.add_progress_bar(tag="progress_bar", default_value=0.0, width=-1)
        
        dpg.add_spacer(height=10)
        dpg.add_separator()
        dpg.add_spacer(height=10)
        
        # Control sliders
        with dpg.group():
            dpg.add_text("Controls", color=(100, 200, 255))
            dpg.add_spacer(height=5)
            
            # Speed
            with dpg.group(horizontal=True):
                dpg.add_text("Speed:")
                dpg.add_spacer(width=20)
                dpg.add_slider_int(
                    tag="speed_slider",
                    default_value=255,
                    min_value=50,
                    max_value=255,
                    width=300,
                    callback=speed_callback
                )
                dpg.add_spacer(width=10)
                dpg.add_text("255", tag="speed_value")
            
            # Force
            with dpg.group(horizontal=True):
                dpg.add_text("Force:")
                dpg.add_spacer(width=20)
                dpg.add_slider_int(
                    tag="force_slider",
                    default_value=150,
                    min_value=20,
                    max_value=255,
                    width=300,
                    callback=force_callback
                )
                dpg.add_spacer(width=10)
                dpg.add_text("150", tag="force_value")
        
        dpg.add_spacer(height=10)
        dpg.add_separator()
        dpg.add_spacer(height=10)
        
        # Statistics
        with dpg.group():
            dpg.add_text("Statistics", color=(100, 200, 255))
            dpg.add_text("Updates: 0", tag="updates_text")
        
        dpg.add_spacer(height=10)
        dpg.add_separator()
        dpg.add_spacer(height=10)
        
        # Buttons
        with dpg.group(horizontal=True):
            dpg.add_button(
                label="CONNECT",
                tag="connect_btn",
                callback=connect_callback,
                width=150,
                height=40
            )
            dpg.add_button(
                label="DISCONNECT",
                tag="disconnect_btn",
                callback=disconnect_callback,
                width=150,
                height=40,
                enabled=False
            )
    
    # Themes
    with dpg.theme(tag="title_theme"):
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 10, 10)
            dpg.add_theme_color(dpg.mvThemeCol_Text, (100, 200, 255))
    
    with dpg.theme(tag="percentage_theme"):
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 20, 20)
    
    # Global theme - dark gaming style
    with dpg.theme(tag="global_theme"):
        with dpg.theme_component(dpg.mvAll):
            dpg.add_theme_color(dpg.mvThemeCol_WindowBg, (20, 20, 30))
            dpg.add_theme_color(dpg.mvThemeCol_Button, (41, 128, 185))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonHovered, (52, 152, 219))
            dpg.add_theme_color(dpg.mvThemeCol_ButtonActive, (41, 128, 185))
            dpg.add_theme_color(dpg.mvThemeCol_SliderGrab, (52, 152, 219))
            dpg.add_theme_color(dpg.mvThemeCol_SliderGrabActive, (41, 128, 185))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBg, (30, 30, 40))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBgHovered, (40, 40, 50))
            dpg.add_theme_color(dpg.mvThemeCol_FrameBgActive, (50, 50, 60))
    
    dpg.bind_theme("global_theme")
    
    # Font setup - cross-platform
    with dpg.font_registry():
        # Try to load system font for larger percentage display
        try:
            if sys.platform == "win32":
                font_path = "C:/Windows/Fonts/segoeui.ttf"
            elif sys.platform == "darwin":  # macOS
                font_path = "/System/Library/Fonts/Helvetica.ttc"
            else:  # Linux
                font_path = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"
            
            if os.path.exists(font_path):
                # Large font for percentage display
                large_font = dpg.add_font(font_path, 72)
                dpg.bind_item_font("percentage_text", large_font)
        except:
            # If font loading fails, just use default (will still work, just smaller)
            print("⚠ Could not load custom font, using default")
            pass
    
    # Setup viewport
    dpg.create_viewport(
        title="Flex Sensor Gripper Control",
        width=600,
        height=700,
        resizable=False
    )
    
    dpg.setup_dearpygui()
    dpg.show_viewport()
    dpg.set_primary_window("main_window", True)


# ========== MAIN ==========
def main():
    print("Flex Sensor Gripper Control - Dear PyGui")
    print("=" * 50)
    print(f"Arduino Port: {ARDUINO_PORT}")
    print(f"Robot IP: {ROBOT_IP}")
    print("\nMake sure to update these values in the code!")
    print("\nStarting GUI...")
    
    create_gui()
    
    # Main loop
    while dpg.is_dearpygui_running():
        dpg.render_dearpygui_frame()
    
    # Cleanup
    if app_state.running:
        app_state.running = False
        time.sleep(0.5)
        
    try:
        if app_state.gripper:
            app_state.gripper.disconnect()
    except:
        pass
        
    try:
        if app_state.arduino:
            app_state.arduino.close()
    except:
        pass
    
    dpg.destroy_context()


if __name__ == "__main__":
    main()