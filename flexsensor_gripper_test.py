"""
FLEX SENSOR GRIPPER CONTROL - GUI VERSION
==========================================
Controls Hand-E gripper with flex sensor input
Shows live percentage and force/speed controls

Requirements:
    pip install pyserial

Usage:
    1. Upload flex_sensor_final.ino to Arduino
    2. Update ARDUINO_PORT and ROBOT_IP below
    3. Run: python flex_gripper_gui.py
"""

import socket
import serial
import time
import threading
import tkinter as tk
from tkinter import ttk

# ========== CONFIGURATION ==========
ARDUINO_PORT = "/dev/ttyUSB0"  # Change to your port (COM3 on Windows, /dev/ttyUSB0 on Linux)
ARDUINO_BAUD = 115200
ROBOT_IP = "192.168.1.191"     # Your robot IP

# Dead zone to prevent jitter
DEAD_ZONE = 3  # percent

# Spike rejection threshold
SPIKE_THRESHOLD = 10  # percent - ignore jumps larger than this


# ========== GRIPPER CONTROLLER ==========
class HandEGripper:
    """Simple Hand-E gripper controller via socket port 63352"""
    
    def __init__(self, robot_ip):
        self.robot_ip = robot_ip
        self.socket = None
        
    def connect(self):
        """Connect to gripper socket server"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.robot_ip, 63352))
            self.socket.settimeout(2.0)
            print(f"✓ Connected to gripper at {self.robot_ip}:63352")
            return True
        except Exception as e:
            print(f"✗ Failed to connect: {e}")
            return False
        
    def send_command(self, cmd):
        """Send command to gripper"""
        if self.socket:
            try:
                self.socket.send(f"{cmd}\n".encode('utf-8'))
                time.sleep(0.01)
            except Exception as e:
                print(f"Error sending command: {e}")
        
    def activate(self):
        """Activate the gripper"""
        print("Activating gripper...")
        self.send_command("SET ACT 1")
        time.sleep(0.5)
        self.send_command("SET GTO 1")
        time.sleep(1.5)
        print("✓ Gripper activated")
        
    def move_to(self, position, speed=255, force=255):
        """
        Move gripper to position
        position: 0 (open) to 255 (closed)
        speed: 0-255
        force: 0-255
        """
        self.send_command(f"SET SPE {speed}")
        self.send_command(f"SET FOR {force}")
        self.send_command(f"SET POS {position}")
        
    def open(self, speed=255, force=255):
        """Open gripper"""
        self.move_to(0, speed, force)
        
    def close(self, speed=255, force=255):
        """Close gripper"""
        self.move_to(255, speed, force)
        
    def disconnect(self):
        """Close connection"""
        if self.socket:
            self.socket.close()
            print("✓ Disconnected")


# ========== GUI APPLICATION ==========
class FlexGripperGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Flex Sensor Gripper Control")
        self.root.geometry("500x400")
        self.root.resizable(False, False)
        
        # State variables
        self.arduino = None
        self.gripper = None
        self.running = False
        self.current_percentage = 0
        self.last_sent_percentage = -999
        self.last_valid_percentage = 0  # For spike rejection
        self.first_reading = True  # Accept first reading unconditionally
        
        # Control variables
        self.speed_var = tk.IntVar(value=255)
        self.force_var = tk.IntVar(value=150)
        
        self.setup_ui()
        
    def setup_ui(self):
        """Create the GUI layout"""
        
        # Title
        title = tk.Label(self.root, text="Flex Sensor → Hand-E Gripper", 
                        font=("Arial", 18, "bold"))
        title.pack(pady=10)
        
        # Status frame
        status_frame = tk.Frame(self.root)
        status_frame.pack(pady=10)
        
        self.status_label = tk.Label(status_frame, text="Status: Not Connected", 
                                     font=("Arial", 12), fg="red")
        self.status_label.pack()
        
        # Percentage display (BIG)
        percentage_frame = tk.Frame(self.root, bg="#2c3e50", bd=5, relief="ridge")
        percentage_frame.pack(pady=20, padx=20, fill="x")
        
        tk.Label(percentage_frame, text="Gripper Position", 
                font=("Arial", 12), bg="#2c3e50", fg="white").pack(pady=5)
        
        self.percentage_label = tk.Label(percentage_frame, text="0%", 
                                        font=("Arial", 48, "bold"), 
                                        bg="#2c3e50", fg="#3498db")
        self.percentage_label.pack(pady=10)
        
        # Progress bar
        self.progress_bar = ttk.Progressbar(percentage_frame, length=400, 
                                           mode='determinate', maximum=100)
        self.progress_bar.pack(pady=10, padx=20)
        
        # Control sliders
        controls_frame = tk.Frame(self.root)
        controls_frame.pack(pady=20, padx=20, fill="x")
        
        # Speed control
        speed_frame = tk.Frame(controls_frame)
        speed_frame.pack(fill="x", pady=5)
        
        tk.Label(speed_frame, text="Speed:", font=("Arial", 10, "bold"), 
                width=8, anchor="w").pack(side="left")
        self.speed_slider = tk.Scale(speed_frame, from_=50, to=255, 
                                     orient="horizontal", variable=self.speed_var,
                                     length=300)
        self.speed_slider.pack(side="left", padx=10)
        self.speed_value_label = tk.Label(speed_frame, text="255", 
                                         font=("Arial", 10), width=4)
        self.speed_value_label.pack(side="left")
        
        # Force control
        force_frame = tk.Frame(controls_frame)
        force_frame.pack(fill="x", pady=5)
        
        tk.Label(force_frame, text="Force:", font=("Arial", 10, "bold"), 
                width=8, anchor="w").pack(side="left")
        self.force_slider = tk.Scale(force_frame, from_=20, to=255, 
                                     orient="horizontal", variable=self.force_var,
                                     length=300)
        self.force_slider.pack(side="left", padx=10)
        self.force_value_label = tk.Label(force_frame, text="150", 
                                         font=("Arial", 10), width=4)
        self.force_value_label.pack(side="left")
        
        # Update slider labels
        self.speed_var.trace_add("write", self.update_slider_labels)
        self.force_var.trace_add("write", self.update_slider_labels)
        
        # Buttons
        button_frame = tk.Frame(self.root)
        button_frame.pack(pady=20)
        
        self.connect_btn = tk.Button(button_frame, text="Connect", 
                                     command=self.connect_devices,
                                     bg="#27ae60", fg="white", 
                                     font=("Arial", 12, "bold"),
                                     width=10)
        self.connect_btn.pack(side="left", padx=5)
        
        self.disconnect_btn = tk.Button(button_frame, text="Disconnect", 
                                       command=self.disconnect_devices,
                                       bg="#e74c3c", fg="white", 
                                       font=("Arial", 12, "bold"),
                                       width=10, state="disabled")
        self.disconnect_btn.pack(side="left", padx=5)
        
    def update_slider_labels(self, *args):
        """Update slider value labels"""
        self.speed_value_label.config(text=str(self.speed_var.get()))
        self.force_value_label.config(text=str(self.force_var.get()))
        
    def connect_devices(self):
        """Connect to Arduino and gripper"""
        self.status_label.config(text="Status: Connecting...", fg="orange")
        self.root.update()
        
        # Connect to Arduino
        try:
            self.arduino = serial.Serial(ARDUINO_PORT, ARDUINO_BAUD, timeout=1)
            time.sleep(2)
            print("✓ Arduino connected")
            
            # Wait for READY signal
            timeout = time.time() + 5
            while time.time() < timeout:
                line = self.arduino.readline().decode('utf-8').strip()
                if line == "READY":
                    print("✓ Arduino ready")
                    break
                time.sleep(0.1)
                
        except Exception as e:
            self.status_label.config(text=f"Status: Arduino Error - {e}", fg="red")
            return
        
        # Connect to gripper
        self.gripper = HandEGripper(ROBOT_IP)
        if not self.gripper.connect():
            self.status_label.config(text="Status: Gripper Connection Failed", fg="red")
            self.arduino.close()
            return
        
        self.gripper.activate()
        
        # Start control loop
        self.running = True
        self.last_valid_percentage = 0  # Reset for new session
        self.first_reading = True  # Accept first reading
        self.control_thread = threading.Thread(target=self.control_loop, daemon=True)
        self.control_thread.start()
        
        # Update UI
        self.status_label.config(text="Status: Connected & Active", fg="green")
        self.connect_btn.config(state="disabled")
        self.disconnect_btn.config(state="normal")
        
    def disconnect_devices(self):
        """Disconnect from devices"""
        self.running = False
        time.sleep(0.5)
        
        if self.gripper:
            self.gripper.open()
            time.sleep(1)
            self.gripper.disconnect()
            
        if self.arduino:
            self.arduino.close()
            
        self.status_label.config(text="Status: Disconnected", fg="red")
        self.connect_btn.config(state="normal")
        self.disconnect_btn.config(state="disabled")
        
    def control_loop(self):
        """Main control loop - runs in separate thread"""
        while self.running:
            try:
                # Read from Arduino
                line = self.arduino.readline().decode('utf-8').strip()
                
                if line.startswith("ANGLE:"):
                    percentage = int(line.split(":")[1])
                    
                    # SPIKE REJECTION: Ignore crazy jumps (except first reading)
                    if not self.first_reading:
                        spike_diff = abs(percentage - self.last_valid_percentage)
                        if spike_diff > SPIKE_THRESHOLD:
                            print(f"⚠ Spike rejected: {self.last_valid_percentage}% → {percentage}% (jump: {spike_diff}%)")
                            continue  # Ignore this reading
                    else:
                        self.first_reading = False
                    
                    # Valid reading - update last valid
                    self.last_valid_percentage = percentage
                    self.current_percentage = percentage
                    
                    # Update GUI
                    self.root.after(0, self.update_display, percentage)
                    
                    # Check dead zone
                    if abs(percentage - self.last_sent_percentage) < DEAD_ZONE:
                        continue
                    
                    self.last_sent_percentage = percentage
                    
                    # Map percentage to gripper position (0-255)
                    gripper_pos = int((percentage / 100.0) * 255)
                    
                    # Send to gripper with current speed/force settings
                    speed = self.speed_var.get()
                    force = self.force_var.get()
                    self.gripper.move_to(gripper_pos, speed, force)
                    
            except Exception as e:
                print(f"Control loop error: {e}")
                time.sleep(0.1)
                
    def update_display(self, percentage):
        """Update the percentage display"""
        self.percentage_label.config(text=f"{percentage}%")
        self.progress_bar['value'] = percentage
        
        # Color coding
        if percentage < 33:
            color = "#27ae60"  # Green - open
        elif percentage < 66:
            color = "#f39c12"  # Orange - mid
        else:
            color = "#e74c3c"  # Red - closed
            
        self.percentage_label.config(fg=color)


# ========== MAIN ==========
def main():
    root = tk.Tk()
    app = FlexGripperGUI(root)
    
    def on_closing():
        if app.running:
            app.disconnect_devices()
        root.destroy()
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    root.mainloop()


if __name__ == "__main__":
    print("Flex Sensor Gripper Control GUI")
    print("=" * 50)
    print(f"Arduino Port: {ARDUINO_PORT}")
    print(f"Robot IP: {ROBOT_IP}")
    print("\nMake sure to update these values in the code!")
    print("\nStarting GUI...")
    print()
    
    main()