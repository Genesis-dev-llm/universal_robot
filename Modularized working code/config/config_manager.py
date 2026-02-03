"""
Configuration management system
Handles loading, saving, and accessing configuration parameters
"""

import json
import os

class ConfigManager:
    """
    Manages persistent configuration via JSON file
    Handles loading, saving, and accessing configuration parameters
    """
    
    DEFAULT_CONFIG = {
        'remapping': {
            'enable_remapped_modes': True  # Enabled by default for finalized system
        },
        'safety': {
            'smoothing_factor': 0.3,
            'velocity_ramp_rate': 0.1,
            'deadzone': 3.0,
            'deadzone_ramp_width': 2.0,
            'max_velocity_scale': 1.0
        },
        'control': {
            'base_translation': 0.002,
            'base_rotation': 0.01,
            'vertical': 0.002,
            'tcp_translation': 0.0005,
            'tcp_vertical': 0.0005,
            'tcp_orientation': 0.005,
            'robot_orientation': 0.008
        },
        'imu_calibration': {
            'roll_offset': 0.0,
            'pitch_offset': 0.0,
            'yaw_offset': 0.0,
            'is_calibrated': False
        },
        'visualization': {
            'window_width': 1200,
            'window_height': 800,
            'camera_distance': 20.0
        },
        'speed_scaling': {
            'linear_scale': 0.1,  # Set to MIN_SCALE for startup safety
            'angular_scale': 0.1, # Set to MIN_SCALE for startup safety
            'min_scale': 0.1,
            'max_scale': 2.0,
            'step': 0.1
        },
        'gripper': {
            'enabled': True,
            'default_speed': 255,
            'default_force': 150,
            'deadzone': 3,
            'update_interval': 0.05
        }
    }
    
    def __init__(self, config_file='robot_config.json'):
        """
        Initialize configuration manager
        
        Args:
            config_file: Path to JSON configuration file
        """
        self.config_file = config_file
        self.config = self.load_config()
    
    def load_config(self):
        """Load configuration from file or create default"""
        if os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r') as f:
                    loaded_config = json.load(f)
                    # Merge with defaults to ensure all keys exist
                    config = self._deep_merge(self.DEFAULT_CONFIG.copy(), loaded_config)
                    print(f"Configuration loaded from {self.config_file}")
                    return config
            except Exception as e:
                print(f"Error loading config: {e}, using defaults")
                return self.DEFAULT_CONFIG.copy()
        else:
            print(f"No config file found, using defaults")
            self.save_config()  # Create default config file
            return self.DEFAULT_CONFIG.copy()
    
    def _deep_merge(self, base, override):
        """Deep merge two dictionaries"""
        for key, value in override.items():
            if key in base and isinstance(base[key], dict) and isinstance(value, dict):
                base[key] = self._deep_merge(base[key], value)
            else:
                base[key] = value
        return base
    
    def save_config(self):
        """Save current configuration to file"""
        try:
            with open(self.config_file, 'w') as f:
                json.dump(self.config, f, indent=4)
            return True
        except Exception as e:
            print(f"Error saving config: {e}")
            return False
    
    def get(self, section, key, default=None):
        """Get a configuration value with optional default"""
        return self.config.get(section, {}).get(key, default)
    
    def set(self, value, section, key):
        """Set a configuration value"""
        if section not in self.config:
            self.config[section] = {}
        self.config[section][key] = value
    
    def print_summary(self):
        """Print current configuration summary"""
        print("\n" + "="*70)
        print("CURRENT CONFIGURATION")
        print("="*70)
        for section, values in self.config.items():
            print(f"\n[{section.upper()}]")
            for key, value in values.items():
                print(f"  {key}: {value}")
        print("="*70 + "\n")


class RuntimeConfig:
    """Runtime configuration that can be modified during execution"""
    
    def __init__(self, config_manager):
        """
        Initialize runtime configuration from config manager
        
        Args:
            config_manager: ConfigManager instance
        """
        self.ENABLE_REMAPPED_MODES = config_manager.get('remapping', 'enable_remapped_modes', False)
        
        # Speed Control
        self.LINEAR_SPEED_SCALE = config_manager.get('speed_scaling', 'linear_scale', 0.1)
        self.ANGULAR_SPEED_SCALE = config_manager.get('speed_scaling', 'angular_scale', 0.1)
        self.MIN_SCALE = config_manager.get('speed_scaling', 'min_scale', 0.1)
        self.MAX_SCALE = config_manager.get('speed_scaling', 'max_scale', 2.0)
        self.STEP_SCALE = config_manager.get('speed_scaling', 'step', 0.1)
        
        # Gripper Control
        self.GRIPPER_ENABLED = config_manager.get('gripper', 'enabled', True)
        self.GRIPPER_SPEED = config_manager.get('gripper', 'default_speed', 255)
        self.GRIPPER_FORCE = config_manager.get('gripper', 'default_force', 150)
        
        # Clamp initial values to ensure they respect safety minimum
        self.LINEAR_SPEED_SCALE = max(self.MIN_SCALE, min(self.MAX_SCALE, self.LINEAR_SPEED_SCALE))
        self.ANGULAR_SPEED_SCALE = max(self.MIN_SCALE, min(self.MAX_SCALE, self.ANGULAR_SPEED_SCALE))