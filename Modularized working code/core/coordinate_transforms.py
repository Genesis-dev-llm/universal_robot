"""
Universal Coordinate Transformation System
Handles all sensor-to-world-to-application transformations

Pipeline:
    BNO085 Sensor Frame → World Frame → Cube Frame
                                      → Robot Frame
"""

import numpy as np
import math
from config.constants import SENSOR_MOUNTING, CUBE_MAPPING, ROBOT_MAPPING

class CoordinateTransformer:
    """
    Centralized coordinate transformation system.
    Eliminates scattered axis mapping logic.
    """
    
    def __init__(self):
        """Initialize transformer with configuration"""
        self.sensor_config = SENSOR_MOUNTING
        self.cube_config = CUBE_MAPPING
        self.robot_config = ROBOT_MAPPING
    
    #==========================================================================
    # STAGE 1: BNO085 Sensor Frame → World Frame (Glove-Centric)
    #==========================================================================
    
    def sensor_to_world_euler(self, sensor_roll, sensor_pitch, sensor_yaw):
        """
        Convert raw BNO085 euler angles to world-frame orientation.
        World frame = natural glove orientation (forward/right/up).
        
        Args:
            sensor_roll, sensor_pitch, sensor_yaw: Raw BNO085 angles (degrees)
        
        Returns:
            dict: {
                'forward': float,  # Glove pointing forward/back
                'right': float,    # Glove tilting left/right  
                'up': float        # Glove tilting up/down
            }
        """
        sensor_data = {
            'roll': sensor_roll,
            'pitch': sensor_pitch,
            'yaw': sensor_yaw
        }
        
        # Apply sensor mounting transformation
        world_forward = self._get_sensor_value(
            sensor_data, self.sensor_config['world_forward'])
        world_right = self._get_sensor_value(
            sensor_data, self.sensor_config['world_right'])
        world_up = self._get_sensor_value(
            sensor_data, self.sensor_config['world_up'])
        
        return {
            'forward': world_forward,
            'right': world_right,
            'up': world_up
        }
    
    def sensor_to_world_quat(self, sensor_quat):
        """
        Convert BNO085 quaternion to world-frame quaternion.
        
        Args:
            sensor_quat: np.array [qi, qj, qk, qr] from BNO085
        
        Returns:
            np.array: [qi, qj, qk, qr] in world frame
        """
        # Apply component reordering based on sensor mounting
        order = self.sensor_config.get('quat_transform', [0, 1, 2, 3])
        sign = self.sensor_config.get('quat_signs', [1, 1, 1, 1])
        
        reordered = np.array([
            sensor_quat[order[0]] * sign[0],
            sensor_quat[order[1]] * sign[1],
            sensor_quat[order[2]] * sign[2],
            sensor_quat[order[3]] * sign[3]
        ])
        
        return reordered
    
    #==========================================================================
    # STAGE 2a: World Frame → Cube Visualization Frame
    #==========================================================================
    
    def world_to_cube_translation(self, world_euler):
        """
        Convert world-frame movements to cube visualization axes.
        
        Args:
            world_euler: dict from sensor_to_world_euler()
        
        Returns:
            dict: {'x': float, 'y': float, 'z': float} for cube velocity
        """
        result = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # Map world directions to cube axes
        result[self.cube_config['forward_axis']] = world_euler['forward']
        result[self.cube_config['right_axis']] = world_euler['right']
        result[self.cube_config['up_axis']] = world_euler['up']
        
        # Apply inversions for correct visual direction
        if self.cube_config['invert_forward']:
            result[self.cube_config['forward_axis']] *= -1
        if self.cube_config['invert_right']:
            result[self.cube_config['right_axis']] *= -1
        if self.cube_config['invert_up']:
            result[self.cube_config['up_axis']] *= -1
        
        return result
    
    def world_to_cube_rotation(self, world_quat):
        """
        Convert world-frame quaternion to cube rotation quaternion.
        Applies any cube-specific orientation adjustments.
        
        Args:
            world_quat: np.array [qi, qj, qk, qr] in world frame
        
        Returns:
            np.array: [qi, qj, qk, qr] for cube rendering
        """
        # Apply cube quaternion transformation if needed
        cube_order = self.cube_config.get('quat_transform', [0, 1, 2, 3])
        cube_signs = self.cube_config.get('quat_signs', [1, 1, 1, 1])
        
        cube_quat = np.array([
            world_quat[cube_order[0]] * cube_signs[0],
            world_quat[cube_order[1]] * cube_signs[1],
            world_quat[cube_order[2]] * cube_signs[2],
            world_quat[cube_order[3]] * cube_signs[3]
        ])
        
        return cube_quat
    
    #==========================================================================
    # STAGE 2b: World Frame → Robot Frame (UR Base Coordinates)
    #==========================================================================
    
    def world_to_robot_translation(self, world_euler, scale=1.0):
        """
        Convert world-frame movements to robot TCP translation.
        
        Args:
            world_euler: dict from sensor_to_world_euler()
            scale: Scaling factor (from control params)
        
        Returns:
            list: [dx, dy, dz] in robot base frame (meters)
        """
        result = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        # Map world directions to robot axes
        result[self.robot_config['forward_axis']] = world_euler['forward']
        result[self.robot_config['right_axis']] = world_euler['right']
        result[self.robot_config['up_axis']] = world_euler['up']
        
        # Apply inversions for correct robot direction
        if self.robot_config['invert_forward']:
            result[self.robot_config['forward_axis']] *= -1
        if self.robot_config['invert_right']:
            result[self.robot_config['right_axis']] *= -1
        if self.robot_config['invert_up']:
            result[self.robot_config['up_axis']] *= -1
        
        # Apply scaling and return as list
        return [result['x'] * scale, result['y'] * scale, result['z'] * scale]
    
    def world_to_robot_rotation(self, world_euler, scale=1.0):
        """
        Convert world-frame movements to robot orientation deltas.
        
        Args:
            world_euler: dict from sensor_to_world_euler()
            scale: Scaling factor (radians per degree)
        
        Returns:
            list: [drx, dry, drz] rotation deltas (radians)
        """
        result = {'rx': 0.0, 'ry': 0.0, 'rz': 0.0}
        
        # Map world movements to robot rotation axes
        # Semantic: forward/back = pitch, left/right = roll, twist = yaw
        result[self.robot_config['pitch_axis']] = world_euler['forward']
        result[self.robot_config['roll_axis']] = world_euler['right']
        result[self.robot_config['yaw_axis']] = world_euler['up']
        
        # Apply inversions
        if self.robot_config['invert_pitch']:
            result[self.robot_config['pitch_axis']] *= -1
        if self.robot_config['invert_roll']:
            result[self.robot_config['roll_axis']] *= -1
        if self.robot_config['invert_yaw']:
            result[self.robot_config['yaw_axis']] *= -1
        
        # Apply scaling and return as list
        return [result['rx'] * scale, result['ry'] * scale, result['rz'] * scale]
    
    def world_to_robot_quat(self, world_quat):
        """
        Convert world-frame quaternion to robot orientation quaternion.
        
        Args:
            world_quat: np.array [qi, qj, qk, qr] in world frame
        
        Returns:
            np.array: [qi, qj, qk, qr] for robot orientation
        """
        # Apply robot quaternion transformation
        robot_order = self.robot_config.get('quat_transform', [0, 1, 2, 3])
        robot_signs = self.robot_config.get('quat_signs', [1, 1, 1, 1])
        
        robot_quat = np.array([
            world_quat[robot_order[0]] * robot_signs[0],
            world_quat[robot_order[1]] * robot_signs[1],
            world_quat[robot_order[2]] * robot_signs[2],
            world_quat[robot_order[3]] * robot_signs[3]
        ])
        
        return robot_quat
    
    #==========================================================================
    # HELPER METHODS
    #==========================================================================
    
    def _get_sensor_value(self, sensor_data, axis_spec):
        """
        Extract sensor value with optional negation.
        
        Args:
            sensor_data: dict with 'roll', 'pitch', 'yaw'
            axis_spec: e.g., 'roll' or '-pitch'
        
        Returns:
            float: sensor value (negated if '-' prefix)
        """
        if axis_spec.startswith('-'):
            axis_name = axis_spec[1:]
            return -sensor_data[axis_name]
        else:
            return sensor_data[axis_spec]
    
    def print_config_summary(self):
        """Display current transformation configuration"""
        print("\n" + "="*70)
        print("COORDINATE TRANSFORMATION CONFIGURATION")
        print("="*70)
        
        print("\n[STAGE 1: SENSOR MOUNTING]")
        print("  How BNO085 maps to glove world frame:")
        for key, val in self.sensor_config.items():
            if not key.startswith('quat'):  # Skip quaternion details for clarity
                print(f"    {key:20s} = {val}")
        
        print("\n[STAGE 2a: CUBE VISUALIZATION]")
        print("  How world frame maps to 3D cube:")
        for key, val in self.cube_config.items():
            if not key.startswith('quat'):  # Skip quaternion details
                print(f"    {key:20s} = {val}")
        
        print("\n[STAGE 2b: ROBOT CONTROL]")
        print("  How world frame maps to UR robot:")
        for key, val in self.robot_config.items():
            if not key.startswith('quat'):  # Skip quaternion details
                print(f"    {key:20s} = {val}")
        
        print("="*70 + "\n")
    
    def validate_config(self):
        """
        Validate configuration for common mistakes.
        
        Returns:
            tuple: (is_valid: bool, errors: list)
        """
        errors = []
        
        # Check sensor mounting
        valid_axes = ['roll', 'pitch', 'yaw', '-roll', '-pitch', '-yaw']
        for key in ['world_forward', 'world_right', 'world_up']:
            if self.sensor_config[key] not in valid_axes:
                errors.append(f"Invalid sensor axis: {key} = {self.sensor_config[key]}")
        
        # Check cube mapping
        valid_cube_axes = ['x', 'y', 'z']
        for key in ['forward_axis', 'right_axis', 'up_axis']:
            if self.cube_config[key] not in valid_cube_axes:
                errors.append(f"Invalid cube axis: {key} = {self.cube_config[key]}")
        
        # Check robot mapping
        valid_robot_axes = ['x', 'y', 'z']
        for key in ['forward_axis', 'right_axis', 'up_axis']:
            if self.robot_config[key] not in valid_robot_axes:
                errors.append(f"Invalid robot axis: {key} = {self.robot_config[key]}")
        
        valid_rotation_axes = ['rx', 'ry', 'rz']
        for key in ['pitch_axis', 'roll_axis', 'yaw_axis']:
            if self.robot_config[key] not in valid_rotation_axes:
                errors.append(f"Invalid robot rotation axis: {key} = {self.robot_config[key]}")
        
        return (len(errors) == 0, errors)


# Convenience function for quick access
def create_transformer():
    """Factory function to create and validate transformer"""
    transformer = CoordinateTransformer()
    is_valid, errors = transformer.validate_config()
    
    if not is_valid:
        print("⚠️  WARNING: Configuration validation failed!")
        for error in errors:
            print(f"  - {error}")
        print()
    
    return transformer