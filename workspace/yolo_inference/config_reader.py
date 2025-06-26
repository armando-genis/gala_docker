import json
import numpy as np
import math
import os

class ConfigurationReader:
    def __init__(self, config_file_path="config.json"):
        """
        Initialize the configuration reader
        
        Args:
            config_file_path (str): Path to the JSON configuration file
        """
        self.config_file_path = config_file_path
        self.config_data = self.load_config()
    
    def load_config(self):
        """Load configuration from JSON file"""
        try:
            with open(self.config_file_path, 'r') as file:
                return json.load(file)
        except FileNotFoundError:
            raise FileNotFoundError(f"Configuration file '{self.config_file_path}' not found")
        except json.JSONDecodeError as e:
            raise ValueError(f"Invalid JSON in configuration file: {e}")
    
    def get_available_configurations(self):
        """Get list of available configuration names"""
        return list(self.config_data["configurations"].keys())
    
    def get_configuration(self, config_name):
        """
        Get configuration for a specific name (e.g., 'west', 'east')
        
        Args:
            config_name (str): Name of the configuration to load
            
        Returns:
            dict: Configuration dictionary with all parameters
        """
        if config_name not in self.config_data["configurations"]:
            available = self.get_available_configurations()
            raise ValueError(f"Configuration '{config_name}' not found. Available: {available}")
        
        config = self.config_data["configurations"][config_name].copy()
        
        # Convert model points to numpy arrays
        config["model_points_ladle"] = np.array(config["model_points_ladle"], dtype=np.float32)
        config["model_points_hook"] = np.array(config["model_points_hook"], dtype=np.float32)
        
        # Create rotation matrices from degrees
        ladle_rot = config["rotation_correction_ladle"]
        hook_rot = config["rotation_correction_hook"]
        
        config["R_combined_ladle"] = self._create_rotation_matrix(
            ladle_rot["rx_degrees"], 
            ladle_rot["ry_degrees"], 
            ladle_rot["rz_degrees"]
        )
        
        config["R_combined_hook"] = self._create_rotation_matrix(
            hook_rot["rx_degrees"], 
            hook_rot["ry_degrees"], 
            hook_rot["rz_degrees"]
        )
        
        return config
    
    def _create_rotation_matrix(self, rx_deg, ry_deg, rz_deg):
        """
        Create combined rotation matrix from X, Y, Z rotations in degrees
        
        Args:
            rx_deg, ry_deg, rz_deg (float): Rotation angles in degrees
            
        Returns:
            np.ndarray: 3x3 rotation matrix
        """
        rx_rad = math.radians(rx_deg)
        ry_rad = math.radians(ry_deg)
        rz_rad = math.radians(rz_deg)
        
        # Rotation around X axis
        Rx = np.array([[1, 0, 0],
                       [0, math.cos(rx_rad), -math.sin(rx_rad)],
                       [0, math.sin(rx_rad), math.cos(rx_rad)]])
        
        # Rotation around Y axis
        Ry = np.array([[math.cos(ry_rad), 0, math.sin(ry_rad)],
                       [0, 1, 0],
                       [-math.sin(ry_rad), 0, math.cos(ry_rad)]])
        
        # Rotation around Z axis
        Rz = np.array([[math.cos(rz_rad), -math.sin(rz_rad), 0],
                       [math.sin(rz_rad), math.cos(rz_rad), 0],
                       [0, 0, 1]])
        
        return Rx @ Ry @ Rz
    
    def print_configuration_summary(self, config_name):
        """Print a summary of the loaded configuration"""
        config = self.get_configuration(config_name)
        
        print(f"\n=== Configuration: {config_name.upper()} ===")
        print(f"Weights Hook:   {config['weights_path_hook']}")
        print(f"Weights Ladle:  {config['weights_path_ladle']}")
        print(f"Video Path:     {config['video_path']}")
        print(f"ladle point:     {config['ladle_point']}")
        print(f"hook point:     {config['hook_point']}")
        print(f"Camera Frame:   {config['camera_frame_id']}")
        print(f"Ladle Frame:    {config['ladle_frame_id']}")
        print(f"Hook Frame:     {config['hook_frame_id']}")
        print(f"Ladle Points:   {len(config['model_points_ladle'])} points")
        print(f"Hook Points:    {len(config['model_points_hook'])} points")
        print(f"Ladle Rotation: Rx={config['rotation_correction_ladle']['rx_degrees']}°, "
              f"Ry={config['rotation_correction_ladle']['ry_degrees']}°, "
              f"Rz={config['rotation_correction_ladle']['rz_degrees']}°")
        print(f"Hook Rotation:  Rx={config['rotation_correction_hook']['rx_degrees']}°, "
              f"Ry={config['rotation_correction_hook']['ry_degrees']}°, "
              f"Rz={config['rotation_correction_hook']['rz_degrees']}°")
        print("=" * 50)

# Example usage function
def load_detector_configuration(config_name, config_file="config.json"):
    """
    Convenience function to load a configuration
    
    Args:
        config_name (str): Configuration name ('west', 'east', etc.)
        config_file (str): Path to config file
        
    Returns:
        dict: Configuration dictionary ready to use
    """
    reader = ConfigurationReader(config_file)
    return reader.get_configuration(config_name)