"""
Configuration file generators for robot URDFs.

This module provides generators for:
- MoveIt2 configuration (SRDF, controllers, etc.)
- Gazebo configuration (world, plugins, etc.)
- Imitation Learning configuration (observation/action spaces)
- Servo mapping configuration (joint to servo ID mapping)
"""

from .export import export_all_configs
from .gazebo_generator import generate_gazebo_config
from .imitation_generator import generate_il_config
from .moveit_generator import generate_controllers_yaml
from .moveit_generator import generate_srdf
from .servo_mapping import generate_servo_mapping_yaml
from .urdf_parser import parse_urdf_content


__all__ = [
    "parse_urdf_content",
    "generate_srdf",
    "generate_controllers_yaml",
    "generate_gazebo_config",
    "generate_il_config",
    "generate_servo_mapping_yaml",
    "export_all_configs",
]
