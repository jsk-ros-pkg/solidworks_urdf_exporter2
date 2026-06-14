"""
Configuration export utilities.

Creates ZIP archives containing all configuration files.
"""

import io
from typing import Any
import zipfile

from .gazebo_generator import generate_gazebo_config
from .gazebo_generator import generate_ros2_control_xacro
from .imitation_generator import generate_il_config
from .moveit_generator import generate_controllers_yaml
from .moveit_generator import generate_srdf
from .servo_mapping import generate_servo_mapping_yaml


def export_all_configs(
    urdf_content: str,
    joints: list[dict[str, Any]],
    servo_mappings: list[dict[str, Any]],
    planning_groups: list[dict[str, Any]],
    controllers: list[dict[str, Any]],
    disabled_collision_pairs: list[tuple[str, str]],
    gazebo_physics: dict[str, Any],
    gazebo_plugins: list[dict[str, Any]],
    il_observation: dict[str, Any],
    il_action: dict[str, Any],
    robot_name: str = "robot",
    export_options: dict[str, bool] | None = None,
) -> bytes:
    """
    Export all configuration files as a ZIP archive.

    Parameters
    ----------
    urdf_content : str
        Original URDF content.
    joints : list
        Parsed joint information.
    servo_mappings : list
        Servo mapping configurations.
    planning_groups : list
        MoveIt planning group configurations.
    controllers : list
        Controller configurations.
    disabled_collision_pairs : list
        List of disabled collision link pairs.
    gazebo_physics : dict
        Gazebo physics settings.
    gazebo_plugins : list
        Gazebo plugin configurations.
    il_observation : dict
        IL observation space configuration.
    il_action : dict
        IL action space configuration.
    robot_name : str
        Name of the robot for file naming.

    Returns
    -------
    bytes
        ZIP file contents as bytes.
    """
    # Default export options if not provided
    if export_options is None:
        export_options = {
            "includeUrdf": True,
            "includeServoMapping": True,
            "includeMoveIt": True,
            "includeGazebo": True,
            "includeImitationLearning": True,
        }

    # Create in-memory ZIP file
    zip_buffer = io.BytesIO()

    with zipfile.ZipFile(zip_buffer, "w", zipfile.ZIP_DEFLATED) as zf:
        # URDF (original)
        if export_options.get("includeUrdf", True):
            zf.writestr(f"{robot_name}/urdf/{robot_name}.urdf", urdf_content)

        # Servo mapping
        if export_options.get("includeServoMapping", True):
            servo_yaml = generate_servo_mapping_yaml(servo_mappings)
            zf.writestr(f"{robot_name}/config/servo_mapping.yaml", servo_yaml)

        # MoveIt SRDF and controllers
        if export_options.get("includeMoveIt", True):
            srdf_content = generate_srdf(
                robot_name=robot_name,
                planning_groups=planning_groups,
                disabled_collision_pairs=[tuple(p) for p in disabled_collision_pairs],
            )
            zf.writestr(f"{robot_name}/config/{robot_name}.srdf", srdf_content)

            controllers_yaml = generate_controllers_yaml(controllers)
            zf.writestr(f"{robot_name}/config/controllers.yaml", controllers_yaml)

        # Gazebo config
        if export_options.get("includeGazebo", True):
            gazebo_config = generate_gazebo_config(gazebo_physics, gazebo_plugins)
            zf.writestr(f"{robot_name}/config/gazebo.xml", gazebo_config)

            ros2_control_xacro = generate_ros2_control_xacro(joints)
            zf.writestr(f"{robot_name}/urdf/ros2_control.xacro", ros2_control_xacro)

        # IL config
        if export_options.get("includeImitationLearning", True):
            il_config = generate_il_config(il_observation, il_action, joints)
            zf.writestr(f"{robot_name}/config/il_config.yaml", il_config)

        # README (always include)
        readme = _generate_readme(robot_name, export_options)
        zf.writestr(f"{robot_name}/README.md", readme)

    return zip_buffer.getvalue()


def _generate_readme(robot_name: str, export_options: dict[str, bool]) -> str:
    """Generate a README file for the configuration package."""
    contents = []
    usage_sections = []

    if export_options.get("includeUrdf", True):
        contents.append(f"- `urdf/{robot_name}.urdf` - Robot URDF description")

    if export_options.get("includeGazebo", True):
        contents.append("- `urdf/ros2_control.xacro` - ros2_control hardware interface configuration")
        contents.append("- `config/gazebo.xml` - Gazebo physics and plugin configuration")
        usage_sections.append(f"""### Gazebo

Include the ros2_control xacro in your robot description:

```xml
<xacro:include filename="$(find {robot_name})/urdf/ros2_control.xacro" />
```""")

    if export_options.get("includeServoMapping", True):
        contents.append("- `config/servo_mapping.yaml` - Joint to servo ID mapping")

    if export_options.get("includeMoveIt", True):
        contents.append(f"- `config/{robot_name}.srdf` - MoveIt2 SRDF (semantic robot description)")
        contents.append("- `config/controllers.yaml` - ros2_control controller configuration")
        usage_sections.append(f"""### MoveIt2

Include the SRDF in your MoveIt2 launch files:

```python
srdf_file = os.path.join(pkg_share, 'config', '{robot_name}.srdf')
```

### Controllers

Load the controller configuration:

```yaml
ros2_control_node:
  ros__parameters:
    robot_description: $(command 'cat $(find {robot_name})/urdf/{robot_name}.urdf')
```""")

    if export_options.get("includeImitationLearning", True):
        contents.append("- `config/il_config.yaml` - Imitation Learning configuration")
        usage_sections.append("""### Imitation Learning

Load the IL configuration in your training script:

```python
import yaml
with open('config/il_config.yaml') as f:
    config = yaml.safe_load(f)
```""")

    contents_str = "\n".join(contents)
    usage_str = "\n\n".join(usage_sections)

    return f"""# {robot_name} Configuration Package

Generated by Robot Compiler.

## Contents

{contents_str}

## Usage

{usage_str}
"""
