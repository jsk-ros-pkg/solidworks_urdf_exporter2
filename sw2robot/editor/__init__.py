"""sw2robot.editor -- the headless core bridging sw2robot.exporter
(CAD->module) and the ROS config export.

Design (a standing directive):
- **State**: ``RobotCompilerState`` (Pydantic) is the single source of truth for
  a CAD-derived module being configured.  It serializes to JSON, so it moves to
  a REST/WebSocket payload unchanged.
- **Core API** (``core``): pure functions that mutate State / produce artifacts.
  A web-editor handler or a FastAPI endpoint is a *thin* wrapper over these.
- **Headless**: the whole pipeline (CAD build -> edit -> ROS export) runs from
  the CLI (``python -m sw2robot.editor``) with no GUI and no SolidWorks.

It uses the *pure* halves: ``sw2robot.exporter.export.build`` (no SolidWorks)
and the vendored ``sw2robot.editor._vendor.rc_config`` (ROS/MoveIt/Gazebo
config generators).
"""

from . import core
from .state import JointEdit, RobotCompilerState

__all__ = ["JointEdit", "RobotCompilerState", "core"]
