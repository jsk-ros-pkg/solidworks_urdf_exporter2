"""UI-independent state for a CAD-derived module under configuration.

The View (viser today, React/Three.js later) only *reads* this; the core API
mutates it; export consumes it.  Pydantic so it serializes to JSON for a
REST/WebSocket payload without change.
"""

from __future__ import annotations

from typing import Optional

from pydantic import BaseModel, Field


class JointEdit(BaseModel):
    """Interactive overlay edits for ONE joint -- the operations robot-compiler's
    GUI exposes (rename / limits / mimic / servo mapping).  All fields optional;
    only the set ones are applied on top of the CAD-derived URDF.

    Axis flip / joint-type change are intentionally NOT here: robot-compiler
    treats axis and type as fixed properties of the module URDF, so those are
    sw2robot.exporter's job (joint-config YAML), not an export-time overlay.
    """

    rename: Optional[str] = None
    lower: Optional[float] = None
    upper: Optional[float] = None
    mimic_joint: Optional[str] = None
    mimic_multiplier: float = 1.0
    mimic_offset: float = 0.0
    servo_id: Optional[int] = None
    direction: int = 1            # +1 normal, -1 reversed
    angle_offset: float = 0.0
    # Actuator limits (None = keep the CAD URDF default); effort N*m, velocity rad/s
    effort: Optional[float] = None
    velocity: Optional[float] = None
    servo_model: Optional[str] = None   # e.g. "HLS3606M" (drives profile auto-fill)
    # CAD-specific edits robot-compiler can't do (axis/type live in the URDF):
    flip_axis: bool = False       # negate the joint's <axis xyz>
    jtype: Optional[str] = None   # override joint type (revolute/continuous/...)


class RobotCompilerState(BaseModel):
    """Everything needed to configure + export one CAD module, GUI-free.

    ``joints``/``links``/``root_link`` are the parsed *base* URDF (as built by
    sw2robot.exporter).  ``edits`` is the interactive overlay, keyed by the joint's
    ORIGINAL name (so a rename never loses its anchor)."""

    robot_name: str
    urdf_path: str
    package_dir: str
    joints: list[dict] = Field(default_factory=list)
    links: list[dict] = Field(default_factory=list)
    root_link: Optional[str] = None
    edits: dict[str, JointEdit] = Field(default_factory=dict)

    def edit_for(self, joint_name: str) -> JointEdit:
        """Get (creating if needed) the overlay for ``joint_name``."""
        return self.edits.setdefault(joint_name, JointEdit())

    def effective_name(self, joint_name: str) -> str:
        e = self.edits.get(joint_name)
        return e.rename if (e and e.rename) else joint_name

    def movable_joints(self) -> list[dict]:
        return [j for j in self.joints
                if j.get("type") in ("revolute", "continuous", "prismatic")]
