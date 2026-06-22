"""UI-independent state for a CAD-derived module under configuration.

The View (viser today, React/Three.js later) only *reads* this; the core API
mutates it; export consumes it.  Pydantic so it serializes to JSON for a
REST/WebSocket payload without change.
"""

from __future__ import annotations

from pydantic import BaseModel, Field


class JointEdit(BaseModel):
    """Interactive overlay edits for ONE joint -- the operations robot-compiler's
    GUI exposes (rename / limits / mimic / servo mapping).  All fields optional;
    only the set ones are applied on top of the CAD-derived URDF.

    Axis flip / joint-type change are intentionally NOT here: robot-compiler
    treats axis and type as fixed properties of the module URDF, so those are
    sw2robot.exporter's job (joint-config YAML), not an export-time overlay.
    """

    rename: str | None = None
    lower: float | None = None
    upper: float | None = None
    mimic_joint: str | None = None
    mimic_multiplier: float = 1.0
    mimic_offset: float = 0.0
    servo_id: int | None = None
    direction: int = 1            # +1 normal, -1 reversed
    angle_offset: float = 0.0
    # Actuator limits (None = keep the CAD URDF default); effort N*m, velocity rad/s
    effort: float | None = None
    velocity: float | None = None
    servo_model: str | None = None   # e.g. "HLS3606M" (drives profile auto-fill)
    # CAD-specific edits robot-compiler can't do (axis/type live in the URDF):
    flip_axis: bool = False       # negate the joint's <axis xyz>
    jtype: str | None = None   # override joint type (revolute/continuous/...)


class LinkEdit(BaseModel):
    """Interactive overlay edits for ONE link -- the per-link properties the
    joint overlay can't carry: visual colour and the inertial mass / centre of
    mass / inertia tensor.  All fields optional; only the set ones are applied on
    top of the base URDF.  Keyed by the link's name in the base URDF."""

    color: str | None = None       # "#RRGGBB" (None = keep the URDF default)
    mass: float | None = None      # kg (None = keep)
    com: list[float] | None = None     # inertial origin xyz [x, y, z] (None = keep)
    # inertia tensor [ixx, ixy, ixz, iyy, iyz, izz] (None = keep)
    inertia: list[float] | None = None


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
    root_link: str | None = None
    edits: dict[str, JointEdit] = Field(default_factory=dict)
    # per-link overlay (colour / inertial), keyed by the link's ORIGINAL name
    link_edits: dict[str, LinkEdit] = Field(default_factory=dict)

    def edit_for(self, joint_name: str) -> JointEdit:
        """Get (creating if needed) the overlay for ``joint_name``."""
        return self.edits.setdefault(joint_name, JointEdit())

    def link_edit_for(self, link_name: str) -> LinkEdit:
        """Get (creating if needed) the per-link overlay for ``link_name``."""
        return self.link_edits.setdefault(link_name, LinkEdit())

    def effective_name(self, joint_name: str) -> str:
        e = self.edits.get(joint_name)
        return e.rename if (e and e.rename) else joint_name

    def effective_type(self, joint: dict) -> str | None:
        """The joint's type with the overlay's ``jtype`` override applied -- so a
        joint made movable (or fixed) via :func:`core.set_joint_type` is treated
        consistently by mimic validation and ``movable_joints``."""
        e = self.edits.get(joint["name"])
        return e.jtype if (e and e.jtype) else joint.get("type")

    def movable_joints(self) -> list[dict]:
        return [j for j in self.joints
                if self.effective_type(j) in ("revolute", "continuous", "prismatic")]
