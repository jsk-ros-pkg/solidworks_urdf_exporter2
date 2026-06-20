"""Serializable intermediate state (the cached SolidWorks extraction).

`extract` (slow, needs SolidWorks) writes a ``GraphState`` to ``graph.json``
plus the per-component meshes.  Everything after that -- choosing the base,
excluding parts, wiring joints, setting axes/limits/root frame, writing URDF --
operates on this JSON with NO SolidWorks, so iteration is instant.
"""

from __future__ import annotations

import numpy as np
from pydantic import BaseModel


class ComponentState(BaseModel):
    name: str                       # SolidWorks Name2 (e.g. "linkB-2")
    link_name: str                  # URDF-safe
    part_path: str | None = None
    is_subassembly: bool = False
    world: list[float]              # 16 floats, row-major 4x4 (local->world)
    fixed: bool = False
    # screw/bolt/nut/washer/pin -- standard hardware that should weld RIGIDLY to
    # whatever it fastens, never spin on its hole's concentric mate.  Set at
    # build time from the part's library folder + nomenclature (see
    # model.is_fastener_part); persisted so an extract can also pre-tag it.
    is_fastener: bool = False
    dof: int | None = None
    mesh_file: str | None = None    # relative path, e.g. "meshes/x.3dxml"
    material: str | None = None     # SolidWorks material name (e.g. "ABS")
    density: float | None = None    # kg/m^3 from that material
    # SolidWorks-native mass properties of the PART, in its own (part-local)
    # frame and SI units -- the same frame the mesh is exported in.  Preferred
    # over the mesh-derived estimate at build time (exact CAD geometry +
    # material/override, not a tessellation).  None on older extracts.
    sw_mass: float | None = None              # kg
    sw_com: list[float] | None = None         # centre of mass [x,y,z] (m)
    sw_inertia: list[float] | None = None     # (ixx,ixy,ixz,iyy,iyz,izz) about COM

    def world_matrix(self):
        return np.array(self.world, float).reshape(4, 4)


class MateGeo(BaseModel):
    """One mate occurrence with its full entity geometry (world coords).

    Parallel arrays, one slot per mate entity: ``etypes`` is the SolidWorks
    ``swMateEntityType_e`` (0 point, 1 line, 2 circle, 3 plane, 4 cylinder,
    5 sphere, 7 cone); ``dirs`` is the axis (line/cylinder) or normal (plane),
    zero when meaningless.  GetMates returns each mate once per component, so
    the same physical mate appears ~twice -- consumers dedup geometrically."""
    type: str                       # e.g. "CONCENTRIC"
    etypes: list[int | None] = []
    points: list[list[float]] = []  # world [x,y,z] per entity
    dirs: list[list[float]] = []    # world unit [x,y,z] per entity
    radii: list[float | None] = []
    # full Name2 of each entity's owning component (e.g. "body-1/armA-1");
    # lets a build-time sub-assembly expansion re-attach the mate to the
    # correct CHILD instead of the collapsed instance
    owners: list[str] = []


class MateEdge(BaseModel):
    a: str                          # component Name2
    b: str
    types: list[str]                # e.g. ["CONCENTRIC", "COINCIDENT"]
    axis_point: list[float] | None = None   # world, on the concentric axis
    axis_dir: list[float] | None = None      # world, unit
    # full per-mate geometry (newer extracts; None on graphs from older ones)
    mates: list[MateGeo] | None = None


class LimitJoint(BaseModel):
    """A SolidWorks LimitDistance/LimitAngle mate = a real slider/hinge.

    These carry the assembly's actual DOFs (the geometric classifier only sees a
    plain DISTANCE/ANGLE constraint and over-fixes them), so the build uses them
    to override the joint type/axis/limits on the ``a``--``b`` edge.  ``lower``/
    ``upper`` are travel relative to the assembled pose (m or rad)."""
    a: str                          # top-level component Name2
    b: str
    type: str                       # "prismatic" | "revolute"
    axis_point: list[float]
    axis_dir: list[float]
    lower: float
    upper: float


class SubGraph(BaseModel):
    """Internal structure of ONE sub-assembly part file, in ITS OWN frame.

    Stored once per unique .SLDASM path; every instance of that sub-assembly
    reuses it (compose with the instance transform).  Lets the build phase
    expand sub-assemblies whose internals actually move."""
    components: list[ComponentState] = []
    edges: list[MateEdge] = []
    ground: list[str] = []


class GraphState(BaseModel):
    """The raw CAD graph extracted from the assembly (UI/build independent)."""
    robot_name: str
    source_assembly: str
    components: list[ComponentState] = []
    edges: list[MateEdge] = []
    ground: list[str] = []          # components mated to the assembly itself
    assembly_mesh: str | None = None
    # part_path -> internals (newer extracts; empty on graphs from older ones)
    subassemblies: dict[str, SubGraph] = {}
    # full nested Name2 ("inst-1/child-2/...") -> row-major 4x4 in the ROOT
    # frame, for EVERY component at every depth.  Flexible sub-assembly
    # instances pose their internals differently per instance; expansion
    # prefers these actual worlds over (instance transform x local layout).
    deep_worlds: dict[str, list[float]] = {}
    # full nested Name2 of components that are HIDDEN in the assembly --
    # SolidWorks renders (and exports) without them, so the build drops them
    hidden: list[str] = []
    # LimitDistance/LimitAngle mates = the assembly's real sliders/hinges; the
    # build promotes these edges to prismatic/revolute (empty on older extracts)
    limit_joints: list[LimitJoint] = []

    def save(self, path):
        with open(path, "w", encoding="utf-8") as f:
            f.write(self.model_dump_json(indent=2))

    @classmethod
    def load(cls, path):
        with open(path, encoding="utf-8") as f:
            return cls.model_validate_json(f.read())
