"""Write a sw2robot package from code: parts, poses and joints -> graph.json.

For robots modelled in code (build123d, trimesh, or anything that writes a
mesh), or proposed by an agent, there are no mates to infer joints from --
the author knows them.  So they are declared::

    from sw2robot.exporter.graph_builder import GraphBuilder

    g = GraphBuilder("arm", unit="mm", cad="build123d")
    g.add_part("base", base_shape, fixed=True, density=1200)
    g.add_part("link1", link_shape, pose=T_link1, mass=0.12)
    g.add_joint("base", "link1", "revolute", axis_dir=[0, 0, 1],
                axis_point=[0, 0, 40], lower=-1.57, upper=1.57)
    pkg_dir = g.write("output")

``unit`` applies to meshes, poses and axis points alike; the package is
written in metres.  Inertia comes from each mesh and its density or target
mass at build time.
"""
from __future__ import annotations

import os
import tempfile

import numpy as np

from .. import __version__
from .inertia import MM_TO_M
from .model import safe_name
from .state import (
    ComponentState,
    DeclaredJoint,
    GraphState,
    SourceInfo,
)

GRAPH_FILE = "graph.json"
UNIT_SCALE = {"m": 1.0, "mm": MM_TO_M}


def _load_geometry(geometry, scale):
    """A metre ``trimesh.Trimesh`` from a mesh, a mesh file or a build123d shape."""
    import trimesh

    if isinstance(geometry, trimesh.Trimesh):
        mesh = geometry.copy()
    elif isinstance(geometry, (str, os.PathLike)):
        mesh = trimesh.load(os.fspath(geometry), force="mesh")
    elif hasattr(geometry, "wrapped"):             # a build123d Shape
        from build123d import export_stl
        with tempfile.TemporaryDirectory() as tmp:
            path = os.path.join(tmp, "shape.stl")
            if not export_stl(geometry, path):
                raise ValueError("build123d could not export the shape")
            mesh = trimesh.load(path, force="mesh")
    else:
        raise TypeError(f"unsupported geometry {type(geometry).__name__}: pass a "
                        "trimesh.Trimesh, a mesh file path or a build123d shape")
    if scale != 1.0:
        mesh.apply_scale(scale)
    mesh.units = "meter"
    return mesh


class GraphBuilder:
    """Collects parts and joints and writes them as a sw2robot package.

    Parameters
    ----------
    robot_name : str
        Robot and package name.
    unit : {'m', 'mm'}
        Length unit of every mesh, pose and axis point passed in.
    cad : str
        What produced the geometry, recorded as ``graph.source.cad``.
    """

    def __init__(self, robot_name, unit="m", cad="code"):
        if unit not in UNIT_SCALE:
            raise ValueError(f"unit must be one of {sorted(UNIT_SCALE)}")
        self.robot_name = safe_name(robot_name)
        self.scale = UNIT_SCALE[unit]
        self.cad = cad
        self._parts = {}
        self._joints = []

    def add_part(self, name, geometry, pose=None, *, fixed=False, mass=None,
                 density=None):
        """Add a rigid part.

        Parameters
        ----------
        name : str
            Unique part name; the URDF link name is its sanitised form.
        geometry : trimesh.Trimesh | str | build123d.Shape
            The part in its own frame.
        pose : array_like (4, 4) | None
            Part frame -> robot frame (identity when None).
        fixed : bool
            Grounded.  A single fixed part is the robot's base.
        mass : float | None
            Target mass in kg; the mesh-derived inertia is scaled to it.
        density : float | None
            Density in kg/m^3 for the mesh-derived inertia.

        Returns
        -------
        str
            ``name``.
        """
        if name in self._parts:
            raise ValueError(f"part {name!r} already added")
        link = safe_name(name)
        if any(p["link"] == link for p in self._parts.values()):
            raise ValueError(f"part {name!r} sanitises to link {link!r}, "
                             "which another part already uses")
        if mass is not None and density is not None:
            raise ValueError(f"part {name!r}: give mass or density, not both")
        world = np.eye(4) if pose is None else np.array(pose, float)
        if world.shape != (4, 4):
            raise ValueError(f"part {name!r}: pose must be 4x4")
        world[:3, 3] *= self.scale
        self._parts[name] = {
            "link": link, "mesh": _load_geometry(geometry, self.scale),
            "world": world, "fixed": fixed, "mass": mass, "density": density}
        return name

    def add_joint(self, parent, child, joint_type, axis_dir=None,
                  axis_point=None, lower=None, upper=None, name=None,
                  source="user"):
        """Declare the joint between two added parts.

        ``axis_dir`` is the direction ``child`` moves positively relative to
        ``parent``; ``lower``/``upper`` are travel from the poses given (radians,
        or ``unit`` for a prismatic joint).  See
        :class:`sw2robot.exporter.state.DeclaredJoint`.
        """
        for part in (parent, child):
            if part not in self._parts:
                raise ValueError(f"joint {name or joint_type!r}: no part "
                                 f"{part!r}")
        if joint_type == "prismatic":
            lower = None if lower is None else lower * self.scale
            upper = None if upper is None else upper * self.scale
        point = (None if axis_point is None
                 else [float(x) * self.scale for x in axis_point])
        self._joints.append(DeclaredJoint(
            parent=parent, child=child, type=joint_type, axis_dir=axis_dir,
            axis_point=point, lower=lower, upper=upper, name=name,
            source=source))

    def write(self, out_dir):
        """Write ``<out_dir>/<robot_name>/`` and return that directory."""
        if not self._parts:
            raise ValueError("no parts added")
        pkg_dir = os.path.join(os.path.abspath(out_dir), self.robot_name)
        meshes_dir = os.path.join(pkg_dir, "meshes")
        os.makedirs(meshes_dir, exist_ok=True)
        components = []
        ground = [n for n, p in self._parts.items() if p["fixed"]]
        for name, part in self._parts.items():
            rel = f"meshes/{part['link']}.stl"
            part["mesh"].export(os.path.join(pkg_dir, rel))
            components.append(ComponentState(
                name=name, link_name=part["link"], part_path=None,
                world=[float(x) for x in part["world"].flatten()],
                fixed=part["fixed"], mesh_file=rel,
                density=part["density"], mass_target=part["mass"]))
        graph = GraphState(
            robot_name=self.robot_name,
            source_assembly=f"{self.cad}:{self.robot_name}",
            source=SourceInfo(cad=self.cad, extractor=f"sw2robot {__version__}"),
            components=components, joints=list(self._joints),
            ground=ground, base=ground[0] if len(ground) == 1 else None)
        graph.save(os.path.join(pkg_dir, GRAPH_FILE))
        return pkg_dir
