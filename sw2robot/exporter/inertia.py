"""Compute a link's inertial (mass, centre of mass, inertia tensor) from its
mesh, so the generated URDF carries real dynamics instead of a placeholder.

Units: the SolidWorks-exported meshes are in **millimetres** (a servo body is
~32 mm), while the URDF skeleton is in **metres**.  We therefore scale the mesh
by ``MM_TO_M`` before applying density, so mass comes out in kg and the inertia
tensor in kg.m^2 -- directly usable by Gazebo / MoveIt.

The mesh is moved into the *link* frame (via the visual origin) before the
integral, so the returned centre of mass and tensor are expressed in link
coordinates -- exactly what the URDF ``<inertial>`` element wants.

Many CAD meshes are NOT watertight; trimesh's volume integral is then
unreliable.  We fall back to the convex hull (always watertight, a mild
over-estimate) and, failing that, the oriented bounding box, and we report which
links used an approximation rather than silently emitting wrong numbers.
"""

from __future__ import annotations

import os

import numpy as np

MM_TO_M = 0.001
DEFAULT_DENSITY = 1000.0  # kg/m^3 -- generic light part; override per build


def _rpy_matrix(rpy):
    """4x4 homogeneous rotation for a URDF ``rpy`` (extrinsic X-Y-Z)."""
    from scipy.spatial.transform import Rotation

    T = np.eye(4)
    T[:3, :3] = Rotation.from_euler("xyz", rpy).as_matrix()
    return T


def _solid_properties(mesh, density):
    """(mass, com(3), I 3x3) for ``mesh`` treated as a solid of ``density``.

    Returns ``None`` if the mesh has no usable volume."""
    if mesh is None or not hasattr(mesh, "vertices") or len(mesh.vertices) == 0:
        return None
    vol = float(getattr(mesh, "volume", 0.0) or 0.0)
    if not np.isfinite(vol) or vol <= 0:
        return None
    mesh.density = float(density)
    mass = float(mesh.mass)
    com = np.asarray(mesh.center_mass, dtype=float)
    inertia = np.asarray(mesh.moment_inertia, dtype=float)  # about COM, link axes
    if not (np.isfinite(mass) and mass > 0 and np.all(np.isfinite(com))
            and np.all(np.isfinite(inertia))):
        return None
    return mass, com, inertia


def link_inertial(mesh_path, visual_xyz, visual_rpy,
                  density=DEFAULT_DENSITY, scale=MM_TO_M):
    """Inertial for one link, computed in the link frame.

    Parameters
    ----------
    mesh_path : str | None
        Absolute path to the link's mesh (any format trimesh can load).
    visual_xyz, visual_rpy : sequence of 3 floats
        The link's visual origin (mesh -> link), in metres / radians.
    density : float
        Material density in kg/m^3.
    scale : float
        Mesh-unit -> metre factor (SolidWorks mm exports -> 0.001).

    Returns
    -------
    dict | None
        ``{mass, com(3), inertia(ixx,ixy,ixz,iyy,iyz,izz), method}`` where
        ``method`` is one of ``"mesh"``, ``"hull"``, ``"bbox"``.  ``None`` if no
        mesh / geometry was usable (caller should keep a placeholder).
    """
    if not mesh_path:
        return None
    if mesh_path.lower().endswith(".glb"):
        # composed sub-assembly GLBs are written in metres already
        # (mesh.py applies the 0.001 and stamps units="meter")
        scale = 1.0
    props = _mesh_props(mesh_path, density, scale)
    if props is None:
        return None
    mass, com_m, I_m, method = props
    # mesh-frame -> link-frame: rotation rotates the tensor (about the com,
    # so the translation does not enter), translation moves the com
    R = _rpy_matrix(visual_rpy)[:3, :3]
    com = R @ com_m + np.asarray(visual_xyz, dtype=float)
    I = R @ I_m @ R.T
    return {
        "mass": mass,
        "com": [float(c) for c in com],
        "inertia": (float(I[0, 0]), float(I[0, 1]), float(I[0, 2]),
                    float(I[1, 1]), float(I[1, 2]), float(I[2, 2])),
        "method": method,
    }


def link_inertial_from_sw(mass, com_local, inertia6_local,
                          visual_xyz, visual_rpy):
    """Inertial in the LINK frame from SolidWorks-native mass properties.

    ``mass``/``com_local``/``inertia6_local`` come straight from the part's
    ``IMassProperty`` (see ``model._sw_mass_props``): SI units already (kg,
    metres, kg.m^2) and expressed in the part's OWN coordinate frame -- the
    very frame the mesh is exported in -- so the visual origin maps them into
    the link frame exactly as the mesh path does, with NO extra scaling.

    ``inertia6_local`` is ``(ixx, ixy, ixz, iyy, iyz, izz)`` of the tensor
    about the centre of mass.  Returns the same dict shape as
    :func:`link_inertial` (``method="solidworks"``), or ``None`` if the values
    are missing / non-finite so the caller can fall back to the mesh."""
    if mass is None or com_local is None or inertia6_local is None:
        return None
    try:
        m = float(mass)
        com_l = np.asarray(com_local, dtype=float)
        ixx, ixy, ixz, iyy, iyz, izz = (float(x) for x in inertia6_local)
    except (TypeError, ValueError):
        return None
    I_l = np.array([[ixx, ixy, ixz],
                    [ixy, iyy, iyz],
                    [ixz, iyz, izz]], dtype=float)
    if not (np.isfinite(m) and m > 0 and com_l.shape == (3,)
            and np.all(np.isfinite(com_l)) and np.all(np.isfinite(I_l))):
        return None
    # mesh-frame -> link-frame, identical transform to link_inertial: the
    # rotation rotates the tensor about the (unchanged) com, the translation
    # moves the com.
    R = _rpy_matrix(visual_rpy)[:3, :3]
    com = R @ com_l + np.asarray(visual_xyz, dtype=float)
    I = R @ I_l @ R.T
    return {
        "mass": m,
        "com": [float(c) for c in com],
        "inertia": (float(I[0, 0]), float(I[0, 1]), float(I[0, 2]),
                    float(I[1, 1]), float(I[1, 2]), float(I[2, 2])),
        "method": "solidworks",
    }


_PROPS_CACHE = {}


def _mesh_props(mesh_path, density, scale):
    """(mass, com, I_about_com, method) in SCALED MESH coordinates, cached by
    (path, mtime, density, scale).  Loading + watertight checks dominate a
    rebuild (~60 ms per mesh), and the result is pose-independent, so every
    re-build / instance reuse after the first is effectively free."""
    try:
        key = (os.path.abspath(mesh_path), os.path.getmtime(mesh_path),
               float(density), float(scale))
    except OSError:
        return None
    if key in _PROPS_CACHE:
        return _PROPS_CACHE[key]
    result = None
    try:
        import trimesh
        mesh = trimesh.load(mesh_path, force="mesh")
        if mesh is not None and hasattr(mesh, "vertices") \
                and len(mesh.vertices):
            mesh = mesh.copy()
            mesh.apply_scale(scale)
            method = "mesh"
            props = _solid_properties(mesh, density) \
                if mesh.is_watertight else None
            if props is None:
                method = "hull"
                try:
                    props = _solid_properties(mesh.convex_hull, density)
                except Exception:
                    props = None
            if props is None:
                method = "bbox"
                try:
                    props = _solid_properties(mesh.bounding_box_oriented,
                                              density)
                except Exception:
                    props = None
            if props is not None:
                mass, com, inertia = props
                result = (mass, com, inertia, method)
    except Exception:
        result = None
    _PROPS_CACHE[key] = result
    return result
