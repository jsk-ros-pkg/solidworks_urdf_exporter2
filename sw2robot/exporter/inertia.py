"""sw2robot's mesh-unit convention for link inertials.

The inertial maths itself lives in :mod:`skrobot.utils.inertia`
(``mesh_mass_properties``, ``transform_inertial``, ``link_inertial_from_mesh``,
``rescale_inertial_to_mass``, ``validate_inertia``) -- call those directly.
What is sw2robot-specific, and all that remains here, is the SCALE of the
meshes we hand it, which ``graph.json`` fixes by file format
(see :mod:`sw2robot.exporter.state`):

* ``.3dxml`` -- what SolidWorks exports per part -- is **millimetres** (a servo
  body is ~32 mm) while the URDF skeleton is in metres, so it needs ``MM_TO_M``
  before density is applied; otherwise mass comes out 1e9 times too large.
* Every other format is **metres**: the composed sub-assembly ``.glb`` files
  ``mesh.py`` writes (it applies the 0.001 itself), and the ``.stl``/``.obj``
  an extractor for another CAD writes.  The ROS/MJCF export and the editor read
  meshes the same way.
"""

from __future__ import annotations

from skrobot.utils.inertia import DEFAULT_DENSITY, link_inertial_from_mesh

MM_TO_M = 0.001

# the only mesh format graph.json stores in millimetres
MILLIMETRE_MESH_SUFFIXES = (".3dxml",)


def mesh_scale_to_metres(mesh_path):
    """Factor that turns a working mesh's coordinates into metres.

    Parameters
    ----------
    mesh_path : str
        Path of a mesh referenced from ``graph.json``.

    Returns
    -------
    float
        ``MM_TO_M`` for a ``.3dxml`` mesh, 1.0 for every other format.
    """
    if mesh_path.lower().endswith(MILLIMETRE_MESH_SUFFIXES):
        return MM_TO_M
    return 1.0


def link_inertial(mesh_path, visual_xyz, visual_rpy,
                  density=DEFAULT_DENSITY, scale=None):
    """Inertial for one link, in the link frame, from a sw2robot-exported mesh.

    A thin wrapper over :func:`skrobot.utils.inertia.link_inertial_from_mesh`
    that applies sw2robot's mesh-unit convention (see the module docstring).

    Parameters
    ----------
    mesh_path : str | None
        Absolute path to the link's mesh (any format trimesh can load).
    visual_xyz, visual_rpy : sequence of 3 floats
        The link's visual origin (mesh -> link), in metres / radians.
    density : float
        Material density in kg/m^3.
    scale : float | None
        Mesh-unit -> metre factor.  None derives it from the file format with
        :func:`mesh_scale_to_metres`.

    Returns
    -------
    dict | None
        ``{mass, com(3), inertia(ixx,ixy,ixz,iyy,iyz,izz), method}`` where
        ``method`` is one of ``"mesh"``, ``"hull"``, ``"bbox"``.  ``None`` if no
        mesh / geometry was usable (caller should keep a placeholder).
    """
    if not mesh_path:
        return None
    if scale is None:
        scale = mesh_scale_to_metres(mesh_path)
    return link_inertial_from_mesh(mesh_path, visual_xyz, visual_rpy,
                                   density=density, scale=scale)
