"""sw2robot's mesh-unit convention for link inertials.

The inertial maths itself lives in :mod:`skrobot.utils.inertia`
(``mesh_mass_properties``, ``transform_inertial``, ``link_inertial_from_mesh``,
``rescale_inertial_to_mass``, ``validate_inertia``) -- call those directly.
What is sw2robot-specific, and all that remains here, is the SCALE of the
meshes we hand it:

* SolidWorks exports parts in **millimetres** (a servo body is ~32 mm) while
  the URDF skeleton is in **metres**, so a per-part mesh needs ``MM_TO_M``
  before density is applied -- otherwise mass comes out 1e9 times too large.
* Composed sub-assembly ``.glb`` files are the exception: ``mesh.py`` already
  applies the 0.001 and stamps ``units="meter"`` when it writes them, so they
  must NOT be scaled again.
"""

from __future__ import annotations

from skrobot.utils.inertia import DEFAULT_DENSITY, link_inertial_from_mesh

MM_TO_M = 0.001


def link_inertial(mesh_path, visual_xyz, visual_rpy,
                  density=DEFAULT_DENSITY, scale=MM_TO_M):
    """Inertial for one link, in the link frame, from a sw2robot-exported mesh.

    A thin wrapper over :func:`skrobot.utils.inertia.link_inertial_from_mesh`
    that applies sw2robot's mesh-unit convention: millimetre parts by default,
    but ``.glb`` sub-assembly composites taken as metres (see the module
    docstring).

    Parameters
    ----------
    mesh_path : str | None
        Absolute path to the link's mesh (any format trimesh can load).
    visual_xyz, visual_rpy : sequence of 3 floats
        The link's visual origin (mesh -> link), in metres / radians.
    density : float
        Material density in kg/m^3.
    scale : float
        Mesh-unit -> metre factor, overridden to 1.0 for ``.glb`` inputs.

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
        scale = 1.0
    return link_inertial_from_mesh(mesh_path, visual_xyz, visual_rpy,
                                   density=density, scale=scale)
