"""sw2robot's mesh-unit convention for inertials (exporter.inertia).

The inertial maths is scikit-robot's; the ONE thing sw2robot owns is the scale
it hands the library.  graph.json fixes it by format: SolidWorks' per-part
``.3dxml`` is millimetres, every other format (the composed ``.glb``, an STL
from another CAD's extractor) is metres -- the same rule the ROS/MJCF export
uses to draw the mesh.  Getting this wrong is a 1e9 error in mass, so it is
pinned here.
"""
import numpy as np
import pytest

from sw2robot.exporter.inertia import (
    MM_TO_M,
    link_inertial,
    mesh_scale_to_metres,
)

DENSITY = 1200.0
EXTENTS = (20.0, 30.0, 40.0)        # in MESH units
ORIGIN = ([0, 0, 0], [0, 0, 0])     # identity visual origin


def _box(tmp_path, suffix):
    trimesh = pytest.importorskip("trimesh")
    path = tmp_path / f"box{suffix}"
    trimesh.creation.box(extents=list(EXTENTS)).export(path)
    return str(path)


@pytest.mark.parametrize("path, scale", [
    ("meshes/part.3dxml", MM_TO_M),
    ("C:\\cad\\PART.3DXML", MM_TO_M),       # paths come from Windows
    ("meshes/sub.glb", 1.0),
    ("meshes/part.stl", 1.0),
    ("meshes/part.obj", 1.0),
])
def test_mesh_unit_follows_the_format(path, scale):
    assert mesh_scale_to_metres(path) == scale


def test_millimetre_scale_gives_millimetre_mass(tmp_path):
    """A 20x30x40 *mm* box at 1200 kg/m^3 weighs 2.4e-5 m^3 * 1200 = 28.8 g.

    trimesh cannot write .3dxml, so the SolidWorks case passes its scale."""
    info = link_inertial(_box(tmp_path, ".stl"), *ORIGIN, density=DENSITY,
                         scale=MM_TO_M)
    volume_m3 = np.prod([e * MM_TO_M for e in EXTENTS])
    assert info["mass"] == pytest.approx(volume_m3 * DENSITY, rel=1e-9)
    assert info["method"] == "mesh"          # a box is watertight


@pytest.mark.parametrize("suffix", [".glb", ".GLB", ".stl"])
def test_non_3dxml_mesh_is_already_metres(tmp_path, suffix):
    """The composed .glb and another CAD's .stl are metres: not scaled again."""
    info = link_inertial(_box(tmp_path, suffix), *ORIGIN, density=DENSITY)
    assert info["mass"] == pytest.approx(np.prod(EXTENTS) * DENSITY, rel=1e-9)


def test_explicit_scale_wins(tmp_path):
    """A caller-supplied scale overrides the format rule."""
    glb = link_inertial(_box(tmp_path, ".glb"), *ORIGIN,
                        density=DENSITY, scale=MM_TO_M)
    volume_m3 = np.prod([e * MM_TO_M for e in EXTENTS])
    assert glb["mass"] == pytest.approx(volume_m3 * DENSITY, rel=1e-9)


@pytest.mark.parametrize("path", [None, "", "no_such_mesh.stl"])
def test_missing_mesh_returns_none(path):
    """No usable mesh -> None, so the writer keeps its placeholder inertial."""
    assert link_inertial(path, *ORIGIN) is None
