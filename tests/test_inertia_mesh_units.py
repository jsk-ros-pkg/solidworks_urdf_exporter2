"""sw2robot's mesh-unit convention for inertials (exporter.inertia).

The inertial maths is scikit-robot's; the ONE thing sw2robot owns is the scale
it hands the library: SolidWorks per-part meshes are millimetres, while the
composed sub-assembly ``.glb`` files ``mesh.py`` writes are already metres and
must NOT be scaled again.  Getting this wrong is a 1e9 error in mass, so it is
pinned here.
"""
import numpy as np
import pytest

from sw2robot.exporter.inertia import MM_TO_M, link_inertial

DENSITY = 1200.0
EXTENTS = (20.0, 30.0, 40.0)        # in MESH units
ORIGIN = ([0, 0, 0], [0, 0, 0])     # identity visual origin


def _box(tmp_path, suffix):
    trimesh = pytest.importorskip("trimesh")
    path = tmp_path / f"box{suffix}"
    trimesh.creation.box(extents=list(EXTENTS)).export(path)
    return str(path)


def test_part_mesh_is_treated_as_millimetres(tmp_path):
    """A 20x30x40 *mm* box at 1200 kg/m^3 weighs 2.4e-5 m^3 * 1200 = 28.8 g."""
    info = link_inertial(_box(tmp_path, ".stl"), *ORIGIN, density=DENSITY)
    volume_m3 = np.prod([e * MM_TO_M for e in EXTENTS])
    assert info["mass"] == pytest.approx(volume_m3 * DENSITY, rel=1e-9)
    assert info["method"] == "mesh"          # a box is watertight


def test_glb_subassembly_mesh_is_already_metres(tmp_path):
    """The same box as .glb is metres, so it comes out 1e9 times heavier --
    mesh.py already applied the 0.001 when it composed the sub-assembly."""
    glb = link_inertial(_box(tmp_path, ".glb"), *ORIGIN, density=DENSITY)
    stl = link_inertial(_box(tmp_path, ".stl"), *ORIGIN, density=DENSITY)
    assert glb["mass"] == pytest.approx(np.prod(EXTENTS) * DENSITY, rel=1e-9)
    assert glb["mass"] / stl["mass"] == pytest.approx(1 / MM_TO_M ** 3, rel=1e-6)


def test_glb_detection_ignores_case(tmp_path):
    """Extensions are matched case-insensitively (paths come from Windows)."""
    lower = link_inertial(_box(tmp_path, ".glb"), *ORIGIN, density=DENSITY)
    upper = link_inertial(_box(tmp_path, ".GLB"), *ORIGIN, density=DENSITY)
    assert upper["mass"] == pytest.approx(lower["mass"], rel=1e-12)


def test_explicit_scale_is_honoured_for_non_glb(tmp_path):
    """A caller-supplied scale overrides the millimetre default."""
    info = link_inertial(_box(tmp_path, ".stl"), *ORIGIN,
                         density=DENSITY, scale=1.0)
    assert info["mass"] == pytest.approx(np.prod(EXTENTS) * DENSITY, rel=1e-9)


@pytest.mark.parametrize("path", [None, "", "no_such_mesh.stl"])
def test_missing_mesh_returns_none(path):
    """No usable mesh -> None, so the writer keeps its placeholder inertial."""
    assert link_inertial(path, *ORIGIN) is None
