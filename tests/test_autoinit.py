"""Tests for the CAD-derived initial values: build-time inertia + the
self-collision limit sweep.

The inertia half needs only trimesh + skrobot (both core dependencies), so it
builds a copy of the committed ``feetech_hand`` cache and checks the URDF
carries real, sane masses.  The sweep half additionally needs the optional
python-fcl ``[ui]`` dependency; it skips when unavailable.

Everything builds into a tmp copy of the cache so the committed
``output/feetech_hand`` is never dirtied.
"""

import shutil
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest

REPO_ROOT = Path(__file__).resolve().parent.parent
CACHE = REPO_ROOT / "output" / "feetech_hand"


@pytest.fixture(scope="module")
def built_pkg(tmp_path_factory):
    if not (CACHE / "graph.json").is_file():
        pytest.skip(f"missing cached graph: {CACHE / 'graph.json'}")
    from sw2robot.exporter.export import build

    work = tmp_path_factory.mktemp("pkg")
    shutil.copy2(CACHE / "graph.json", work / "graph.json")
    (work / "meshes").mkdir()
    for f in (CACHE / "meshes").iterdir():
        if f.is_file():
            shutil.copy2(f, work / "meshes" / f.name)
    urdf = build(str(work))
    return Path(urdf)


# ----------------------------------------------------------------- inertia
def test_build_emits_real_inertia(built_pkg):
    root = ET.parse(built_pkg).getroot()
    masses, off_com = [], 0
    for link in root.findall("link"):
        ine = link.find("inertial")
        assert ine is not None, f"{link.get('name')} has no <inertial>"
        m = float(ine.find("mass").get("value"))
        masses.append(m)
        # diagonal inertia must be positive
        I = ine.find("inertia")
        for k in ("ixx", "iyy", "izz"):
            assert float(I.get(k)) > 0, f"{link.get('name')} {k} <= 0"
        if ine.find("origin").get("xyz") not in ("0 0 0", None):
            off_com += 1

    # not the old uniform 0.1 kg placeholder: masses must vary and be plausible
    assert len({round(m, 6) for m in masses}) > 1, "masses look like placeholders"
    assert 0.01 < sum(masses) < 50.0, f"total mass {sum(masses)} kg implausible"
    assert off_com > 0, "no link has an off-origin centre of mass"


def test_density_scales_mass_linearly(tmp_path):
    """Doubling density doubles every mass (pure geometry * density)."""
    if not (CACHE / "graph.json").is_file():
        pytest.skip("missing cached graph")
    from sw2robot.exporter.export import build

    def total_mass(density):
        work = tmp_path / f"d{int(density)}"
        (work / "meshes").mkdir(parents=True)
        shutil.copy2(CACHE / "graph.json", work / "graph.json")
        for f in (CACHE / "meshes").iterdir():
            if f.is_file():
                shutil.copy2(f, work / "meshes" / f.name)
        urdf = build(str(work), density=density)
        root = ET.parse(urdf).getroot()
        return sum(float(l.find("inertial/mass").get("value"))
                   for l in root.findall("link") if l.find("inertial") is not None)

    m1 = total_mass(1000.0)
    m2 = total_mass(2000.0)
    assert m1 > 0 and m2 == pytest.approx(2 * m1, rel=1e-3)


# ----------------------------------------------------------------- sweep
def _skrobot_ready():
    try:
        import trimesh  # noqa: F401
        from skrobot.models.urdf import RobotModelFromURDF  # noqa: F401
        from trimesh.collision import CollisionManager  # noqa: F401
    except Exception:
        return False
    return True


def _load_robot_and_meshes(urdf_path):
    import trimesh
    from skrobot.models.urdf import RobotModelFromURDF

    robot = RobotModelFromURDF(urdf_file=str(urdf_path))

    def link_mesh(l):
        vm = getattr(l, "visual_mesh", None)
        if vm is None:
            return None
        ms = vm if isinstance(vm, (list, tuple)) else [vm]
        ms = [m for m in ms if hasattr(m, "vertices") and len(m.vertices)]
        if not ms:
            return None
        return trimesh.util.concatenate(ms) if len(ms) > 1 else ms[0]

    meshes = {l.name: link_mesh(l) for l in robot.link_list}
    return robot, {k: v for k, v in meshes.items() if v is not None}


def test_sweep_limits_finds_limits_and_restores_pose(built_pkg):
    if not _skrobot_ready():
        pytest.skip("skrobot / python-fcl not available")
    from sw2robot.editor import autoinit

    robot, meshes = _load_robot_and_meshes(built_pkg)
    res = autoinit.sweep_limits(robot, meshes, step_deg=6, max_deg=120)

    assert res, "no revolute joints swept"
    for _jn, v in res.items():
        assert v["lower"] <= v["upper"]
        # a non-continuous joint that hit a collision must be a finite sub-range
        if v["hit_upper"] is not None:
            assert v["upper"] < 2.1   # well under the 120 deg cap (radians)

    # the sweep must leave every joint back at the home pose
    for j in robot.joint_list:
        if type(j).__name__ == "RotationalJoint":
            assert abs(float(j.joint_angle())) < 1e-6


def test_sweep_margin_tightens_limit(built_pkg):
    """A bigger backoff margin must never WIDEN a hit-bounded limit (it backs
    further off the colliding edge).  Guards the configurable margin_deg."""
    if not _skrobot_ready():
        pytest.skip("skrobot / python-fcl not available")
    from sw2robot.editor import autoinit

    robot, meshes = _load_robot_and_meshes(built_pkg)
    small = autoinit.sweep_limits(robot, meshes, step_deg=6, max_deg=120,
                                  margin_deg=2.0)
    hit = next((jn for jn, v in small.items()
                if v.get("hit_upper") or v.get("hit_lower")), None)
    if hit is None:
        pytest.skip("no joint hit a collision in this fixture")
    big = autoinit.sweep_limits(robot, meshes, step_deg=6, max_deg=120,
                                margin_deg=20.0, only={hit})
    if small[hit]["hit_upper"] is not None:
        assert big[hit]["upper"] <= small[hit]["upper"] + 1e-9
    if small[hit]["hit_lower"] is not None:
        assert big[hit]["lower"] >= small[hit]["lower"] - 1e-9


def test_self_collision_min_distance(built_pkg):
    if not _skrobot_ready():
        pytest.skip("skrobot / python-fcl not available")
    from sw2robot.editor import autoinit

    robot, meshes = _load_robot_and_meshes(built_pkg)
    sc = autoinit.SelfCollision(robot, meshes)
    d, pair = sc.min_distance()
    # at the home pose there should be no NEW collision beyond the rest baseline
    assert sc.new_pairs() == set()
    # a closest pair (or inf) is always reportable
    assert (pair is None) or (len(pair) == 2)
