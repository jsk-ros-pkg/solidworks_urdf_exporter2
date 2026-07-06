"""Joint physics (imported from the classic SW2URDF exporter): optional URDF
``<dynamics>`` (damping/friction), ``<safety_controller>`` (soft limits +
k_position/k_velocity), ``<calibration>`` (rising/falling), plus editable
``<limit effort/velocity>``.  Three surfaces must agree:

- exporter build path:  joints.yaml -> resolve_directed -> Joint -> urdf_writer
- editor overlay path:  JointEdit -> core.build_urdf
- CAD-mode web edit:    /api/set_physics -> joints.yaml patch + served-URDF patch

These are pure unit tests (no SolidWorks, no server); the CAD-mode integration
lives in test_cad_mode_webserver-style suites and is exercised end-to-end there.
"""

import types
import xml.etree.ElementTree as ET

import yaml


# ----------------------------------------------------- exporter: urdf_writer
def _rev_joint(**kw):
    from sw2robot.exporter.model import Joint
    base = {"name": "a__b", "parent": "a", "child": "b", "jtype": "revolute",
            "axis": [0, 0, 1], "lower": -1.0, "upper": 1.0}
    base.update(kw)
    return Joint(**base)


def test_urdf_writer_emits_all_physics():
    from sw2robot.exporter import urdf_writer as uw
    j = _rev_joint(effort=5.0, velocity=2.0,
                   dynamics={"damping": 0.1, "friction": 0.05},
                   safety={"soft_lower_limit": -0.9, "soft_upper_limit": 0.9,
                           "k_position": 100, "k_velocity": 10},
                   calibration={"rising": 0.0})
    frag = ET.fromstring("<robot>" + uw._joint_xml(j) + "</robot>")[0]
    lim = frag.find("limit")
    assert lim.get("effort") == "5" and lim.get("velocity") == "2"
    dyn = frag.find("dynamics")
    assert dyn.get("damping") == "0.1" and dyn.get("friction") == "0.05"
    saf = frag.find("safety_controller")
    assert saf.get("soft_lower_limit") == "-0.9"
    assert saf.get("k_position") == "100" and saf.get("k_velocity") == "10"
    assert frag.find("calibration").get("rising") == "0"


def test_urdf_writer_defaults_when_unset():
    from sw2robot.exporter import urdf_writer as uw
    frag = ET.fromstring("<robot>" + uw._joint_xml(_rev_joint()) + "</robot>")[0]
    lim = frag.find("limit")
    assert lim.get("effort") == "10" and lim.get("velocity") == "3.14"
    assert frag.find("dynamics") is None
    assert frag.find("safety_controller") is None
    assert frag.find("calibration") is None


def test_urdf_writer_continuous_effort_only():
    from sw2robot.exporter import urdf_writer as uw
    j = _rev_joint(jtype="continuous", lower=None, upper=None, effort=7.0)
    frag = ET.fromstring("<robot>" + uw._joint_xml(j) + "</robot>")[0]
    lim = frag.find("limit")
    assert lim is not None and lim.get("effort") == "7"
    assert lim.get("lower") is None            # continuous carries no endpoints


# ------------------------------------------------- exporter: config parsing
def test_physics_from_cfg_reads_and_drops_none():
    from sw2robot.exporter.model import physics_from_cfg
    got = physics_from_cfg({
        "effort": 5, "velocity": 2,
        "dynamics": {"damping": 0.1},
        "safety_controller": {"k_position": 100},   # URDF spelling
        "calibration": {"rising": 0.0, "falling": None},
    })
    assert got["effort"] == 5.0 and got["velocity"] == 2.0
    assert got["dynamics"] == {"damping": 0.1}
    assert got["safety"] == {"k_position": 100.0}
    assert got["calibration"] == {"rising": 0.0}   # falling=None dropped


def test_physics_from_cfg_safety_alias_and_empty():
    from sw2robot.exporter.model import physics_from_cfg
    assert physics_from_cfg({"safety": {"k_velocity": 3}})["safety"] == \
        {"k_velocity": 3.0}
    empty = physics_from_cfg({"type": "revolute"})
    assert all(empty[k] is None for k in empty)


# ------------------------------------------------- exporter: joints.yaml write
def test_write_template_roundtrips_physics(tmp_path):
    from sw2robot.exporter import jointcfg
    from sw2robot.exporter.model import physics_from_cfg
    j = _rev_joint(effort=5.0, velocity=2.0,
                   dynamics={"damping": 0.1, "friction": 0.0},
                   safety={"soft_lower_limit": -0.9, "k_position": 100},
                   calibration={"rising": 0.0, "falling": 0.0},
                   mate_types=[], geo_note=None)
    model = types.SimpleNamespace(base_link="a", joints=[j], detected_edges=[])
    p = tmp_path / "r.joints.yaml"
    jointcfg.write_template(model, str(p))
    entry = yaml.safe_load(p.read_text())["joints"][0]
    back = physics_from_cfg(entry)
    assert back["effort"] == 5.0 and back["velocity"] == 2.0
    assert back["dynamics"] == {"damping": 0.1, "friction": 0.0}
    assert back["safety"] == {"soft_lower_limit": -0.9, "k_position": 100.0}
    assert back["calibration"] == {"rising": 0.0, "falling": 0.0}


# ------------------------------------------------------ editor overlay path
def _state(tmp_path, urdf):
    from sw2robot.editor.state import RobotCompilerState
    p = tmp_path / "t.urdf"
    p.write_text(urdf, encoding="utf-8")
    return RobotCompilerState(
        robot_name="t", urdf_path=str(p), package_dir=str(tmp_path),
        joints=[{"name": "j1", "type": "revolute"}],
        links=[{"name": "base"}, {"name": "arm"}], root_link="base")


_URDF = """<?xml version="1.0"?>
<robot name="t">
  <link name="base"/>
  <link name="arm"/>
  <joint name="j1" type="revolute">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base"/><child link="arm"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="10" velocity="3.14"/>
  </joint>
</robot>"""


def test_overlay_applies_and_clears_physics(tmp_path):
    from sw2robot.editor import core
    st = _state(tmp_path, _URDF)
    core.set_actuator(st, "j1", effort=5.0, velocity=2.0)
    core.set_joint_physics(st, "j1", damping=0.1, friction=0.02,
                           soft_lower_limit=-0.9, soft_upper_limit=0.9,
                           k_position=100, k_velocity=10, cal_rising=0.0)
    j = ET.fromstring(core.build_urdf(st, sanitize=False)).find("joint")
    assert j.find("limit").get("effort") == "5"
    assert j.find("dynamics").get("damping") == "0.1"
    assert j.find("safety_controller").get("k_position") == "100"
    assert j.find("calibration").get("rising") == "0"

    # clearing just dynamics removes <dynamics>, keeps the rest
    core.set_joint_physics(st, "j1", damping=None, friction=None)
    j2 = ET.fromstring(core.build_urdf(st, sanitize=False)).find("joint")
    assert j2.find("dynamics") is None
    assert j2.find("safety_controller") is not None
    assert j2.find("calibration") is not None


def test_overlay_physics_skipped_on_fixed(tmp_path):
    from sw2robot.editor import core
    st = _state(tmp_path, _URDF)
    core.set_joint_physics(st, "j1", damping=0.1)
    core.set_joint_type(st, "j1", "fixed")
    j = ET.fromstring(core.build_urdf(st, sanitize=False)).find("joint")
    assert j.find("dynamics") is None            # not emitted on a fixed joint


# ---------------------------------------------------- CAD-mode web patchers
_YAML = """base: base_link
joints:
  - parent: base
    child:  arm
    type:   revolute # mates: concentric
    lower: -1.00000
    upper: 1.00000
  - parent: arm
    child:  tip
    type:   fixed

# --- reference (do not touch) ---
#   base <-> arm
"""

_FULL = {"child": "arm", "effort": 5.0, "velocity": 2.0, "damping": 0.1,
         "friction": None, "soft_lower_limit": -0.9, "soft_upper_limit": 0.9,
         "k_position": 100, "k_velocity": 10, "cal_rising": 0.0,
         "cal_falling": None}


def test_yaml_patch_insert_idempotent_clear():
    from sw2robot.editor import webserver as ws
    from sw2robot.exporter.model import physics_from_cfg

    out, n = ws._set_joint_physics_yaml(_YAML, "arm", _FULL)
    assert n == 1
    entry = next(j for j in yaml.safe_load(out)["joints"]
                 if j["child"] == "arm")
    back = physics_from_cfg(entry)
    assert back["effort"] == 5.0 and back["velocity"] == 2.0
    assert back["dynamics"] == {"damping": 0.1}          # friction=None dropped
    assert back["safety"]["k_position"] == 100.0
    assert back["calibration"] == {"rising": 0.0}
    # the reference comment block is preserved untouched
    assert "# --- reference (do not touch) ---" in out
    # replay is stable (idempotent)
    out2, _ = ws._set_joint_physics_yaml(out, "arm", _FULL)
    assert out2 == out
    # clearing drops every physics line again
    cleared, _ = ws._set_joint_physics_yaml(out, "arm", {"child": "arm"})
    assert "dynamics:" not in cleared and "effort:" not in cleared
    assert "calibration:" not in cleared


def test_urdf_patch_rebuilds_elements(tmp_path):
    from sw2robot.editor import webserver as ws
    (tmp_path / "urdf").mkdir()
    rel = "urdf/t.urdf"
    (tmp_path / rel).write_text(_URDF, encoding="utf-8")
    assert ws._set_physics_in_urdf(str(tmp_path), rel, "arm", _FULL)
    j = ET.fromstring((tmp_path / rel).read_text()).find("joint")
    assert j.find("limit").get("effort") == "5"
    assert j.find("dynamics").get("damping") == "0.1"
    assert j.find("dynamics").get("friction") is None    # cleared field absent
    assert j.find("safety_controller").get("soft_upper_limit") == "0.9"
    assert j.find("calibration").get("rising") == "0"
    # re-applying with all cleared removes the optional elements
    ws._set_physics_in_urdf(str(tmp_path), rel, "arm", {"child": "arm"})
    j2 = ET.fromstring((tmp_path / rel).read_text()).find("joint")
    assert j2.find("dynamics") is None and j2.find("calibration") is None
