"""Frame-only links: a CAD-only part (a 'dummy_axis' that exists to carry an
axis) stays in the kinematic tree but exports as a BARE coordinate frame -- no
visual, no collision, no inertial.  Covers the writer, the build_model config
key (incl. that it works on a movable joint, unlike mass_only), the
dropped-geometry check that must not flag it, and the editor's yaml plumbing."""
import xml.etree.ElementTree as ET

import numpy as np


def _eye():
    return list(np.eye(4).flatten())


# --------------------------------------------------------------- URDF writer

def _model_with_frame_only():
    from sw2robot.exporter.model import Component, Joint, RobotModel
    comps = [
        Component(name="Base-1", link_name="base_link", part_path=None,
                  is_subassembly=False, world=np.eye(4), fixed=True, dof=0,
                  mesh_file="meshes/base.3dxml", sw_mass=2.0, sw_com=[0, 0, 0],
                  sw_inertia=[1, 0, 0, 1, 0, 1]),
        # a dummy part: it HAS a mesh and a CAD mass, both fictional
        Component(name="dummy_axis-1", link_name="dummy_axis", part_path=None,
                  is_subassembly=False, world=np.eye(4), fixed=False, dof=1,
                  mesh_file="meshes/dummy_axis.3dxml", sw_mass=0.4,
                  sw_com=[0, 0, 0], sw_inertia=[0.1, 0, 0, 0.1, 0, 0.1],
                  frame_only=True),
    ]
    joints = [Joint(name="base_link__dummy_axis", parent="base_link",
                    child="dummy_axis", jtype="revolute", axis=[0, 0, 1],
                    lower=-1.0, upper=1.0)]
    return RobotModel(name="demo", components=comps, joints=joints,
                      base_link="base_link")


def test_frame_only_link_has_no_geometry_and_no_inertial(tmp_path):
    from sw2robot.exporter import urdf_writer
    out = tmp_path / "urdf" / "demo.urdf"
    urdf_writer.write_urdf(_model_with_frame_only(), str(out))

    root = ET.parse(out).getroot()
    links = {ln.get("name"): ln for ln in root.findall("link")}
    dummy = links["dummy_axis"]
    assert dummy.find("visual") is None
    assert dummy.find("collision") is None
    assert dummy.find("inertial") is None        # unlike a mass-only link
    # the link itself and its joint stay: it still carries the rotation axis
    joint = next(j for j in root.findall("joint")
                 if j.find("child").get("link") == "dummy_axis")
    assert joint.get("type") == "revolute"
    # a real link is untouched
    assert links["base_link"].find("visual") is not None
    assert links["base_link"].find("inertial") is not None


def test_frame_only_is_not_reported_as_an_inertia_source(capsys, tmp_path):
    """The per-link inertia summary counts sources; a link with no <inertial>
    must not show up as a 'None' bucket."""
    from sw2robot.exporter import urdf_writer
    urdf_writer.write_urdf(_model_with_frame_only(),
                           str(tmp_path / "urdf" / "demo.urdf"))
    assert "None" not in capsys.readouterr().out


# ---------------------------------------------------------------- build_model

def _graph(comp_states):
    from sw2robot.exporter.state import GraphState
    return GraphState(robot_name="r", source_assembly="x.SLDASM",
                      components=comp_states)


def _states():
    from sw2robot.exporter.state import ComponentState
    return [
        ComponentState(name="base", link_name="base", world=_eye(), fixed=True,
                       sw_mass=1.0, sw_com=[0, 0, 0],
                       sw_inertia=[1, 0, 0, 1, 0, 1]),
        ComponentState(name="dummy_axis", link_name="dummy_axis", world=_eye(),
                       sw_mass=0.4, sw_com=[0, 0, 0],
                       sw_inertia=[1, 0, 0, 1, 0, 1]),
    ]


def test_build_model_frame_only_works_on_a_movable_link():
    """mass_only is cleared on a movable child (an invisible heavy link is
    misleading); frame_only is exactly the case where that IS what you want."""
    from sw2robot.exporter.model import build_model
    config = {
        "base": "base",
        "joints": [{"parent": "base", "child": "dummy_axis",
                    "type": "revolute", "axis_dir": [0, 0, 1]}],
        "frame_only": ["dummy_axis"],
    }
    model = build_model(_graph(_states()), config=config)
    by = {c.link_name: c for c in model.components}
    assert by["dummy_axis"].frame_only is True
    assert by["base"].frame_only is False
    assert next(j for j in model.joints
                if j.child == "dummy_axis").jtype == "revolute"


def test_build_model_frame_only_clears_a_mass_override_on_that_link():
    from sw2robot.exporter.model import build_model
    config = {
        "base": "base",
        "joints": [{"parent": "base", "child": "dummy_axis", "type": "fixed"}],
        "masses": {"dummy_axis": 0.9},
        "mass_only": ["dummy_axis"],
        "frame_only": ["dummy_axis"],
    }
    model = build_model(_graph(_states()), config=config)
    c = next(c for c in model.components if c.link_name == "dummy_axis")
    assert c.frame_only is True
    assert c.mass_target is None and c.mass_only is False


def test_build_model_frame_only_unmatched_name_is_ignored(capsys):
    from sw2robot.exporter.model import build_model
    model = build_model(_graph(_states()),
                        config={"base": "base", "frame_only": ["nope"]})
    assert all(not c.frame_only for c in model.components)
    assert "frame_only: 'nope' matched no link" in capsys.readouterr().out


# --------------------------------------------------- dropped-geometry check

_URDF = """<?xml version="1.0"?>
<robot name="demo">
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <geometry><mesh filename="../meshes/base.glb"/></geometry>
    </visual>
  </link>
  <link name="dummy_axis"/>
  <joint name="j" type="revolute">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base_link"/><child link="dummy_axis"/>
    <axis xyz="0 0 1"/><limit lower="-1" upper="1" effort="1" velocity="1"/>
  </joint>
</robot>
"""


def _dropped_geometry_case(tmp_path, monkeypatch, skip):
    """Run the check on a URDF where 'dummy_axis' has NO geometry while the
    graph says it has a mesh, a metre away from anything in the scene."""
    from sw2robot.exporter import validate
    pkg = tmp_path / "pkg"
    (pkg / "urdf").mkdir(parents=True)
    (pkg / "meshes").mkdir()
    urdf = pkg / "urdf" / "demo.urdf"
    urdf.write_text(_URDF, encoding="utf-8")
    # stand in for the .glb reader: a small cube of points around the origin
    cube = np.array([[x, y, z] for x in (0, 0.01) for y in (0, 0.01)
                     for z in (0, 0.01)], float)
    monkeypatch.setattr(validate, "_load_glb_verts", lambda p: cube)
    far = np.eye(4)
    far[:3, 3] = [1.0, 0, 0]           # nowhere near the base link's mesh
    graph = {
        "components": [
            {"name": "base", "mesh_file": "meshes/base.glb",
             "is_subassembly": False},
            {"name": "dummy_axis", "mesh_file": "meshes/dummy.glb",
             "is_subassembly": False}],
        "deep_worlds": {"base": _eye(), "dummy_axis": list(far.flatten())},
    }
    return validate.warn_dropped_geometry(str(pkg), str(urdf), graph,
                                          skip_components=skip)


def test_dropped_geometry_check_flags_a_part_that_is_missing(tmp_path,
                                                             monkeypatch):
    # the control: without the skip set, this IS reported as dropped geometry
    assert _dropped_geometry_case(tmp_path, monkeypatch, None) == ["dummy_axis"]


def test_dropped_geometry_check_skips_geometry_free_links(tmp_path,
                                                          monkeypatch):
    """A frame-only part has no geometry in the URDF BY DESIGN -- it must not be
    reported as a part that fell out of the export."""
    assert _dropped_geometry_case(tmp_path, monkeypatch,
                                  {"dummy_axis"}) == []


# ------------------------------------------------------------ editor wiring

def test_set_frame_only_members_adds_and_removes():
    from sw2robot.editor.webserver import _set_frame_only_members
    txt = ("base: x\njoints:\n  - parent: a\n    child: dummy_axis\n"
           "    type: revolute\n")
    added = _set_frame_only_members(txt, {"dummy_axis"}, set())
    assert "frame_only:\n- dummy_axis\n" in added
    cleared = _set_frame_only_members(added, set(), {"dummy_axis"})
    assert "frame_only:" not in cleared and "joints:" in cleared
    # a no-op edit returns the text unchanged
    assert _set_frame_only_members(txt, set(), set()) == txt


def test_read_frame_only_reads_the_config_block(tmp_path):
    from sw2robot.editor.webserver import _read_frame_only
    (tmp_path / "r.joints.yaml").write_text(
        "base: x\nframe_only:\n- dummy_axis\n- probe\n", encoding="utf-8")
    assert _read_frame_only(str(tmp_path), "urdf/r.urdf") == {"dummy_axis",
                                                              "probe"}
    # no config -> empty, never an exception
    assert _read_frame_only(str(tmp_path), "urdf/other.urdf") == set()
    assert _read_frame_only(None, "urdf/r.urdf") == set()
