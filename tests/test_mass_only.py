"""Mass-only links in the exporter (issue #16 backlog): keep a part's weight but
drop its visual/collision geometry.  Covers the URDF writer (geometry stripped,
inertial kept, final names reported), the build_model config + only-fixed guard,
and the sidecar the detached ROS export reads."""
import xml.etree.ElementTree as ET

import numpy as np
import yaml


def _eye():
    return list(np.eye(4).flatten())


# --------------------------------------------------------------- URDF writer

def _model_with_mass_only():
    from sw2robot.exporter.model import Component, Joint, RobotModel
    comps = [
        Component(name="Base-1", link_name="base_link", part_path=None,
                  is_subassembly=False, world=np.eye(4), fixed=True, dof=0,
                  mesh_file="meshes/base.3dxml", sw_mass=2.0, sw_com=[0, 0, 0],
                  sw_inertia=[1, 0, 0, 1, 0, 1]),
        # a mass-only internal part: it HAS a mesh, but mass_only must drop it
        Component(name="Pcb-1", link_name="pcb", part_path=None,
                  is_subassembly=False, world=np.eye(4), fixed=True, dof=0,
                  mesh_file="meshes/pcb.3dxml", sw_mass=0.3, sw_com=[0, 0, 0],
                  sw_inertia=[0.1, 0, 0, 0.1, 0, 0.1], mass_only=True),
    ]
    joints = [Joint(name="base_link__pcb", parent="base_link", child="pcb",
                    jtype="fixed")]
    return RobotModel(name="demo", components=comps, joints=joints,
                      base_link="base_link")


def test_write_urdf_strips_geometry_keeps_inertial_and_reports_names(tmp_path):
    from sw2robot.exporter import urdf_writer
    out = tmp_path / "urdf" / "demo.urdf"
    mass_only = urdf_writer.write_urdf(_model_with_mass_only(), str(out))

    # the writer reports the final link name so the export step can fold it
    assert mass_only == {"pcb"}

    root = ET.parse(out).getroot()
    links = {ln.get("name"): ln for ln in root.findall("link")}
    pcb = links["pcb"]
    # geometry dropped despite the part having a mesh; weight (inertial) kept
    assert pcb.find("visual") is None and pcb.find("collision") is None
    assert pcb.find("inertial") is not None
    assert abs(float(pcb.find("inertial").find("mass").get("value")) - 0.3) < 1e-9
    # the ordinary link is untouched -- still has its geometry
    assert links["base_link"].find("visual") is not None


# --------------------------------------------------------------- build_model

def _graph(comp_states, robot="r"):
    from sw2robot.exporter.state import GraphState
    return GraphState(robot_name=robot, source_assembly="x.SLDASM",
                      components=comp_states)


def test_build_model_sets_mass_only_on_fixed_child_only():
    """`mass_only:` config flags the matching part; the only-fixed guard keeps it
    on a fixed child and drops it (with a warning) on a movable one / the root."""
    from sw2robot.exporter.state import ComponentState
    from sw2robot.exporter.model import build_model
    comps = [
        ComponentState(name="base", link_name="base", world=_eye(), fixed=True,
                       sw_mass=1.0, sw_com=[0, 0, 0], sw_inertia=[1, 0, 0, 1, 0, 1]),
        ComponentState(name="pcb", link_name="pcb", world=_eye(),
                       sw_mass=0.2, sw_com=[0, 0, 0],
                       sw_inertia=[1, 0, 0, 1, 0, 1]),
        ComponentState(name="arm", link_name="arm", world=_eye(),
                       sw_mass=0.5, sw_com=[0, 0, 0],
                       sw_inertia=[1, 0, 0, 1, 0, 1]),
    ]
    # config wires the tree explicitly so joint types are deterministic:
    # pcb is fixed to base (mass-only OK), arm is revolute (mass-only invalid)
    config = {
        "base": "base",
        "joints": [
            {"parent": "base", "child": "pcb", "type": "fixed"},
            {"parent": "base", "child": "arm", "type": "revolute",
             "axis_dir": [0, 0, 1]},
        ],
        "mass_only": ["pcb", "arm"],
    }
    model = build_model(_graph(comps), config=config)
    by = {c.link_name: c for c in model.components}
    assert by["pcb"].mass_only is True          # fixed child -> kept
    assert by["arm"].mass_only is False         # revolute -> guard cleared it


def test_build_model_mass_only_unmatched_name_is_ignored():
    from sw2robot.exporter.state import ComponentState
    from sw2robot.exporter.model import build_model
    comps = [ComponentState(name="base", link_name="base", world=_eye(),
                            fixed=True, sw_mass=1.0, sw_com=[0, 0, 0],
                            sw_inertia=[1, 0, 0, 1, 0, 1])]
    model = build_model(_graph(comps), config={"mass_only": ["nope"]})
    assert all(not c.mass_only for c in model.components)


# --------------------------------------------------------------- sidecar

def test_read_mass_only_sidecar_roundtrip(tmp_path):
    from sw2robot.exporter.ros_export import _read_mass_only
    assert _read_mass_only(str(tmp_path)) == set()      # no sidecar -> empty
    (tmp_path / "mass_only.yaml").write_text(yaml.safe_dump(["pcb", "wiring"]))
    assert _read_mass_only(str(tmp_path)) == {"pcb", "wiring"}


def test_build_writes_mass_only_sidecar_for_export(tmp_path):
    """End-to-end through build(): the working URDF keeps the stripped link, and
    a mass_only.yaml sidecar lists its final name so the detached ROS export can
    fold it.  A build with no mass-only links leaves no stale sidecar behind."""
    from sw2robot.exporter.state import ComponentState, GraphState
    from sw2robot.exporter.export import build
    from sw2robot.exporter.ros_export import _read_mass_only

    pkg = tmp_path / "pkg"
    pkg.mkdir()
    graph = GraphState(
        robot_name="demo", source_assembly="x.SLDASM",
        components=[
            ComponentState(name="base", link_name="base", world=_eye(),
                           fixed=True, sw_mass=1.0, sw_com=[0, 0, 0],
                           sw_inertia=[1, 0, 0, 1, 0, 1]),
            ComponentState(name="pcb", link_name="pcb", world=_eye(),
                           sw_mass=0.2, sw_com=[0, 0, 0],
                           sw_inertia=[1, 0, 0, 1, 0, 1]),
        ])
    graph.save(str(pkg / "graph.json"))

    cfg = pkg / "demo.joints.yaml"
    cfg.write_text(yaml.safe_dump({
        "base": "base",
        "joints": [{"parent": "base", "child": "pcb", "type": "fixed"}],
        "mass_only": ["pcb"],
    }))
    build(str(pkg), config_path=str(cfg))

    side = pkg / "mass_only.yaml"
    assert side.exists()
    assert _read_mass_only(str(pkg)) == {"pcb"}

    # the working URDF keeps the link (selectable) but with no geometry
    root = ET.parse(pkg / "urdf" / "demo.urdf").getroot()
    pcb = next(ln for ln in root.findall("link") if ln.get("name") == "pcb")
    assert pcb.find("visual") is None and pcb.find("inertial") is not None

    # a rebuild with the flag removed clears the sidecar (no stale state)
    cfg.write_text(yaml.safe_dump({
        "base": "base",
        "joints": [{"parent": "base", "child": "pcb", "type": "fixed"}],
    }))
    build(str(pkg), config_path=str(cfg))
    assert not side.exists()
