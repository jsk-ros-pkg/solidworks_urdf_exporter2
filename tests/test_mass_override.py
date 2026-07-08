"""Per-link target-mass override (issue #29): a `masses:` block (mirroring
`densities:`) rescales a link's inertial to an exact weight.  Covers the
inertia.rescale_to_mass helper, the urdf_writer consumption, the build_model
config parsing + mutual-exclusivity with a density override, and an end-to-end
build() through joints.yaml."""
import xml.etree.ElementTree as ET

import numpy as np
import yaml


def _eye():
    return list(np.eye(4).flatten())


# ---------------------------------------------------------------- rescale helper

def test_rescale_to_mass_scales_mass_and_tensor_keeps_com():
    from sw2robot.exporter.inertia import rescale_to_mass
    info = {"mass": 2.0, "com": [0.1, -0.2, 0.3],
            "inertia": (3.0, 0.5, 0.0, 4.0, 0.0, 5.0), "method": "solidworks"}
    out = rescale_to_mass(info, 6.0)          # factor 3
    assert out["mass"] == 6.0
    assert np.allclose(out["com"], [0.1, -0.2, 0.3])          # com unchanged
    assert np.allclose(out["inertia"], (9.0, 1.5, 0.0, 12.0, 0.0, 15.0))
    assert out["method"] == "solidworks->mass"
    # original dict is not mutated
    assert info["mass"] == 2.0 and info["method"] == "solidworks"


def test_rescale_to_mass_noop_on_bad_input():
    from sw2robot.exporter.inertia import rescale_to_mass
    assert rescale_to_mass(None, 5.0) is None
    bad = {"mass": 0.0, "com": [0, 0, 0], "inertia": (1, 0, 0, 1, 0, 1),
           "method": "x"}
    assert rescale_to_mass(bad, 5.0) is bad          # zero base mass -> unchanged
    good = {"mass": 1.0, "com": [0, 0, 0], "inertia": (1, 0, 0, 1, 0, 1),
            "method": "x"}
    assert rescale_to_mass(good, 0.0) is good        # non-positive target -> unchanged


# ---------------------------------------------------------------- urdf writer

def _model_with_sw(mass_target=None):
    from sw2robot.exporter.model import Component, RobotModel
    c = Component(name="Part-1", link_name="base_link", part_path=None,
                  is_subassembly=False, world=np.eye(4), fixed=True, dof=0,
                  sw_mass=2.0, sw_com=[0.0, 0.0, 0.1],
                  sw_inertia=[0.2, 0.0, 0.0, 0.3, 0.0, 0.4],
                  mass_target=mass_target)
    return RobotModel(name="demo", components=[c], joints=[],
                      base_link="base_link")


def test_writer_rescales_solidworks_inertial_to_target_mass(tmp_path):
    from sw2robot.exporter import urdf_writer
    out = tmp_path / "urdf" / "demo.urdf"
    urdf_writer.write_urdf(_model_with_sw(mass_target=6.0), str(out))  # factor 3
    inertial = ET.parse(out).getroot().find("link/inertial")
    assert np.isclose(float(inertial.find("mass").get("value")), 6.0)
    # com is unchanged by a mass rescale
    assert np.allclose(
        [float(x) for x in inertial.find("origin").get("xyz").split()],
        [0.0, 0.0, 0.1])
    inr = inertial.find("inertia")
    assert np.isclose(float(inr.get("ixx")), 0.6)   # 0.2 * 3
    assert np.isclose(float(inr.get("izz")), 1.2)   # 0.4 * 3


def test_writer_rescales_placeholder_when_no_geometry(tmp_path):
    """With no sw values and no mesh the inertial falls to the placeholder; a
    target mass still rescales it, so the URDF carries the requested weight."""
    from sw2robot.exporter import urdf_writer
    from sw2robot.exporter.model import Component, RobotModel
    c = Component(name="P-1", link_name="base_link", part_path=None,
                  is_subassembly=False, world=np.eye(4), fixed=True, dof=0,
                  mass_target=0.5)
    model = RobotModel(name="demo", components=[c], joints=[],
                       base_link="base_link")
    out = tmp_path / "urdf" / "demo.urdf"
    urdf_writer.write_urdf(model, str(out))
    inertial = ET.parse(out).getroot().find("link/inertial")
    assert np.isclose(float(inertial.find("mass").get("value")), 0.5)


# ---------------------------------------------------------------- build_model

def _graph(comp_states, robot="r"):
    from sw2robot.exporter.state import GraphState
    return GraphState(robot_name=robot, source_assembly="x.SLDASM",
                      components=comp_states)


def test_build_model_masses_sets_target_and_wins_over_density():
    """A `masses:` entry sets mass_target; when the same link ALSO has a
    `densities:` override, mass wins and density_override is cleared."""
    from sw2robot.exporter.model import build_model
    from sw2robot.exporter.state import ComponentState
    comps = [
        ComponentState(name="base", link_name="base", world=_eye(), fixed=True,
                       sw_mass=1.0, sw_com=[0, 0, 0], sw_inertia=[1, 0, 0, 1, 0, 1]),
        ComponentState(name="pcb", link_name="pcb", world=_eye(),
                       sw_mass=0.2, sw_com=[0, 0, 0],
                       sw_inertia=[1, 0, 0, 1, 0, 1]),
    ]
    config = {
        "base": "base",
        "joints": [{"parent": "base", "child": "pcb", "type": "fixed"}],
        "densities": {"pcb": 1500.0},
        "masses": {"pcb": 0.75},
    }
    model = build_model(_graph(comps), config=config)
    pcb = next(c for c in model.components if c.link_name == "pcb")
    assert pcb.mass_target == 0.75
    assert pcb.density_override is False     # mass wins over the density override


def test_build_model_masses_unmatched_or_invalid_is_ignored():
    from sw2robot.exporter.model import build_model
    from sw2robot.exporter.state import ComponentState
    comps = [ComponentState(name="base", link_name="base", world=_eye(),
                            fixed=True, sw_mass=1.0, sw_com=[0, 0, 0],
                            sw_inertia=[1, 0, 0, 1, 0, 1])]
    model = build_model(_graph(comps), config={
        "masses": {"nope": 1.0, "base": -3.0}})   # unmatched name + non-positive
    base = next(c for c in model.components if c.link_name == "base")
    assert base.mass_target is None


# ---------------------------------------------------------------- end-to-end build

def test_build_masses_block_sets_urdf_mass(tmp_path):
    """Through build(): a joints.yaml `masses:` block yields the exact <mass>
    in the working URDF (rescaled from the SW-native value)."""
    from sw2robot.exporter.export import build
    from sw2robot.exporter.state import ComponentState, GraphState

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
                           sw_inertia=[2, 0, 0, 2, 0, 2]),
        ])
    graph.save(str(pkg / "graph.json"))
    cfg = pkg / "demo.joints.yaml"
    cfg.write_text(yaml.safe_dump({
        "base": "base",
        "joints": [{"parent": "base", "child": "pcb", "type": "fixed"}],
        "masses": {"pcb": 0.8},          # from sw_mass 0.2 -> factor 4
    }))
    build(str(pkg), config_path=str(cfg))

    root = ET.parse(pkg / "urdf" / "demo.urdf").getroot()
    pcb = next(ln for ln in root.findall("link") if ln.get("name") == "pcb")
    inertial = pcb.find("inertial")
    assert np.isclose(float(inertial.find("mass").get("value")), 0.8)
    inr = inertial.find("inertia")
    assert np.isclose(float(inr.get("ixx")), 8.0)      # 2 * 4


# -------------------------------------------- SW mass-override flag (Part B)

def test_sw_mass_overridden_reads_override_flags():
    from sw2robot.exporter.model import _sw_mass_overridden

    class OverrideMass:
        OverrideMass = True
    assert _sw_mass_overridden(OverrideMass()) is True

    class OverrideCom:              # a different interface's flag name
        OverrideCenterOfMass = True
    assert _sw_mass_overridden(OverrideCom()) is True

    class NoOverride:
        OverrideMass = False
        OverrideCenterOfMass = False
        OverrideMomentsOfInertia = False
    assert _sw_mass_overridden(NoOverride()) is False

    class OlderInterface:           # none of the attributes present
        pass
    assert _sw_mass_overridden(OlderInterface()) is False


# -------------------------------------------- default-mass detection (Part B)

def _write_pkg(tmp_path, comp_states, cfg):
    from sw2robot.exporter.state import GraphState
    pkg = tmp_path / "pkg"
    pkg.mkdir()
    GraphState(robot_name="demo", source_assembly="x.SLDASM",
               components=comp_states).save(str(pkg / "graph.json"))
    if cfg is not None:
        (pkg / "demo.joints.yaml").write_text(yaml.safe_dump(cfg))
    return pkg


def test_default_mass_links_flags_unset_and_default_density(tmp_path):
    from sw2robot.editor.webserver import _default_mass_links
    from sw2robot.exporter.state import ComponentState
    comps = [
        # assigned material, non-default density -> OK
        ComponentState(name="Base", link_name="base", world=_eye(),
                       material="ABS", density=1040.0),
        # no material at all -> flagged
        ComponentState(name="NoMat", link_name="nomat", world=_eye()),
        # no material name but SW default density ~1000 -> flagged
        ComponentState(name="Def", link_name="defdens", world=_eye(),
                       density=1000.0),
        # a named material that happens to sit at ~1000 -> flagged too (the
        # "also flag ~1000" rule accepts this false-positive)
        ComponentState(name="Water", link_name="water", world=_eye(),
                       material="Water", density=1000.0),
    ]
    pkg = _write_pkg(tmp_path, comps, {"base": "base"})
    flagged = _default_mass_links(str(pkg), "urdf/demo.urdf")
    assert flagged == {"nomat", "defdens", "water"}


def test_default_mass_links_respects_resolutions(tmp_path):
    """SW mass override, an editor density/mass override, and an explicit
    acknowledgement each clear the flag."""
    from sw2robot.editor.webserver import _default_mass_links
    from sw2robot.exporter.state import ComponentState
    comps = [
        ComponentState(name="SwOver", link_name="swover", world=_eye(),
                       density=1000.0, sw_mass_overridden=True),
        ComponentState(name="DensOvr", link_name="densovr", world=_eye(),
                       density=1000.0),
        ComponentState(name="MassOvr", link_name="massovr", world=_eye()),
        ComponentState(name="Ack", link_name="ack", world=_eye()),
        ComponentState(name="Still", link_name="still", world=_eye()),
    ]
    cfg = {
        "densities": {"densovr": 500.0},
        "masses": {"massovr": 0.3},
        "mass_reviewed": ["ack"],
    }
    pkg = _write_pkg(tmp_path, comps, cfg)
    flagged = _default_mass_links(str(pkg), "urdf/demo.urdf")
    assert flagged == {"still"}      # only the untouched, material-less link


def test_default_mass_links_empty_without_graph(tmp_path):
    from sw2robot.editor.webserver import _default_mass_links
    assert _default_mass_links(str(tmp_path), "urdf/demo.urdf") == set()
    assert _default_mass_links(None, None) == set()
