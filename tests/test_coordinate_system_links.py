"""Named SolidWorks coordinate systems -> frame-only links.

A coordinate system the designer drew is a statement that a spot/orientation
matters (sensor mount, tool point), so each one becomes an empty ``dummy_link``
on a fixed joint: no visual, no collision, no inertial.  Covers who owns each
frame (part / sub-assembly / top-level assembly), the pose it lands at, the
config switch, and the owner lookup on the extraction side."""
import xml.etree.ElementTree as ET

import numpy as np


def _eye():
    return list(np.eye(4).flatten())


def _xlate(x=0.0, y=0.0, z=0.0):
    m = np.eye(4)
    m[:3, 3] = [x, y, z]
    return list(m.flatten())


def _graph(components, **kw):
    from sw2robot.exporter.state import GraphState
    return GraphState(robot_name="r", source_assembly="x.SLDASM",
                      components=components, **kw)


def _parts(*specs):
    """Component states: (name, world) pairs, all with a mass so the build has
    a valid inertial to write."""
    from sw2robot.exporter.state import ComponentState
    out = []
    for name, world, path in specs:
        out.append(ComponentState(
            name=name, link_name=name, part_path=path, world=world,
            fixed=(len(out) == 0), sw_mass=1.0, sw_com=[0, 0, 0],
            sw_inertia=[1, 0, 0, 1, 0, 1]))
    return out


def _frame(name, world):
    from sw2robot.exporter.state import CoordinateSystemState
    return CoordinateSystemState(name=name, document_from_frame=world)


def _ports(model):
    return {p.name: p for p in model.ports}


# --------------------------------------------------------- part-level frames

def test_part_frame_becomes_a_port_on_every_instance_of_that_part():
    """A frame drawn INSIDE a .SLDPRT travels with the part: both instances get
    one, each in its own link's frame, and the repeated name is disambiguated."""
    from sw2robot.exporter.model import build_model
    comps = _parts(("base", _eye(), "base.SLDPRT"),
                   ("gripper_l", _xlate(1, 0, 0), "gripper.SLDPRT"),
                   ("gripper_r", _xlate(-1, 0, 0), "gripper.SLDPRT"))
    graph = _graph(comps, part_coordinate_systems={
        "gripper.SLDPRT": [_frame("tcp", _xlate(0, 0, 0.05))]})
    config = {"base": "base",
              "joints": [{"parent": "base", "child": "gripper_l",
                          "type": "fixed"},
                         {"parent": "base", "child": "gripper_r",
                          "type": "fixed"}]}

    model = build_model(graph, config=config)

    ports = _ports(model)
    assert set(ports) == {"gripper_l_tcp", "gripper_r_tcp"}
    for name, link in (("gripper_l_tcp", "gripper_l"),
                       ("gripper_r_tcp", "gripper_r")):
        assert ports[name].parent_link == link
        # the frame is authored in the PART's own frame, so the offset is the
        # same on both instances -- the instance world is already the anchor
        assert np.allclose(ports[name].xyz, [0, 0, 0.05], atol=1e-9)
        assert np.allclose(ports[name].rpy, [0, 0, 0], atol=1e-9)


def test_part_frame_keeps_its_own_name_when_it_is_unique():
    from sw2robot.exporter.model import build_model
    comps = _parts(("base", _eye(), "base.SLDPRT"),
                   ("hand", _xlate(0.2, 0, 0), "hand.SLDPRT"))
    graph = _graph(comps, part_coordinate_systems={
        "hand.SLDPRT": [_frame("end_coords", _xlate(0, 0, 0.1))]})
    model = build_model(graph, config={
        "base": "base",
        "joints": [{"parent": "base", "child": "hand", "type": "fixed"}]})

    ports = _ports(model)
    assert set(ports) == {"end_coords"}
    assert ports["end_coords"].parent_link == "hand"


# ----------------------------------------------------- assembly-level frames

def test_top_level_frame_hangs_off_the_component_it_was_built_on():
    """The owner recorded at extraction (IEntity.GetComponent) decides the
    parent link; the transform is in the ASSEMBLY frame, so it is re-expressed
    in that link's anchor."""
    from sw2robot.exporter.model import build_model
    comps = _parts(("base", _eye(), None),
                   ("head", _xlate(0, 0, 1.0), None))
    frame = _frame("camera", _xlate(0, 0, 1.3))
    frame.owner_component = "head"
    model = build_model(_graph(comps, coordinate_systems=[frame]), config={
        "base": "base",
        "joints": [{"parent": "base", "child": "head", "type": "fixed"}]})

    ports = _ports(model)
    assert set(ports) == {"camera"}
    assert ports["camera"].parent_link == "head"
    # 1.3 in the assembly frame, on a link sitting at 1.0 -> 0.3 locally
    assert np.allclose(ports["camera"].xyz, [0, 0, 0.3], atol=1e-9)


def test_top_level_frame_without_an_owner_falls_back_to_the_base_link():
    from sw2robot.exporter.model import build_model
    comps = _parts(("base", _eye(), None), ("head", _xlate(0, 0, 1.0), None))
    model = build_model(
        _graph(comps, coordinate_systems=[_frame("world_ref",
                                                 _xlate(0.4, 0, 0))]),
        config={"base": "base",
                "joints": [{"parent": "base", "child": "head",
                            "type": "fixed"}]})

    ports = _ports(model)
    assert ports["world_ref"].parent_link == "base"
    assert np.allclose(ports["world_ref"].xyz, [0.4, 0, 0], atol=1e-9)


def test_subassembly_frame_lands_on_each_instance_of_that_subassembly():
    from sw2robot.exporter.model import build_model
    from sw2robot.exporter.state import ComponentState, SubGraph
    comps = _parts(("base", _eye(), None),
                   ("wrist_l", _xlate(0, 0.5, 0), "wrist.SLDASM"),
                   ("wrist_r", _xlate(0, -0.5, 0), "wrist.SLDASM"))
    for c in comps[1:]:
        c.is_subassembly = True
    # the sub-assembly is NOT expanded (its internals do not move), so each
    # instance stays one link and carries its document's frame
    sub = SubGraph(components=[ComponentState(name="inner", link_name="inner",
                                              world=_eye())],
                   coordinate_systems=[_frame("mount", _xlate(0, 0, 0.02))])
    graph = _graph(comps, subassemblies={"wrist.SLDASM": sub})
    model = build_model(graph, config={
        "base": "base",
        "joints": [{"parent": "base", "child": "wrist_l", "type": "fixed"},
                   {"parent": "base", "child": "wrist_r", "type": "fixed"}]})

    ports = _ports(model)
    assert set(ports) == {"wrist_l_mount", "wrist_r_mount"}
    assert ports["wrist_l_mount"].parent_link == "wrist_l"
    assert np.allclose(ports["wrist_r_mount"].xyz, [0, 0, 0.02], atol=1e-9)


# ------------------------------------------------------------ config switch

def test_coordinate_system_links_off_emits_nothing():
    from sw2robot.exporter.model import build_model
    comps = _parts(("base", _eye(), "base.SLDPRT"))
    graph = _graph(comps, part_coordinate_systems={
        "base.SLDPRT": [_frame("tcp", _eye())]})
    for value in ("off", False):
        model = build_model(graph, config={"base": "base",
                                           "coordinate_system_links": value})
        assert model.ports == []


def test_coordinate_system_links_list_selects_by_cad_name():
    from sw2robot.exporter.model import build_model
    comps = _parts(("base", _eye(), "base.SLDPRT"))
    graph = _graph(comps, part_coordinate_systems={
        "base.SLDPRT": [_frame("keep", _eye()), _frame("drop", _eye())]})
    model = build_model(graph, config={"base": "base",
                                       "coordinate_system_links": ["keep"]})
    assert list(_ports(model)) == ["keep"]


def test_frames_an_sw2urdf_config_already_uses_as_link_anchors_are_skipped():
    """`auto` (the default) must not duplicate the add-in's per-link frames as
    dummy links -- those already ARE the link origins.  `all` emits them
    anyway."""
    from sw2robot.exporter.model import Component, build_model, coordinate_system_ports
    comps = _parts(("base", _eye(), None), ("arm", _xlate(0, 0, 0.5), None))
    graph = _graph(comps, coordinate_systems=[
        _frame("Origin_arm", _xlate(0, 0, 0.5)),
        _frame("tool_tip", _xlate(0, 0, 0.9))])
    model = build_model(graph, config={
        "base": "base",
        "joints": [{"parent": "base", "child": "arm", "type": "fixed"}]})
    built = [Component(name=c.name, link_name=c.link_name, part_path=None,
                       is_subassembly=False, world=np.eye(4), fixed=False,
                       dof=None) for c in model.components]
    anchors = {c.name: np.eye(4) for c in built}
    base = built[0]

    consumed = {"Origin_arm"}
    auto = coordinate_system_ports(graph, built, model.joints, anchors, base,
                                   mode="auto", consumed=consumed)
    assert [p.name for p in auto] == ["tool_tip"]
    every = coordinate_system_ports(graph, built, model.joints, anchors, base,
                                    mode="all", consumed=consumed)
    assert sorted(p.name for p in every) == ["Origin_arm", "tool_tip"]


# ------------------------------------------------------------- URDF output

def test_frame_link_has_no_visual_collision_or_inertial(tmp_path):
    from sw2robot.exporter.model import build_model
    from sw2robot.exporter.urdf_writer import write_urdf
    comps = _parts(("base", _eye(), "base.SLDPRT"))
    graph = _graph(comps, part_coordinate_systems={
        "base.SLDPRT": [_frame("tcp", _xlate(0, 0, 0.3))]})
    model = build_model(graph, config={"base": "base"})
    out = tmp_path / "urdf" / "r.urdf"
    write_urdf(model, str(out))

    root = ET.parse(out).getroot()
    links = {ln.get("name"): ln for ln in root.findall("link")}
    assert "tcp" in links
    tcp = links["tcp"]
    assert tcp.find("visual") is None
    assert tcp.find("collision") is None
    assert tcp.find("inertial") is None
    joint = next(j for j in root.findall("joint")
                 if j.find("child").get("link") == "tcp")
    assert joint.get("type") == "fixed"
    assert np.allclose([float(v) for v in
                        joint.find("origin").get("xyz").split()],
                       [0, 0, 0.3], atol=1e-9)


def test_a_frame_named_in_japanese_gets_a_usable_link_name():
    """SolidWorks in Japanese calls its coordinate systems '座標系1', which
    plain ASCII sanitizing would reduce to 'c_1'."""
    from sw2robot.exporter.model import _frame_link_name
    assert _frame_link_name("座標系1") == "coord1"
    assert _frame_link_name("座標系") == "coord"
    assert _frame_link_name("end_coords") == "end_coords"
    assert _frame_link_name("TCP 2") == "TCP_2"


def test_japanese_named_frames_stay_unique_per_link():
    from sw2robot.exporter.model import build_model
    comps = _parts(("base", _eye(), "base.SLDPRT"),
                   ("hand", _xlate(0.3, 0, 0), "hand.SLDPRT"))
    graph = _graph(comps, part_coordinate_systems={
        "base.SLDPRT": [_frame("座標系1", _eye())],
        "hand.SLDPRT": [_frame("座標系1", _eye())]})
    model = build_model(graph, config={
        "base": "base",
        "joints": [{"parent": "base", "child": "hand", "type": "fixed"}]})

    assert sorted(_ports(model)) == ["base_coord1", "hand_coord1"]


# -------------------------------------------------- extraction: owner lookup

class _Entity:
    def __init__(self, component):
        self._component = component

    def GetComponent(self):
        return self._component


class _Component:
    def __init__(self, name):
        self.Name2 = name


def test_coordsys_owner_component_reads_the_origin_selection(monkeypatch):
    from sw2robot.exporter import model as model_mod
    monkeypatch.setattr(model_mod, "as_iface", lambda obj, _name: obj)

    class _Data:
        OriginEntity = _Entity(_Component("head-1"))

    assert model_mod._coordsys_owner_component(_Data(), doc=None) == "head-1"


def test_coordsys_owner_component_is_none_for_assembly_own_geometry(
        monkeypatch):
    from sw2robot.exporter import model as model_mod
    monkeypatch.setattr(model_mod, "as_iface", lambda obj, _name: obj)

    class _NoComponent:
        OriginEntity = _Entity(None)

    class _NoEntity:
        OriginEntity = None

    assert model_mod._coordsys_owner_component(_NoComponent(), doc=None) is None
    assert model_mod._coordsys_owner_component(_NoEntity(), doc=None) is None
