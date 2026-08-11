import base64
import json

import numpy as np
import pytest

from sw2robot.exporter.model import build_model
from sw2robot.exporter.state import (
    ComponentState,
    CoordinateSystemState,
    GraphState,
    MateEdge,
    ReferenceAxisState,
)
from sw2robot.exporter.sw2urdf_import import reconstruct_sw2urdf_config


def _m4(xyz):
    mat = np.eye(4)
    mat[:3, 3] = np.asarray(xyz, float)
    return [float(x) for x in mat.flatten()]


def _comp(name, xyz):
    return ComponentState(
        name=name,
        link_name=name.replace("-", "_"),
        world=_m4(xyz),
    )


def _coord(name, xyz):
    return CoordinateSystemState(
        name=name,
        document_from_frame=_m4(xyz),
    )


def _axis(name, point, direction):
    return ReferenceAxisState(
        name=name,
        document_point=[float(x) for x in point],
        document_direction=[float(x) for x in direction],
    )


def _edge(a, b, point, direction, mtype="CONCENTRIC"):
    return MateEdge(
        a=a,
        b=b,
        types=[mtype],
        axis_point=[float(x) for x in point],
        axis_dir=[float(x) for x in direction],
        mates=[],
    )


def _graph(include_knee_axis=True):
    comps = [
        _comp("base_link-1", [0.0, 0.0, 0.0]),
        _comp("hip_link-1", [0.0, 0.0, 0.1]),
        _comp("shin-1", [0.0, 0.0, 0.2]),
    ]
    coords = [
        _coord("base_origin", [0.0, 0.0, 0.0]),
        _coord("hip_origin", [0.0, 0.0, 0.1]),   # hip_link-1 -> hip_origin
        _coord("shin_origin", [0.0, 0.0, 0.2]),  # shin-1 -> shin_origin
    ]
    axes = [
        _axis("hip_y", [0.0, 0.0, 0.1], [-1.0, 0.0, 0.0]),
    ]
    if include_knee_axis:
        axes.append(_axis("knee_p", [0.0, 0.0, 0.2], [0.0, -2.0, 0.0]))
    edges = [
        _edge("base_link-1", "hip_link-1", [0.0, 0.0, 0.1], [1.0, 0.0, 0.0]),
        _edge("hip_link-1", "shin-1", [0.0, 0.0, 0.2], [0.0, 1.0, 0.0]),
        # Off-axis contact edge; must NOT be chosen as shin's parent.
        _edge("base_link-1", "shin-1", [0.2, 0.0, 0.2], [0.0, 0.0, 1.0],
              mtype="COINCIDENT"),
    ]
    return GraphState(
        robot_name="sw2",
        source_assembly="sw2.SLDASM",
        components=comps,
        edges=edges,
        ground=["base_link-1"],
        coordinate_systems=coords,
        reference_axes=axes,
    )


def _graph_convention_free(marker=None):
    graph = _graph(include_knee_axis=True)
    graph.coordinate_systems = [
        _coord("Origin_global", [0.0, 0.0, 0.0]),
        _coord("frameA", [0.0, 0.0, 0.1]),
        _coord("frameB", [0.0, 0.0, 0.2]),
    ]
    graph.sw2urdf_marker = marker
    return graph


def _reconstruct(graph):
    return reconstruct_sw2urdf_config(
        graph.components,
        graph.coordinate_systems,
        graph.reference_axes,
        graph.edges,
        graph.ground,
    )


def test_sw2urdf_detection_and_reconstruction():
    graph = _graph(include_knee_axis=True)
    cfg = _reconstruct(graph)
    assert cfg is not None
    assert cfg["root_link"] == "base_link-1"

    # Both origin naming forms are accepted:
    # - hip_link-1 -> hip_origin (drop _link)
    # - shin-1 -> shin_origin (exact stem)
    assert cfg["links"]["hip_link-1"]["origin_name"] == "hip_origin"
    assert cfg["links"]["shin-1"]["origin_name"] == "shin_origin"

    by_name = {joint["name"]: joint for joint in cfg["joints"]}
    assert set(by_name) == {"hip_y", "knee_p"}

    # Child assignment is by axis-line distance to origins.
    assert by_name["hip_y"]["child"] == "hip_link-1"
    assert by_name["knee_p"]["child"] == "shin-1"
    assert by_name["hip_y"]["axis_origin_distance"] == pytest.approx(0.0)
    assert by_name["knee_p"]["axis_origin_distance"] == pytest.approx(0.0)

    # Parent selection rejects the off-axis contact edge base<->shin.
    assert by_name["hip_y"]["parent"] == "base_link-1"
    assert by_name["knee_p"]["parent"] == "hip_link-1"

    # Sign normalization: largest-|component| must be positive.
    np.testing.assert_allclose(by_name["hip_y"]["axis_direction"], [1.0, 0.0, 0.0])
    np.testing.assert_allclose(by_name["knee_p"]["axis_direction"], [0.0, 1.0, 0.0])
    assert by_name["hip_y"]["flipped"] is True
    assert by_name["knee_p"]["flipped"] is True


def test_sw2urdf_detection_negative_without_origins():
    graph = _graph(include_knee_axis=True)
    graph.coordinate_systems = []
    assert _reconstruct(graph) is None


def test_sw2urdf_geometric_route_with_marker():
    graph = _graph_convention_free(marker="URDF Export Configuration (v1.4)")
    model = build_model(graph, config={"sw2urdf_config": "auto"})
    by_name = {j.name: j for j in model.joints}
    assert set(by_name) == {"hip_y", "knee_p"}
    assert by_name["hip_y"].parent == "base_link"
    assert by_name["hip_y"].child == "hip_link"
    assert by_name["knee_p"].parent == "hip_link"
    assert by_name["knee_p"].child == "shin"


def test_sw2urdf_geometric_route_not_attempted_without_marker():
    graph = _graph_convention_free(marker=None)
    model = build_model(graph, config={"sw2urdf_config": "auto"})
    assert [j.name for j in model.joints] == [
        "base_link_1__hip_link_1",
        "hip_link_1__shin_1",
    ]


def test_sw2urdf_geometric_route_falls_back_when_axis_has_no_edge():
    graph = _graph_convention_free(marker="URDF Export Configuration (v1.4)")
    graph.reference_axes = [
        _axis("hip_y", [0.0, 0.0, 0.1], [-1.0, 0.0, 0.0]),
        _axis("knee_p", [1.0, 1.0, 1.0], [0.0, 1.0, 0.0]),
    ]
    model = build_model(graph, config={"sw2urdf_config": "auto"})
    assert [j.name for j in model.joints] == [
        "base_link_1__hip_link_1",
        "hip_link_1__shin_1",
    ]


def test_sw2urdf_require_vs_auto_fallback():
    # Trigger detection but force inconsistency: shin has no axis.
    graph = _graph(include_knee_axis=False)

    model = build_model(graph, config={"sw2urdf_config": "auto"})
    assert len(model.joints) == len(model.components) - 1

    with pytest.raises(RuntimeError, match="sw2urdf_config=require"):
        build_model(graph, config={"sw2urdf_config": "require"})


def test_sw2urdf_user_joint_echo_keeps_config_and_merges_limits():
    # The generated joints.yaml snapshots the applied tree; feeding it back
    # (same parent/child/type) must keep the configured joint name/axis and
    # only take the user's explicit limits.
    graph = _graph(include_knee_axis=True)
    cfg = {"sw2urdf_config": "auto", "joints": [
        {"parent": "base_link-1", "child": "hip_link-1", "type": "revolute",
         "lower": -0.5, "upper": 0.5},
    ]}
    model = build_model(graph, config=cfg)
    by_name = {j.name: j for j in model.joints}
    assert "hip_y" in by_name and "knee_p" in by_name
    assert by_name["hip_y"].lower == -0.5
    assert by_name["hip_y"].upper == 0.5


def test_sw2urdf_user_joint_rewire_overrides_config():
    # A user entry that re-wires a configured child (different parent/type)
    # replaces that configured joint; the others stay.
    graph = _graph(include_knee_axis=True)
    cfg = {"sw2urdf_config": "auto", "joints": [
        {"parent": "base_link-1", "child": "shin-1", "type": "fixed"},
    ]}
    model = build_model(graph, config=cfg)
    by_name = {j.name: j for j in model.joints}
    assert "hip_y" in by_name
    assert "knee_p" not in by_name
    fixed = [j for j in model.joints if j.jtype == "fixed"]
    assert len(fixed) == 1
    assert fixed[0].child == "shin"


def test_sw2urdf_marker_defaults_none_on_older_graph_json():
    payload = json.loads(_graph(include_knee_axis=True).model_dump_json())
    payload.pop("sw2urdf_marker", None)
    payload.pop("sw2urdf_config_xml", None)
    graph = GraphState.model_validate_json(json.dumps(payload))
    assert graph.sw2urdf_marker is None
    assert graph.sw2urdf_config_xml is None


def test_sw2urdf_link_name_collision_falls_back():
    # A non-SW2URDF component whose link name equals a reconstructed stem
    # must not silently produce duplicate URDF link names.
    graph = _graph(include_knee_axis=True)
    stray = _comp("stray-9", [0.5, 0.5, 0.5])
    stray.link_name = "shin"
    graph.components.append(stray)

    model = build_model(graph, config={"sw2urdf_config": "auto"})
    joint_names = {j.name for j in model.joints}
    assert not joint_names & {"hip_y", "knee_p"}

    with pytest.raises(RuntimeError, match="sw2urdf_config=require"):
        build_model(graph, config={"sw2urdf_config": "require"})


def _pid_blob(name):
    raw = (b"\x10\x00\x00\x00\x01\x00\x00\x00" + b"\xff\xfe\xff"
           + bytes([len(name)]) + name.encode("utf-16-le"))
    return base64.b64encode(raw).decode()


def _payload_xml():
    # Minimal SW2URDF DataContract payload: base_link -> hip_link, one
    # revolute joint naming its RefAxis/CoordSys features explicitly.
    return f'''<Link xmlns:z="http://schemas.microsoft.com/2003/10/Serialization/" xmlns:i="http://www.w3.org/2001/XMLSchema-instance">
  <Attributes>
    <Attribute><AttributeType>name</AttributeType><Value>base_link</Value></Attribute>
  </Attributes>
  <ChildElements/>
  <ElementName>link</ElementName>
  <Children>
    <Link>
      <Attributes>
        <Attribute><AttributeType>name</AttributeType><Value>hip_link</Value></Attribute>
      </Attributes>
      <ChildElements>
        <URDFElement i:type="Joint">
          <Attributes>
            <Attribute z:Id="j1"><AttributeType>name</AttributeType><Value>hip_y</Value></Attribute>
            <Attribute z:Id="j2"><AttributeType>type</AttributeType><Value>revolute</Value></Attribute>
          </Attributes>
          <ChildElements/>
          <ElementName>joint</ElementName>
          <AxisName>hip_y</AxisName>
          <CoordinateSystemName>frameA</CoordinateSystemName>
          <NameAttribute z:Ref="j1"/>
          <TypeAttribute z:Ref="j2"/>
        </URDFElement>
      </ChildElements>
      <ElementName>link</ElementName>
      <Children/>
      <SWComponentPIDs><base64Binary>{_pid_blob("hip_link-1@asm")}</base64Binary></SWComponentPIDs>
    </Link>
  </Children>
  <SWComponentPIDs><base64Binary>{_pid_blob("base_link-1@asm")}</base64Binary></SWComponentPIDs>
</Link>'''


def _payload_graph(config_xml):
    # Convention-free coordsys names and NO marker: neither the name route
    # nor the geometric route can apply, so only the payload route can
    # produce the configured joint names.
    graph = GraphState(
        robot_name="sw2", source_assembly="sw2.SLDASM",
        components=[_comp("base_link-1", [0.0, 0.0, 0.0]),
                    _comp("hip_link-1", [0.0, 0.0, 0.1])],
        edges=[_edge("base_link-1", "hip_link-1",
                     [0.0, 0.0, 0.1], [1.0, 0.0, 0.0])],
        ground=["base_link-1"],
        coordinate_systems=[_coord("Origin_global", [0.0, 0.0, 0.0]),
                            _coord("frameA", [0.0, 0.0, 0.1])],
        reference_axes=[_axis("hip_y", [0.0, 0.0, 0.1], [-1.0, 0.0, 0.0])],
        sw2urdf_config_xml=config_xml,
    )
    return graph


def test_sw2urdf_payload_route_wins_without_naming_convention():
    model = build_model(_payload_graph(_payload_xml()),
                        config={"sw2urdf_config": "auto"})
    by_name = {j.name: j for j in model.joints}
    assert set(by_name) == {"hip_y"}
    assert by_name["hip_y"].jtype == "revolute"
    assert by_name["hip_y"].parent == "base_link"
    assert by_name["hip_y"].child == "hip_link"
    # axis from the NAMED RefAxis feature, sign-normalized to +X
    np.testing.assert_allclose(by_name["hip_y"].axis, [1.0, 0.0, 0.0])


def test_sw2urdf_payload_malformed_falls_back():
    model = build_model(_payload_graph("<not-a-link/>"),
                        config={"sw2urdf_config": "auto"})
    assert [j.name for j in model.joints] == ["base_link_1__hip_link_1"]


def test_sw2urdf_payload_scoped_coordsys_resolves_via_subassembly():
    # The add-in may reference a CoordSys inside a link sub-assembly; the
    # payload then scopes the name as "frameA <hip_link-1>".  It must resolve
    # through that component's sub-assembly frames (composed with the
    # component world), not fall back to the axis point: the sub coordsys
    # here sits 0.02 ALONG the axis, so the joint anchor lands at x=0.02.
    from sw2robot.exporter.state import SubGraph

    xml = _payload_xml().replace(
        "<CoordinateSystemName>frameA</CoordinateSystemName>",
        "<CoordinateSystemName>frameA &lt;hip_link-1&gt;"
        "</CoordinateSystemName>")
    graph = _payload_graph(xml)
    graph.coordinate_systems = [_coord("Origin_global", [0.0, 0.0, 0.0])]
    for c in graph.components:
        if c.name == "hip_link-1":
            c.part_path = r"C:\cad\hip_link.SLDASM"
    graph.subassemblies = {
        r"C:\cad\hip_link.SLDASM": SubGraph(coordinate_systems=[
            _coord("frameA", [0.02, 0.0, 0.0])]),
    }
    model = build_model(graph, config={"sw2urdf_config": "auto"})
    by_name = {j.name: j for j in model.joints}
    assert set(by_name) == {"hip_y"}
    np.testing.assert_allclose(by_name["hip_y"].xyz, [0.02, 0.0, 0.1],
                               atol=1e-12)


def _pid_blob2(*names):
    raw = b"\x10\x00\x00\x00\x02\x00\x00\x00"
    for name in names:
        raw += (b"\xff\xfe\xff" + bytes([len(name)])
                + name.encode("utf-16-le"))
    return base64.b64encode(raw).decode()


def test_sw2urdf_payload_multi_component_link_gets_fixed_members():
    # SW2URDF allows several loose components per link: the first PID acts
    # as the main (tree) component, the rest become rigid FIXED children.
    xml = _payload_xml().replace(
        _pid_blob("hip_link-1@asm"),
        _pid_blob2("hip_link-1@asm", "bracket-1@asm"))
    graph = _payload_graph(xml)
    graph.components.append(_comp("bracket-1", [0.0, 0.1, 0.1]))
    model = build_model(graph, config={"sw2urdf_config": "auto"})
    by_name = {j.name: j for j in model.joints}
    assert "hip_y" in by_name
    fixed = [j for j in model.joints if j.jtype == "fixed"]
    assert len(fixed) == 1
    assert fixed[0].parent == "hip_link"
    assert fixed[0].child == "bracket_1"


def test_sw2urdf_payload_shared_component_fails_partition_check():
    # A component claimed by TWO links (transmission-style config) cannot
    # form a URDF tree; the payload route must fail loudly and fall back.
    xml = _payload_xml().replace(
        _pid_blob("hip_link-1@asm"),
        _pid_blob2("hip_link-1@asm", "base_link-1@asm"))
    graph = _payload_graph(xml)
    model = build_model(graph, config={"sw2urdf_config": "auto"})
    assert [j.name for j in model.joints] == ["base_link_1__hip_link_1"]


def test_sw2urdf_payload_limits_mirrored_on_axis_flip():
    # The synthetic hip_y axis points -X, so normalization flips it to +X;
    # payload limits are expressed in the ORIGINAL joint coordinate and must
    # be mirrored: (lower, upper) -> (-upper, -lower).
    xml = _payload_xml().replace(
        "<ElementName>joint</ElementName>",
        '''<ChildElements><URDFElement z:Id="LIM" i:type="Limit">
          <Attributes>
            <Attribute z:Id="L1"><AttributeType>lower</AttributeType><Value i:type="a:double" xmlns:a="http://www.w3.org/2001/XMLSchema">-1.0</Value></Attribute>
            <Attribute z:Id="L2"><AttributeType>upper</AttributeType><Value i:type="a:double" xmlns:a="http://www.w3.org/2001/XMLSchema">0.5</Value></Attribute>
          </Attributes>
          <ChildElements/><ElementName>limit</ElementName>
          <LowerAttribute z:Ref="L1"/><UpperAttribute z:Ref="L2"/>
        </URDFElement></ChildElements><ElementName>joint</ElementName>
        <Limit z:Ref="LIM"/>''',
        1)
    graph = _payload_graph(xml)
    model = build_model(graph, config={"sw2urdf_config": "auto"})
    by_name = {j.name: j for j in model.joints}
    assert "hip_y" in by_name
    np.testing.assert_allclose(by_name["hip_y"].axis, [1.0, 0.0, 0.0])
    assert by_name["hip_y"].lower == pytest.approx(-0.5)
    assert by_name["hip_y"].upper == pytest.approx(1.0)
