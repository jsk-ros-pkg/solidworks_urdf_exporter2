"""graph.json as a published, versioned contract with declared joints.

An extractor for another CAD (or an agent) writes graph.json without importing
sw2robot, so the schema file must match the model, old files must keep loading,
and joints the source DECLARED must reach the URDF as given instead of being
re-inferred from mates.
"""
import json

import numpy as np
import pytest
from pydantic import ValidationError

from sw2robot.exporter import schema as schema_cli
from sw2robot.exporter.model import build_model, to_graph_state
from sw2robot.exporter.state import (
    SCHEMA_VERSION,
    ComponentState,
    DeclaredJoint,
    GraphState,
    MateEdge,
    MateGeo,
    graph_json_schema,
)


def _comp(name, xyz=(0, 0, 0), fixed=False):
    w = np.eye(4)
    w[:3, 3] = xyz
    return ComponentState(name=name, link_name=name, fixed=fixed,
                          world=[float(x) for x in w.flatten()])


def _hinge_mates(point, direction):
    """One concentric mate on the given line: what a pin hinge extracts to."""
    return [MateGeo(type="CONCENTRIC", etypes=[4, 4],
                    points=[list(point), list(point)],
                    dirs=[list(direction), list(direction)],
                    radii=[0.004, 0.004])]


def _graph(joints, edges=(), components=None):
    comps = components or [_comp("base", fixed=True), _comp("arm", (0.1, 0, 0))]
    return GraphState(robot_name="t", source_assembly="t.iam",
                      components=comps, edges=list(edges), ground=["base"],
                      joints=list(joints))


def _joint(model, child):
    return next(j for j in model.joints if j.child == child)


def test_schema_file_matches_the_model():
    assert schema_cli.main(["--check"]) == 0


def test_a_dumped_graph_validates_against_the_schema():
    jsonschema = pytest.importorskip("jsonschema")
    graph = _graph([DeclaredJoint(parent="base", child="arm", type="revolute",
                                  axis_dir=[0, 0, 1], source="ai")])
    jsonschema.validate(json.loads(graph.model_dump_json()), graph_json_schema())


def test_graph_without_a_version_loads_as_version_1(tmp_path):
    path = tmp_path / "graph.json"
    data = json.loads(_graph([]).model_dump_json())
    del data["schema_version"]
    path.write_text(json.dumps(data), encoding="utf-8")
    assert GraphState.load(str(path)).schema_version == 1


def test_new_graphs_carry_the_current_version(tmp_path):
    path = tmp_path / "graph.json"
    _graph([]).save(str(path))
    assert GraphState.load(str(path)).schema_version == SCHEMA_VERSION


def test_a_newer_graph_is_refused(tmp_path):
    path = tmp_path / "graph.json"
    data = json.loads(_graph([]).model_dump_json())
    data["schema_version"] = SCHEMA_VERSION + 1
    path.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(ValueError, match="newer"):
        GraphState.load(str(path))


def test_validate_cli_reports_a_bad_file(tmp_path, capsys):
    path = tmp_path / "graph.json"
    path.write_text(json.dumps({"robot_name": "t"}), encoding="utf-8")
    assert schema_cli.main(["--validate", str(path)]) == 1
    assert "invalid" in capsys.readouterr().err


def test_solidworks_extracts_name_their_source():
    graph = to_graph_state([], {}, set(), "r", "r.SLDASM")
    assert graph.source.cad == "solidworks"
    assert graph.source.extractor.startswith("sw2robot ")


@pytest.mark.parametrize("kwargs, message", [
    ({"type": "revolute"}, "axis_dir"),
    ({"type": "prismatic", "axis_dir": [0, 0, 0]}, "axis_dir"),
    ({"type": "continuous", "axis_dir": [0, 0, 1], "lower": -1.0}, "continuous"),
    ({"type": "revolute", "axis_dir": [0, 0, 1], "lower": 1.0, "upper": 0.0},
     "lower"),
    ({"type": "fixed", "child": "base"}, "parent and child"),
])
def test_malformed_declared_joints_are_rejected(kwargs, message):
    args = {"parent": "base", "child": "arm", **kwargs}
    with pytest.raises(ValidationError, match=message):
        DeclaredJoint(**args)


def test_declared_revolute_needs_no_mates():
    graph = _graph([DeclaredJoint(
        parent="base", child="arm", type="revolute", name="shoulder",
        axis_point=[0.1, 0, 0], axis_dir=[0, 0, 2], lower=-0.5, upper=1.0)])
    j = _joint(build_model(graph), "arm")
    assert j.jtype == "revolute"
    assert j.name == "shoulder"
    np.testing.assert_allclose(j.axis, [0, 0, 1])
    assert (j.lower, j.upper) == (-0.5, 1.0)
    np.testing.assert_allclose(j.xyz, [0.1, 0, 0])


def test_declared_axis_flips_when_the_tree_roots_the_pair_the_other_way():
    # declared with parent/child swapped relative to the tree (base is ground)
    graph = _graph([DeclaredJoint(
        parent="arm", child="base", type="prismatic",
        axis_point=[0, 0, 0], axis_dir=[1, 0, 0], lower=-0.02, upper=0.03)])
    j = _joint(build_model(graph), "arm")
    assert j.jtype == "prismatic"
    np.testing.assert_allclose(j.axis, [-1, 0, 0])
    assert (j.lower, j.upper) == (-0.02, 0.03)


def test_open_range_keeps_the_default_travel():
    graph = _graph([DeclaredJoint(parent="base", child="arm", type="prismatic",
                                  axis_dir=[0, 1, 0])])
    j = _joint(build_model(graph), "arm")
    assert (j.lower, j.upper) == (-0.05, 0.05)


def test_declared_fixed_overrides_a_hinge_the_mates_imply():
    edge = MateEdge(a="base", b="arm", types=["CONCENTRIC"],
                    mates=_hinge_mates([0.1, 0, 0], [0, 0, 1]))
    assert _joint(build_model(_graph([], [edge])), "arm").jtype == "revolute"
    fixed = DeclaredJoint(parent="base", child="arm", type="fixed")
    assert _joint(build_model(_graph([fixed], [edge])), "arm").jtype == "fixed"


def test_multi_dof_joint_is_exported_fixed_with_a_warning(capsys):
    ball = DeclaredJoint(parent="base", child="arm", type="ball",
                         source_type="INVENTOR_BALL")
    j = _joint(build_model(_graph([ball])), "arm")
    assert j.jtype == "fixed"
    assert "exported FIXED" in capsys.readouterr().out


def test_declared_joint_on_a_fastener_is_not_welded():
    comps = [_comp("base", fixed=True), _comp("shoulder_pin", (0.1, 0, 0))]
    graph = _graph([DeclaredJoint(parent="base", child="shoulder_pin",
                                  type="continuous", axis_dir=[0, 0, 1])],
                   components=comps)
    assert _joint(build_model(graph), "shoulder_pin").jtype == "continuous"


def test_joint_on_an_unknown_component_is_skipped(capsys):
    graph = _graph([DeclaredJoint(parent="base", child="ghost",
                                  type="revolute", axis_dir=[0, 0, 1])])
    build_model(graph)
    assert "not in the model" in capsys.readouterr().out


def test_declared_joints_can_be_switched_off():
    graph = _graph([DeclaredJoint(parent="base", child="arm", type="revolute",
                                  axis_dir=[0, 0, 1])])
    model = build_model(graph, config={"use_declared_joints": False})
    assert _joint(model, "arm").jtype == "fixed"


def test_declared_fixed_twin_is_not_retyped_by_the_mirror_fallback():
    # two copies of one part: the left is declared a hinge, the right is
    # declared welded.  The mirror fallback copies a twin's joint onto a
    # mate-less copy; it must not override what the source declared.
    comps = [_comp("base", fixed=True),
             _comp("finger_l", (0.1, 0, 0)), _comp("finger_r", (-0.1, 0, 0))]
    for c in comps[1:]:
        c.part_path = "finger.SLDPRT"
    graph = _graph([
        DeclaredJoint(parent="base", child="finger_l", type="revolute",
                      axis_point=[0.1, 0, 0], axis_dir=[0, 0, 1]),
        DeclaredJoint(parent="base", child="finger_r", type="fixed"),
    ], components=comps)
    model = build_model(graph)
    assert _joint(model, "finger_l").jtype == "revolute"
    assert _joint(model, "finger_r").jtype == "fixed"


def test_schema_alone_rejects_a_hinge_without_an_axis():
    # a non-Python extractor only has graph.schema.json to check against
    jsonschema = pytest.importorskip("jsonschema")
    data = json.loads(_graph([]).model_dump_json())
    data["joints"] = [{"parent": "base", "child": "arm", "type": "prismatic"}]
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(data, graph_json_schema())
    data["joints"] = [{"parent": "base", "child": "arm", "type": "continuous",
                       "axis_dir": [0, 0, 1], "lower": -1.0}]
    with pytest.raises(jsonschema.ValidationError):
        jsonschema.validate(data, graph_json_schema())


def test_a_misspelt_declared_joint_key_is_refused():
    with pytest.raises(ValidationError, match="soruce"):
        DeclaredJoint(parent="base", child="arm", type="fixed", soruce="ai")


@pytest.mark.parametrize("version", [False, 0, -1, "2"])
def test_a_bogus_schema_version_is_refused(tmp_path, version):
    path = tmp_path / "graph.json"
    data = json.loads(_graph([]).model_dump_json())
    data["schema_version"] = version
    path.write_text(json.dumps(data), encoding="utf-8")
    with pytest.raises(ValueError, match="not a version number"):
        GraphState.load(str(path))


def test_a_component_pose_must_be_a_4x4_matrix():
    with pytest.raises(ValidationError, match="world"):
        ComponentState(name="a", link_name="a", world=[1.0, 0.0, 0.0])


def test_declared_base_beats_the_hub_heuristic():
    # 'arm' has more connections, so the heuristic would root the tree there
    comps = [_comp("base", fixed=True), _comp("arm", (0.1, 0, 0)),
             _comp("hand", (0.2, 0, 0))]
    joints = [DeclaredJoint(parent="base", child="arm", type="fixed"),
              DeclaredJoint(parent="arm", child="hand", type="fixed")]
    graph = _graph(joints, components=comps)
    assert build_model(graph).base_link == "arm"
    graph.base = "base"
    assert build_model(graph).base_link == "base"
    assert build_model(graph, config={"base": "hand"}).base_link == "hand"
