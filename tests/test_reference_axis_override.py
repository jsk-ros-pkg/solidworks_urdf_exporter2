"""Per-joint override from a CAD reference axis (`axis_joints:`).

A designer who never used the SW2URDF add-in still draws SolidWorks reference
axes where the joints are.  The add-in import routes read those axes only as
part of an all-or-nothing reconstruction of the whole tree; this is the partial
form -- one axis names one joint, everything else keeps its inferred verdict.

Which pair an axis belongs to has to be SAID: an axis through a clevis passes
through fork, pin and fork alike, so the geometry does not identify the joint.
"""
import numpy as np
import pytest
from test_classify_geo import O, Z, coinc_planes, conc, dup

from sw2robot.exporter.model import build_model
from sw2robot.exporter.state import (
    ComponentState,
    GraphState,
    MateEdge,
    MateGeo,
    ReferenceAxisState,
)


def _comp(name, xyz=(0, 0, 0), fixed=False):
    w = np.eye(4)
    w[:3, 3] = xyz
    return ComponentState(name=name, link_name=name.replace("-", "_"),
                          part_path=None, is_subassembly=False,
                          world=[float(x) for x in w.flatten()], fixed=fixed)


def _welded_graph():
    """Two parts bolted together -- a pair of concentric axes, so inference
    says fixed."""
    a = _comp("frame-1", fixed=True)
    b = _comp("arm-1", (0, 0, 0.05))
    mates = dup(conc([0.01, 0, 0], Z), conc([-0.01, 0, 0], Z),
                coinc_planes(O, Z))
    edge = MateEdge(a="frame-1", b="arm-1",
                    types=[m["type"] for m in mates],
                    axis_point=list(O), axis_dir=list(Z),
                    mates=[MateGeo(**m) for m in mates])
    return GraphState(robot_name="t", source_assembly="t.SLDASM",
                      components=[a, b], edges=[edge], ground=["frame-1"])


AXIS = ReferenceAxisState(name="elbow_axis", document_point=[0.0, 0.0, 0.0],
                          document_direction=[0.0, 0.0, 1.0])


def _joint(model, child):
    return next(j for j in model.joints if j.child == child)


def test_baseline_is_fixed():
    # a bolt pair: two concentric axes leave no rotation at all
    model = build_model(_welded_graph())
    assert _joint(model, "arm_1").jtype == "fixed"


def test_assigned_axis_makes_the_joint():
    g = _welded_graph()
    g.reference_axes = [AXIS]
    model = build_model(g, config={"axis_joints": [
        {"axis": "elbow_axis", "parent": "frame-1", "child": "arm-1"}]})
    j = _joint(model, "arm_1")
    assert j.jtype == "revolute"
    assert abs(abs(float(np.dot(j.axis, [0, 0, 1]))) - 1.0) < 1e-6
    assert "elbow_axis" in (j.geo_note or "")


def test_assignment_accepts_link_names_too():
    g = _welded_graph()
    g.reference_axes = [AXIS]
    model = build_model(g, config={"axis_joints": [
        {"axis": "elbow_axis", "parent": "frame_1", "child": "arm_1"}]})
    assert _joint(model, "arm_1").jtype == "revolute"


def test_axis_joins_a_pair_with_no_mate_at_all():
    # the constraint was forgotten outright -- the commonest reason a designer
    # reaches for a reference axis
    g = _welded_graph()
    g.components.append(_comp("hand-1", (0, 0, 0.12)))
    g.reference_axes = [ReferenceAxisState(
        name="wrist", document_point=[0.0, 0.0, 0.12],
        document_direction=[1.0, 0.0, 0.0])]
    model = build_model(g, config={"axis_joints": [
        {"axis": "wrist", "parent": "arm-1", "child": "hand-1"}]})
    j = _joint(model, "hand_1")
    assert j.jtype == "revolute" and j.parent == "arm_1"
    assert abs(abs(float(np.dot(j.axis, [1, 0, 0]))) - 1.0) < 1e-6


def test_unassigned_axis_is_reported_not_guessed(capsys):
    g = _welded_graph()
    g.reference_axes = [AXIS]
    model = build_model(g)
    assert _joint(model, "arm_1").jtype == "fixed"     # unchanged
    out = capsys.readouterr().out
    assert "define no joint" in out and "axis_joints" in out


def test_reference_axes_off_ignores_even_an_assignment(capsys):
    g = _welded_graph()
    g.reference_axes = [AXIS]
    model = build_model(g, config={
        "reference_axes": "off",
        "axis_joints": [{"axis": "elbow_axis", "parent": "frame-1",
                         "child": "arm-1"}]})
    assert _joint(model, "arm_1").jtype == "fixed"


def test_bad_assignment_warns_and_leaves_inference_alone(capsys):
    g = _welded_graph()
    g.reference_axes = [AXIS]
    model = build_model(g, config={"axis_joints": [
        {"axis": "elbow_axis", "parent": "frame-1", "child": "nosuchpart-9"}]})
    assert _joint(model, "arm_1").jtype == "fixed"
    assert "axis_joints" in capsys.readouterr().out


if __name__ == "__main__":
    raise SystemExit(pytest.main([__file__, "-v"]))


def test_self_pair_is_refused_not_crashed(capsys):
    # frozenset(("a", "a")) has ONE element, and every consumer does
    # `a, b = tuple(key)` -- so this used to raise instead of warning
    g = _welded_graph()
    g.reference_axes = [AXIS]
    model = build_model(g, config={"axis_joints": [
        {"axis": "elbow_axis", "parent": "arm-1", "child": "arm-1"}]})
    assert _joint(model, "arm_1").jtype == "fixed"
    assert "same parent and child" in capsys.readouterr().out
