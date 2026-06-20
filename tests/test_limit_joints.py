"""SolidWorks LimitDistance/LimitAngle mates become prismatic/revolute joints.

A limit mate IS a real slider/hinge (travel between min..max).  The geometric
classifier reads it as a plain DISTANCE/ANGLE constraint and over-fixes it, and
the global-lock solve would then demote it -- so a graph's ``limit_joints`` must
win: the edge becomes prismatic/revolute with the CAD axis + travel limits, and
the global solve must NOT lock it back to fixed.
"""
import numpy as np

from sw2robot.exporter.model import build_model, classify_edge_auto
from sw2robot.exporter.state import (
    ComponentState,
    GraphState,
    LimitJoint,
    MateEdge,
)


def test_classify_edge_auto_honours_limit_joint():
    rec = {"types": ["DISTANCE"], "axis": None, "mates": [],
           "limit_joint": {"type": "prismatic",
                           "axis": (np.zeros(3), np.array([0.0, 1.0, 0.0])),
                           "lower": -0.05, "upper": 0.15}}
    jt, ax, note = classify_edge_auto(rec)
    assert jt == "prismatic"
    np.testing.assert_allclose(ax[1], [0, 1, 0])
    assert "limit mate" in note


def _comp(name, xyz=(0, 0, 0)):
    w = np.eye(4)
    w[:3, 3] = xyz
    return ComponentState(name=name, link_name=name,
                          world=[float(x) for x in w.flatten()], fixed=False)


def _graph(limit):
    # base + slider, with a DISTANCE mate between them (over-constraining) AND a
    # limit-mate joint that must win
    base = _comp("base")
    base.fixed = True
    slider = _comp("slider", (0.1, 0, 0))
    edge = MateEdge(a="base", b="slider", types=["DISTANCE"],
                    axis_point=None, axis_dir=None, mates=None)
    return GraphState(
        robot_name="t", source_assembly="t.SLDASM",
        components=[base, slider], edges=[edge], ground=["base"],
        limit_joints=[limit] if limit else [])


def test_limit_joint_builds_prismatic_with_limits():
    # ``a`` (the reference whose +axis grows the mate distance) is the slider =
    # the tree child, so the axis is used as-is (no parent/child flip)
    lj = LimitJoint(a="slider", b="base", type="prismatic",
                    axis_point=[0, 0, 0], axis_dir=[0, 1, 0],
                    lower=-0.05, upper=0.15)
    model = build_model(_graph(lj))
    j = next(j for j in model.joints if j.child == "slider")
    assert j.jtype == "prismatic"
    np.testing.assert_allclose([round(x) for x in j.axis], [0, 1, 0])
    assert abs(j.lower - (-0.05)) < 1e-9
    assert abs(j.upper - 0.15) < 1e-9


def test_limit_joint_axis_flips_for_the_other_side():
    # if the tree child is the OTHER mate side (not the reference ``a``), its +
    # motion shrinks the distance, so the axis flips to keep joint>0 = toward max
    lj = LimitJoint(a="base", b="slider", type="prismatic",
                    axis_point=[0, 0, 0], axis_dir=[0, 1, 0],
                    lower=-0.05, upper=0.15)
    model = build_model(_graph(lj))
    j = next(j for j in model.joints if j.child == "slider")
    np.testing.assert_allclose([round(x) for x in j.axis], [0, -1, 0])


def test_without_limit_joint_the_distance_mate_fixes_it():
    # same geometry, no limit joint -> the lone DISTANCE constraint leaves it
    # NOT a clean prismatic (regression guard that the win comes from the limit)
    model = build_model(_graph(None))
    j = next(j for j in model.joints if j.child == "slider")
    assert j.jtype == "fixed"


def test_rigid_group_stays_together_under_limit_joints():
    """Bed-plate scenario: a plate is fixed to a carriage that slides, AND has a
    redundant limit slide to a separately-sliding toolhead.  The plate must stay
    FIXED under its carriage (one rigid group) -- not get pulled under the
    toolhead by the redundant limit mate, which is a loop closure."""
    base = _comp("base"); base.fixed = True
    carriage = _comp("carriage", (0, 0.1, 0))
    plate = _comp("plate", (0, 0.1, 0.1))
    tool = _comp("tool", (0.1, 0, 0))
    # plate <-> carriage: a (force-)fixed structural weld = rigid backbone
    e = MateEdge(a="carriage", b="plate", types=["COINCIDENT"],
                 axis_point=None, axis_dir=None, mates=None)
    g = GraphState(
        robot_name="t", source_assembly="t.SLDASM",
        components=[base, carriage, plate, tool], edges=[e], ground=["base"],
        limit_joints=[
            LimitJoint(a="base", b="carriage", type="prismatic",
                       axis_point=[0, 0, 0], axis_dir=[0, 1, 0],
                       lower=-0.05, upper=0.05),
            LimitJoint(a="base", b="tool", type="prismatic",
                       axis_point=[0, 0, 0], axis_dir=[1, 0, 0],
                       lower=0.0, upper=0.1),
            LimitJoint(a="tool", b="plate", type="prismatic",  # redundant / loop
                       axis_point=[0, 0, 0], axis_dir=[1, 0, 0],
                       lower=0.0, upper=0.1),
        ])
    model = build_model(g, config={"force_fixed": [["carriage", "plate"]]})
    jt = {j.child: j for j in model.joints}
    assert jt["plate"].jtype == "fixed"
    assert jt["plate"].parent == "carriage"     # stays in the carriage's group
    assert jt["carriage"].jtype == "prismatic"
    assert jt["tool"].jtype == "prismatic"
