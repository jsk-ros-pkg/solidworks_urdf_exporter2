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
    MateGeo,
    SubGraph,
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


def test_template_round_trips_joint_limits(tmp_path):
    # write_template must emit lower/upper -- a joint without a SolidWorks limit
    # mate (a promoted slide) has its travel range ONLY in the config, so a
    # template that drops it silently resets the joint on the next build
    from sw2robot.exporter import jointcfg
    lj = LimitJoint(a="slider", b="base", type="prismatic",
                    axis_point=[0, 0, 0], axis_dir=[0, 1, 0],
                    lower=-0.05, upper=0.15)
    model = build_model(_graph(lj))
    out = tmp_path / "j.joints.yaml"
    jointcfg.write_template(model, str(out))
    txt = out.read_text(encoding="utf-8")
    assert "lower: -0.05000" in txt
    assert "upper: 0.15000" in txt


def test_limit_joint_axis_flips_for_the_other_side():
    # if the tree child is the OTHER mate side (not the reference ``a``), its +
    # motion shrinks the distance, so the axis flips to keep joint>0 = toward max
    lj = LimitJoint(a="base", b="slider", type="prismatic",
                    axis_point=[0, 0, 0], axis_dir=[0, 1, 0],
                    lower=-0.05, upper=0.15)
    model = build_model(_graph(lj))
    j = next(j for j in model.joints if j.child == "slider")
    np.testing.assert_allclose([round(x) for x in j.axis], [0, -1, 0])


def test_revolute_limit_joint_pivots_on_concentric_axis():
    # a LimitAngle mate stores its plane point, which is NOT on the rotation
    # axis; the hinge is the concentric mate.  build_model must pivot the
    # revolute on the concentric axis line, not the (offset) angle point.
    base = _comp("base"); base.fixed = True
    door = _comp("door", (0.2, 0, 0))
    conc = MateGeo(type="CONCENTRIC", etypes=[4, 4],
                   points=[[0.05, 0.0, 0.0], [0.05, 0.0, 0.02]],
                   dirs=[[0, 0, 1], [0, 0, 1]], radii=[None, None])
    edge = MateEdge(a="base", b="door", types=["CONCENTRIC"],
                    axis_point=None, axis_dir=None, mates=[conc])
    lj = LimitJoint(a="door", b="base", type="revolute",
                    axis_point=[0.3, 0.4, 0.1],   # bogus off-axis angle point
                    axis_dir=[0, 0, 1], lower=-0.5, upper=0.5)
    g = GraphState(robot_name="t", source_assembly="t.SLDASM",
                   components=[base, door], edges=[edge], ground=["base"],
                   limit_joints=[lj])
    model = build_model(g)
    j = next(jj for jj in model.joints if jj.child == "door")
    assert j.jtype == "revolute"
    # the axis line goes through the concentric point (x=0.05, y=0), NOT (0.3,0.4)
    np.testing.assert_allclose(j.sw_axis_point[:2], [0.05, 0.0], atol=1e-6)


def test_without_limit_joint_the_distance_mate_fixes_it():
    # same geometry, no limit joint -> the lone DISTANCE constraint leaves it
    # NOT a clean prismatic (regression guard that the win comes from the limit)
    model = build_model(_graph(None))
    j = next(j for j in model.joints if j.child == "slider")
    assert j.jtype == "fixed"


def _sub_slider_graph():
    """A linear module sub-assembly (case + slider on a limit DISTANCE mate)
    mounted on a base, its instance turned 90 deg about z."""
    case = _comp("case-1")
    slider = _comp("slider-1", (0, 0.05, 0))
    sub = SubGraph(
        components=[case, slider],
        edges=[MateEdge(a="case-1", b="slider-1", types=["DISTANCE"])],
        ground=["case-1"],
        limit_joints=[LimitJoint(a="slider-1", b="case-1", type="prismatic",
                                 axis_point=[0, 0, 0], axis_dir=[0, 1, 0],
                                 lower=-0.05, upper=0.15)])
    base = _comp("base")
    base.fixed = True
    inst = _comp("module-1", (0.2, 0, 0))
    w = np.array(inst.world, float).reshape(4, 4)
    w[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
    inst.world = [float(x) for x in w.flatten()]
    inst.is_subassembly = True
    inst.part_path = "module.SLDASM"
    return GraphState(
        robot_name="t", source_assembly="t.SLDASM",
        components=[base, inst],
        edges=[MateEdge(a="base", b="module-1", types=["COINCIDENT"],
                        mates=[MateGeo(type="COINCIDENT", etypes=[3, 3],
                                       points=[[0.2, 0, 0]] * 2,
                                       dirs=[[0, 0, 1]] * 2,
                                       radii=[None, None],
                                       owners=["base", "module-1/case-1"])])],
        ground=["base"], subassemblies={"module.SLDASM": sub})


def test_limit_joint_inside_a_subassembly_builds_prismatic():
    """A limit mate authored INSIDE a sub-assembly is a real slider too: the
    sub-assembly expands for it (its lone DISTANCE edge classifies fixed), and
    the slide axis is carried into the instance's pose."""
    graph = GraphState.model_validate_json(_sub_slider_graph()
                                           .model_dump_json())
    model = build_model(graph)
    j = next(j for j in model.joints if j.child == "module_1__slider_1")
    assert j.jtype == "prismatic"
    np.testing.assert_allclose([round(x) for x in j.axis], [-1, 0, 0])
    assert abs(j.lower - (-0.05)) < 1e-9
    assert abs(j.upper - 0.15) < 1e-9


def test_use_limit_joints_off_ignores_subassembly_limit_mates():
    model = build_model(_sub_slider_graph(),
                        config={"use_limit_joints": False})
    assert not any(j.jtype == "prismatic" for j in model.joints)


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


def test_limit_distance_with_zero_minimum_travels_both_ways():
    """A limit distance bounds |d|: with min 0 the part can cross d = 0, so the
    travel from the assembled pose spans both sides, not just [0, max]."""
    from sw2robot.exporter.model import _limit_distance_travel

    # assembled on the plane (d = 0): symmetric
    assert _limit_distance_travel(0.0, 0.1, 0.0) == (-0.1, 0.1)
    # assembled 0.03 out: it can still go back through 0 to -max
    lo, hi = _limit_distance_travel(0.0, 0.1, 0.03)
    assert abs(lo - (-0.13)) < 1e-12 and abs(hi - 0.07) < 1e-12


def test_limit_distance_with_positive_minimum_stays_on_its_side():
    from sw2robot.exporter.model import _limit_distance_travel

    lo, hi = _limit_distance_travel(0.02, 0.1, 0.05)
    assert abs(lo - (-0.03)) < 1e-12 and abs(hi - 0.05) < 1e-12
