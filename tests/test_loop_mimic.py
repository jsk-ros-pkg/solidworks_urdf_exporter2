"""A closed-loop (four-bar) linkage is exported as driver + mimic followers.

A SolidWorks closed linkage is fully constrained -- four revolute hinges about
parallel axes around one loop, so only ONE joint is really free.  The spanning
tree keeps three hinges as revolute and drops the fourth as a loop closure;
without coupling the three move independently and the linkage flies apart.
``_auto_loop_mimic`` detects the four-bar and makes the two passive tree hinges
``<mimic>`` the driver.  For a parallelogram the coupling is exactly +-1, which
these tests pin so the detection (and multiplier maths) cannot silently regress.
"""
import numpy as np

from sw2robot.exporter.model import build_model
from sw2robot.exporter.state import ComponentState, GraphState, MateEdge, MateGeo

CYL, PLANE = 4, 3


def _comp(name, xyz=(0, 0, 0), fixed=False):
    w = np.eye(4)
    w[:3, 3] = xyz
    return ComponentState(
        name=name, link_name=name.replace(" ", "_"), part_path=None,
        is_subassembly=False, world=[float(x) for x in w.flatten()],
        fixed=fixed)


def _geo(mtype, ents):
    return MateGeo(type=mtype,
                   etypes=[e[0] for e in ents],
                   points=[list(map(float, e[1])) for e in ents],
                   dirs=[list(map(float, e[2])) for e in ents],
                   radii=[None] * len(ents))


def _hinge_z(a, b, p):
    """A pure revolute about world +Z at point ``p``: a concentric Z cylinder
    (rotation + axial slide) pinned axially by a coincident Z-normal plane."""
    p = list(map(float, p))
    return MateEdge(a=a, b=b, types=["CONCENTRIC", "COINCIDENT"],
                    axis_point=p, axis_dir=[0.0, 0.0, 1.0],
                    mates=[_geo("CONCENTRIC", [(CYL, p, [0, 0, 1]),
                                               (CYL, p, [0, 0, 1])]),
                           _geo("COINCIDENT", [(PLANE, p, [0, 0, 1]),
                                               (PLANE, p, [0, 0, 1])])])


def _graph(comps, edges, ground):
    return GraphState(robot_name="t", source_assembly="t.SLDASM",
                      components=comps, edges=edges, ground=ground)


def _parallelogram():
    # ground bar A(0,0)->D(2,0); equal input A->B(0,1) and output D->C(2,1);
    # coupler B->C has the same length as the ground bar -> a parallelogram.
    base = _comp("base", fixed=True)
    inl = _comp("inlink", (0, 1, 0))
    cpl = _comp("coupler", (1, 1, 0))
    outl = _comp("outlink", (2, 1, 0))
    edges = [
        _hinge_z("base", "inlink", [0, 0, 0]),       # A
        _hinge_z("inlink", "coupler", [0, 1, 0]),    # B
        _hinge_z("coupler", "outlink", [2, 1, 0]),   # C
        _hinge_z("outlink", "base", [2, 0, 0]),      # D
    ]
    return _graph([base, inl, cpl, outl], edges, ground=["base"])


def test_four_bar_loop_becomes_driver_plus_two_mimics():
    model = build_model(_parallelogram())
    rev = [j for j in model.joints if j.jtype == "revolute"]
    # 4 physical hinges, one dropped as the loop closure -> 3 revolute tree edges
    assert len(rev) == 3
    mimics = [j for j in rev if j.mimic]
    free = [j for j in rev if not j.mimic]
    assert len(mimics) == 2 and len(free) == 1      # one driver, two followers
    driver = free[0]
    # both followers mimic the SAME (driver) joint, offset 0
    assert all(j.mimic["joint"] == driver.name for j in mimics)
    assert all(j.mimic["offset"] == 0.0 for j in mimics)
    # a parallelogram couples exactly +-1
    for j in mimics:
        assert abs(abs(j.mimic["multiplier"]) - 1.0) < 1e-3


def test_loop_mimic_round_trips_through_joints_yaml(tmp_path):
    # The editor rebuilds via build(--config joints.yaml), which takes the
    # DIRECTED branch and does NOT re-run auto loop detection -- so the mimic
    # must be persisted in the template and read back, or it (and the viewer's
    # purple axis) vanishes on the next build.
    import yaml

    from sw2robot.exporter import jointcfg
    from sw2robot.exporter.model import build_model

    graph = _parallelogram()
    model = build_model(graph)
    follower = next(j for j in model.joints if j.mimic)

    tmpl = tmp_path / "j.yaml"
    jointcfg.write_template(model, str(tmpl))
    cfg = yaml.safe_load(tmpl.read_text(encoding="utf-8"))
    entry = next(j for j in cfg["joints"]
                 if j.get("mimic", {}).get("joint") == follower.mimic["joint"])
    assert entry["mimic"]["joint"] == follower.mimic["joint"]
    assert "poly" in entry["mimic"] and len(entry["mimic"]["poly"]) >= 2

    # feeding that config back keeps the <mimic> (directed path, no auto detect)
    model2 = build_model(graph, config=cfg)
    f2 = next((j for j in model2.joints
               if j.name == follower.name), None)
    assert f2 is not None and f2.mimic is not None
    assert f2.mimic["joint"] == follower.mimic["joint"]


def test_four_bar_driver_carries_no_mimic_and_loop_is_open():
    model = build_model(_parallelogram())
    # exactly one revolute hinge is dropped (the URDF tree stays acyclic): 4
    # links, and link count - 1 == joint count for a tree
    assert len(model.joints) == len(model.components) - 1
    # the driver is a real, independent joint (no <mimic> on it)
    rev = [j for j in model.joints if j.jtype == "revolute"]
    driver = next(j for j in rev if not j.mimic)
    assert driver.mimic is None
