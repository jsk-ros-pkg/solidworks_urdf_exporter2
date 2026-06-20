"""A concentric axial slide that the GLOBAL solve proves free is a prismatic.

classify_edge_geo welds a lone concentric-axis slide to fixed because, edge on
its own, a linear guide and a bolt's unmodelled face contact look identical (see
test_classify_geo.test_servo_mount_concentric_plus_perp_parallel_is_fixed).  The
assembly-wide twist solve CAN tell them apart: if the pair still slides freely
along that axis once every mate is accounted for, it is a real prismatic -- the
3D printer's Z gantry riding its rails.  These tests pin that promotion (and the
twin-rail carriage welding) so it cannot silently regress.
"""
import numpy as np

from sw2robot.exporter.model import build_model
from sw2robot.exporter.state import ComponentState, GraphState, MateEdge, MateGeo

CYL, PLANE = 4, 3


def _comp(name, xyz=(0, 0, 0), fixed=False, part_path=None):
    w = np.eye(4)
    w[:3, 3] = xyz
    return ComponentState(
        name=name, link_name=name.replace(" ", "_"), part_path=part_path,
        is_subassembly=False, world=[float(x) for x in w.flatten()],
        fixed=fixed)


def _geo(mtype, ents):
    """ents: list of (etype, point, dir)."""
    return MateGeo(type=mtype,
                   etypes=[e[0] for e in ents],
                   points=[list(map(float, e[1])) for e in ents],
                   dirs=[list(map(float, e[2])) for e in ents],
                   radii=[None] * len(ents))


def _conc_z(a, b, p):
    return MateEdge(a=a, b=b, types=["CONCENTRIC", "PARALLEL"],
                    axis_point=list(map(float, p)), axis_dir=[0.0, 0.0, 1.0],
                    mates=[_geo("CONCENTRIC", [(CYL, p, [0, 0, 1]),
                                              (CYL, p, [0, 0, 1])]),
                           _geo("PARALLEL", [(PLANE, p, [1, 0, 0]),
                                            (PLANE, p, [1, 0, 0])])])


def _graph(comps, edges, ground):
    return GraphState(robot_name="t", source_assembly="t.SLDASM",
                      components=comps, edges=edges, ground=ground)


def test_free_concentric_slide_becomes_prismatic():
    # frame (grounded) + carriage held ONLY by a concentric Z + perpendicular
    # parallel -> nothing pins Z -> the global solve promotes it to prismatic
    frame = _comp("frame", fixed=True)
    car = _comp("carriage", (0, 0, 0.1))
    g = _graph([frame, car], [_conc_z("frame", "carriage", [0, 0, 0])],
               ground=["frame"])
    model = build_model(g)
    j = next(j for j in model.joints if j.child == "carriage")
    assert j.jtype == "prismatic"
    assert abs(abs(np.dot(j.axis, [0, 0, 1])) - 1.0) < 1e-6


def test_locked_concentric_slide_stays_fixed():
    # same, but a COINCIDENT plane with a Z normal pins the axial position ->
    # the global solve finds no motion -> it must stay fixed (NOT promoted)
    frame = _comp("frame", fixed=True)
    car = _comp("carriage", (0, 0, 0.1))
    e = _conc_z("frame", "carriage", [0, 0, 0])
    e.mates.append(_geo("COINCIDENT", [(PLANE, [0, 0, 0], [0, 0, 1]),
                                       (PLANE, [0, 0, 0], [0, 0, 1])]))
    e.types.append("COINCIDENT")
    g = _graph([frame, car], [e], ground=["frame"])
    model = build_model(g)
    j = next(j for j in model.joints if j.child == "carriage")
    assert j.jtype == "fixed"


def test_twin_rails_collapse_to_one_carriage_one_dof():
    # two rail holders (SAME part) each concentric-Z to the frame and tied to
    # each other: ONE slide DOF, the other rail welded rigidly into the carriage
    frame = _comp("frame", fixed=True)
    railA = _comp("rail-1", (0.1, 0, 0.1), part_path="rail.SLDPRT")
    railB = _comp("rail-2", (-0.1, 0, 0.1), part_path="rail.SLDPRT")
    edges = [
        _conc_z("frame", "rail-1", [0.1, 0, 0]),
        _conc_z("frame", "rail-2", [-0.1, 0, 0]),
        MateEdge(a="rail-1", b="rail-2", types=["WIDTH", "WIDTH"],
                 axis_point=None, axis_dir=None,
                 mates=[_geo("WIDTH", [(PLANE, [0, 0, 0.1], [1, 0, 0]),
                                       (PLANE, [0, 0, 0.1], [1, 0, 0])])]),
    ]
    g = _graph([frame, railA, railB], edges, ground=["frame"])
    model = build_model(g)
    prism = [j for j in model.joints if j.jtype == "prismatic"]
    assert len(prism) == 1                      # one carriage, one slide DOF
    # the other rail is fixed into the carriage (rides the single slide)
    other = next(j for j in model.joints if j.child in ("rail-1", "rail-2")
                 and j.jtype != "prismatic")
    assert other.jtype == "fixed"
