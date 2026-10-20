"""Regression test for the mate-less mirror/copy parent rule.

SolidWorks 'Mirror Components' and pattern copies carry NO mates, so a mirrored
part (e.g. the right gripper finger) reaches the tree builder with nothing to
connect to.  The plain nearest-component fallback then bolts it to whatever
sits closest -- which for a second finger copy is the FIRST finger copy, not
the gripper base, producing a bogus finger-under-finger chain.

The fix: a mate-less part whose twin (another instance of the same part) IS
mated reuses that twin's connection -- it attaches to the nearest already-placed
instance of the twin's parent PART, i.e. it connects exactly like its mirror
original.  A safety guard only fires the rule when the world-nearest instance of
that parent part is already placed, so it can never cross sides (a right finger
onto a left base).
"""

import numpy as np

from sw2robot.exporter.model import Component, _auto_parent_map


def _c(name, part, xyz):
    w = np.eye(4)
    w[:3, 3] = xyz
    return Component(name=name, link_name=name, part_path=part,
                     is_subassembly=False, world=w, fixed=False, dof=0)


def test_mirrored_finger_attaches_to_its_base_not_the_sibling_finger():
    base = _c("base", "root.SLDASM", (0, 0, 0))
    # both gripper bases are the SAME part, mated to the root on each side
    base_l = _c("base_l", "gbase.SLDASM", (-1.0, 0, 0))
    base_r = _c("base_r", "gbase.SLDASM", (1.0, 0, 0))
    # left finger is mated to the left base (the original, mate-bearing side)
    finger_l = _c("finger_l", "finger.SLDASM", (-1.0, 0, 0.10))
    # right fingers are UN-mated mirror copies of the same finger part; the
    # second copy sits closest to the FIRST copy (the bug trigger)
    finger_r1 = _c("finger_r1", "finger.SLDASM", (1.0, 0, 0.10))
    finger_r2 = _c("finger_r2", "finger.SLDASM", (1.0, 0, 0.15))

    adjacency = {
        frozenset(("base", "base_l")):
            {"types": ["COINCIDENT"], "axis": None, "mates": []},
        frozenset(("base", "base_r")):
            {"types": ["COINCIDENT"], "axis": None, "mates": []},
        frozenset(("base_l", "finger_l")):
            {"types": ["CONCENTRIC"],
             "axis": (np.zeros(3), np.array([0.0, 0.0, 1.0])), "mates": []},
        # finger_r1 / finger_r2 deliberately have NO edges (mirror copies)
    }
    comps = [base, base_l, base_r, finger_l, finger_r1, finger_r2]
    parent_of, _edge = _auto_parent_map(comps, adjacency, base)

    # left finger keeps its real mated parent
    assert parent_of["finger_l"] == "base_l"
    # BOTH right fingers hang off the right base -- independent, like the left
    assert parent_of["finger_r1"] == "base_r"
    assert parent_of["finger_r2"] == "base_r", \
        "second mirror finger bogusly parented under the first finger"


def test_mirror_copy_inherits_the_twins_revolute_and_axis():
    """A mate-less mirror copy must not just get the right parent -- it must
    inherit the JOINT itself: the twin's revolute type and a reflected axis,
    instead of welding as a dead fixed link with no axis."""
    base = _c("base", "root.SLDASM", (0, 0, 0))
    base_l = _c("base_l", "gbase.SLDASM", (-1.0, 0, 0))
    base_r = _c("base_r", "gbase.SLDASM", (1.0, 0, 0))
    finger_l = _c("finger_l", "finger.SLDASM", (-1.0, 0, 0.10))
    finger_r = _c("finger_r", "finger.SLDASM", (1.0, 0, 0.10))

    adjacency = {
        frozenset(("base", "base_l")):
            {"types": ["COINCIDENT"], "axis": None, "mates": []},
        frozenset(("base", "base_r")):
            {"types": ["COINCIDENT"], "axis": None, "mates": []},
        # the left finger is a real CONCENTRIC revolute about +Z
        frozenset(("base_l", "finger_l")):
            {"types": ["CONCENTRIC"],
             "axis": (np.zeros(3), np.array([0.0, 0.0, 1.0])), "mates": []},
        # finger_r is a mate-less copy (no edge)
    }
    comps = [base, base_l, base_r, finger_l, finger_r]
    _parent_of, edge_info = _auto_parent_map(comps, adjacency, base)

    left = edge_info[("finger_l", "base_l")]
    right = edge_info[("finger_r", "base_r")]
    assert left["type"] == "revolute"               # the mated original
    assert right["type"] == "revolute", \
        "mirror copy welded as a fixed link instead of inheriting the revolute"
    assert right["axis"] is not None
    assert right.get("mirrored_axis") is True
    # +Z reflected through a pure translation stays parallel to +Z
    assert abs(abs(float(np.asarray(right["axis"][1]) @ [0, 0, 1])) - 1.0) < 1e-6


def test_mirror_chain_cascades_in_tree_order_not_to_nearest():
    """A whole mate-less limb (shoulder->elbow) must rebuild in tree order: the
    elbow copy attaches to the shoulder copy, even though it sits closest to the
    base.  This needs the shoulder copy placed first so the elbow can mirror
    onto it, instead of the elbow grabbing the nearest placed component."""
    base = _c("base", "root.SLDASM", (0, 0, 0))
    shoulder_l = _c("shoulder_l", "shoulder.SLDASM", (-1.0, 0, 0))
    elbow_l = _c("elbow_l", "elbow.SLDASM", (-1.0, 0, 0.10))
    # right copies (no mates); the elbow copy sits RIGHT NEXT TO the base, so a
    # nearest-component fallback would wrongly weld it to the base
    shoulder_r = _c("shoulder_r", "shoulder.SLDASM", (1.0, 0, 0))
    elbow_r = _c("elbow_r", "elbow.SLDASM", (0.1, 0, 0.05))

    adjacency = {
        frozenset(("base", "shoulder_l")):
            {"types": ["COINCIDENT"], "axis": None, "mates": []},
        frozenset(("shoulder_l", "elbow_l")):
            {"types": ["CONCENTRIC"],
             "axis": (np.zeros(3), np.array([0.0, 1.0, 0.0])), "mates": []},
    }
    comps = [base, shoulder_l, elbow_l, shoulder_r, elbow_r]
    parent_of, edge_info = _auto_parent_map(comps, adjacency, base)

    assert parent_of["shoulder_r"] == "base"
    # the elbow copy rides the shoulder copy, NOT the nearby base
    assert parent_of["elbow_r"] == "shoulder_r", \
        "elbow copy welded to nearest component instead of cascading"
    # and it inherits the elbow's revolute, not a dead fixed link
    assert edge_info[("elbow_r", "shoulder_r")]["type"] == "revolute"


def test_rule_defers_instead_of_crossing_sides_when_same_side_base_unplaced():
    """The right base EXISTS but, being farther from the root, is reached AFTER
    the right finger.  The finger's world-nearest base instance is the (still
    unplaced) right base, so the twin rule must DEFER -- it must not grab the
    already-placed LEFT base and cross sides.  Deferral hands the finger to the
    plain nearest-component fallback (here the root), never the left base."""
    base = _c("base", "root.SLDASM", (0, 0, 0))
    base_l = _c("base_l", "gbase.SLDASM", (-1.0, 0, 0))
    finger_l = _c("finger_l", "finger.SLDASM", (-1.0, 0, 0.10))
    # right side sits far out on +X; the finger (dist 4) is reached before the
    # right base (dist 5), so the right base is NOT yet placed for the finger
    finger_r = _c("finger_r", "finger.SLDASM", (4.0, 0, 0))
    base_r = _c("base_r", "gbase.SLDASM", (5.0, 0, 0))

    adjacency = {
        frozenset(("base", "base_l")):
            {"types": ["COINCIDENT"], "axis": None, "mates": []},
        frozenset(("base_l", "finger_l")):
            {"types": ["CONCENTRIC"],
             "axis": (np.zeros(3), np.array([0.0, 0.0, 1.0])), "mates": []},
        # finger_r and base_r are mate-less mirror copies (no edges)
    }
    comps = [base, base_l, finger_l, finger_r, base_r]
    parent_of, _edge = _auto_parent_map(comps, adjacency, base)

    # the right finger must NOT be parented to the wrong-side (left) base
    assert parent_of["finger_r"] != "base_l", "twin rule crossed sides"


def test_unmated_island_welds_to_base_not_the_nearest_moving_link():
    """A part with no mates and no mated twin has nothing saying it moves with
    anything, so it must weld FIXED to the base -- not to the component that
    happens to sit nearest, which here is a hinged arm it would then ride."""
    base = _c("base", "frame.SLDPRT", (0, 0, 0))
    arm = _c("arm", "arm.SLDPRT", (0.0, 0.2, 0.1))
    island = _c("island", "plate.SLDPRT", (0.0, 0.19, 0.0))
    adjacency = {
        frozenset(("base", "arm")):
            {"types": ["CONCENTRIC"],
             "axis": (np.zeros(3), np.array([0.0, 0.0, 1.0])), "mates": []},
    }
    comps = [base, arm, island]
    parent_of, edge_info = _auto_parent_map(comps, adjacency, base)

    assert edge_info[("arm", "base")]["type"] == "revolute"
    assert parent_of["island"] == "base", "unmated part rides the nearest link"
    assert edge_info[("island", "base")]["type"] == "fixed"
