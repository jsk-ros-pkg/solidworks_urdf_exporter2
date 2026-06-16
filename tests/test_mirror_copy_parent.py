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
