"""Regression test for the mirror-feature joint-axis fallback.

SolidWorks 'Mirror Components' positions the opposite-hand copies via the mirror
feature, so GetMates returns nothing for them -- the mirrored side (e.g. the
right arm) reaches _config_parent_map with no axis and would build as a dead
fixed link.  When the mirrored child part has exactly one sibling instance whose
joint DOES have an axis (the mated original), the axis -- a feature of that
part, identical in its own frame -- is reflected into the copy's pose:
``ax_R = W_child_R @ W_child_L^-1 @ ax_L``.
"""

import numpy as np

from sw2robot.exporter.model import Component, _config_parent_map


def _comp(name, world):
    return Component(name=name, link_name=name, part_path=None,
                     is_subassembly=False, world=np.asarray(world, float),
                     fixed=False, dof=0)


def _rot_z_180():
    m = np.eye(4)
    m[0, 0] = m[1, 1] = -1.0          # 180 deg about Z (a left<->right mirror)
    return m


def test_mirror_axis_is_reflected_from_the_mated_sibling():
    # left child sits at identity; the joint to it has axis +X (from real mates)
    left = _comp("link_L", np.eye(4))
    # right child is the same PART, placed by a 180-deg-about-Z mirror pose
    Wr = _rot_z_180()
    Wr[:3, 3] = [0.10, 0.20, 0.0]
    right = _comp("link_R", Wr)
    right.part_path = left.part_path = "arm_link.SLDPRT"   # same part = a pair
    parentL, parentR = _comp("parent_L", np.eye(4)), _comp("parent_R", np.eye(4))

    adjacency = {
        # the left joint carries a real concentric axis (+X through origin)
        frozenset(("link_L", "parent_L")):
            {"types": ["CONCENTRIC"],
             "axis": (np.zeros(3), np.array([1.0, 0.0, 0.0])), "mates": []},
        # the right joint has NO mates (mirror feature) -> no axis
        frozenset(("link_R", "parent_R")):
            {"types": [], "axis": None, "mates": []},
    }
    directed = [
        {"parent": "parent_L", "child": "link_L", "type": "revolute"},
        {"parent": "parent_R", "child": "link_R", "type": "revolute"},
    ]

    _parent_of, edge_info = _config_parent_map(
        [left, right, parentL, parentR], adjacency,
        _comp("base", np.eye(4)), directed)

    axR = edge_info[("link_R", "parent_R")]["axis"]
    assert axR is not None, "mirrored child got no axis (regression)"
    assert edge_info[("link_R", "parent_R")].get("mirrored_axis") is True
    # +X reflected by the 180-about-Z pose becomes -X
    np.testing.assert_allclose(np.asarray(axR[1], float), [-1.0, 0.0, 0.0],
                               atol=1e-9)


def test_no_mirror_when_sibling_is_ambiguous():
    """With two same-part siblings that both have an axis, the pairing is
    ambiguous and the fallback must NOT guess."""
    a = _comp("a", np.eye(4))
    b = _comp("b", np.eye(4))
    c = _comp("c", np.eye(4))              # the orphan
    for x in (a, b, c):
        x.part_path = "thing.SLDPRT"
    pa = _comp("pa", np.eye(4))
    adjacency = {
        frozenset(("a", "pa")): {"types": ["CONCENTRIC"],
                                 "axis": (np.zeros(3), np.array([1.0, 0, 0])),
                                 "mates": []},
        frozenset(("b", "pa")): {"types": ["CONCENTRIC"],
                                 "axis": (np.zeros(3), np.array([0, 1.0, 0])),
                                 "mates": []},
        frozenset(("c", "pa")): {"types": [], "axis": None, "mates": []},
    }
    directed = [
        {"parent": "pa", "child": "a", "type": "revolute"},
        {"parent": "pa", "child": "b", "type": "revolute"},
        {"parent": "pa", "child": "c", "type": "revolute"},
    ]
    _parent_of, edge_info = _config_parent_map(
        [a, b, c, pa], adjacency, _comp("base", np.eye(4)), directed)
    assert edge_info[("c", "pa")]["axis"] is None     # left ambiguous, untouched
