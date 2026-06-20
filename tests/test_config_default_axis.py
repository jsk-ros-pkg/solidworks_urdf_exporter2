"""A config-requested movable joint with NO derivable axis gets a default axis.

Flipping a joint to prismatic/revolute in the editor used to silently snap back
to fixed when the edge had no CAD axis to derive (a fully DISTANCE-constrained
pair, or a STEP body with no mates).  It must instead default to world +Z so the
joint is real and drivable, with axis_dir to point it.
"""
import numpy as np

from sw2robot.exporter.model import Component, _config_parent_map


def _comp(name, xyz=(0, 0, 0)):
    w = np.eye(4)
    w[:3, 3] = xyz
    return Component(name=name, link_name=name, part_path=None,
                     is_subassembly=False, world=w, fixed=False, dof=0)


def _build(directed, adjacency=None):
    base = _comp("base")
    body = _comp("body")
    tool = _comp("tool", (0.1, 0.2, 0.3))
    return _config_parent_map([base, body, tool], adjacency or {}, base,
                              directed)


def test_axisless_prismatic_defaults_to_world_z():
    _pa, edge = _build([{"parent": "body", "child": "tool", "type": "prismatic"}])
    ax = edge[("tool", "body")]["axis"]
    assert edge[("tool", "body")]["type"] == "prismatic"
    assert ax is not None                                  # NOT degraded to fixed
    np.testing.assert_allclose(ax[1], [0, 0, 1])           # default world +Z
    np.testing.assert_allclose(ax[0], [0.1, 0.2, 0.3])     # through child origin


def test_axisless_revolute_defaults_too():
    _pa, edge = _build([{"parent": "body", "child": "tool", "type": "revolute"}])
    assert edge[("tool", "body")]["axis"] is not None


def test_explicit_axis_dir_wins():
    _pa, edge = _build([{"parent": "body", "child": "tool",
                         "type": "prismatic", "axis_dir": [0, 1, 0]}])
    np.testing.assert_allclose(edge[("tool", "body")]["axis"][1], [0, 1, 0])


def test_fixed_stays_axisless():
    _pa, edge = _build([{"parent": "body", "child": "tool", "type": "fixed"}])
    assert edge[("tool", "body")]["axis"] is None
