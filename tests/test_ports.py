"""Add-port (dummy_link) editor support: the +Z->rpy helper shared with the
root-align endpoint, and the joints.yaml block-list helpers that append/remove
a ``ports:`` entry round-trip (the joints list survives, the empty block is
dropped)."""
import numpy as np
import yaml


def test_zdir_to_rpy_points_local_z_along_zdir():
    from sw2robot.editor import webserver as w
    from sw2robot.exporter.geometry import matrix_from_rpy
    for zdir in ([0, 0, 1], [1, 0, 0], [0, 1, 0], [0, 0, -1], [1, 1, 1]):
        rpy = w._zdir_to_rpy(zdir)
        got = matrix_from_rpy(rpy)[:3, :3] @ np.array([0.0, 0.0, 1.0])
        want = np.asarray(zdir, float)
        want = want / np.linalg.norm(want)
        assert np.allclose(got, want, atol=1e-6), (zdir, rpy, got)
    # the +Z-aligned case is the identity (no surprise yaw)
    assert np.allclose(w._zdir_to_rpy([0, 0, 1]), [0, 0, 0], atol=1e-9)


def test_zdir_to_rpy_near_antiparallel_is_stable():
    """A normal pointing (almost) along -Z must not blow up 1/(1+c)."""
    from sw2robot.editor import webserver as w
    from sw2robot.exporter.geometry import matrix_from_rpy
    for zdir in ([0, 0, -1], [1e-9, 0, -1], [0, -1e-8, -1]):
        rpy = w._zdir_to_rpy(zdir)
        got = matrix_from_rpy(rpy)[:3, :3] @ np.array([0.0, 0.0, 1.0])
        assert np.all(np.isfinite(got))
        assert got[2] < -0.999, (zdir, rpy, got)     # local +Z points down


def test_append_yaml_list_item_creates_and_extends_ports():
    from sw2robot.editor import webserver as w
    base = ("base: foo\njoints:\n  - parent: a\n    child: b\n    type: fixed\n")
    t1 = w._append_yaml_list_item(
        base, "ports",
        ["parent: b", "xyz: [0, 0, 0.01]", "rpy: [0, 0, 0]"])
    d1 = yaml.safe_load(t1)
    assert d1["ports"] == [{"parent": "b", "xyz": [0, 0, 0.01],
                            "rpy": [0, 0, 0]}]
    # the existing joints list / scalars are untouched
    assert d1["base"] == "foo" and len(d1["joints"]) == 1
    # a second add appends, it does not replace
    t2 = w._append_yaml_list_item(
        t1, "ports", ["parent: a", "xyz: [1, 2, 3]", "rpy: [0, 0, 0]"])
    d2 = yaml.safe_load(t2)
    assert len(d2["ports"]) == 2 and d2["ports"][1]["parent"] == "a"


def test_append_yaml_list_item_when_no_trailing_newline():
    from sw2robot.editor import webserver as w
    t = w._append_yaml_list_item("base: foo", "ports", ["parent: b"])
    assert yaml.safe_load(t) == {"base": "foo", "ports": [{"parent": "b"}]}


def test_remove_yaml_list_item_drops_one_and_clears_empty_block():
    from sw2robot.editor import webserver as w
    t = ("base: foo\nports:\n  - parent: a\n    xyz: [1, 2, 3]\n"
         "  - parent: b\n    xyz: [4, 5, 6]\njoints:\n  - parent: a\n"
         "    child: b\n")
    t1 = w._remove_yaml_list_item(t, "ports", 0)
    d1 = yaml.safe_load(t1)
    assert [p["parent"] for p in d1["ports"]] == ["b"]
    assert d1["joints"] and d1["base"] == "foo"     # neighbours intact
    # removing the last item drops the whole ports: block
    t2 = w._remove_yaml_list_item(t1, "ports", 0)
    d2 = yaml.safe_load(t2)
    assert "ports" not in d2 and d2["base"] == "foo" and d2["joints"]
    # out-of-range index is a no-op
    assert w._remove_yaml_list_item(t, "ports", 9) == t


def test_yaml_scalar_quotes_only_when_needed():
    from sw2robot.editor import webserver as w
    assert w._yaml_scalar("part_link-1") == "part_link-1"
    assert yaml.safe_load("k: " + w._yaml_scalar("Part 1/sub")) == {
        "k": "Part 1/sub"}
    assert yaml.safe_load("k: " + w._yaml_scalar("a: b")) == {"k": "a: b"}
