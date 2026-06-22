"""Unit tests for the per-link overlay (colour + inertial) added to the unified
edit model: ``LinkEdit`` / ``set_color`` / ``set_inertial`` and how
``build_urdf`` bakes them into the URDF.

Self-contained: a tiny hand-written URDF is loaded with ``core.load_module`` so
the tests need neither SolidWorks nor a graph.json cache.
"""

import xml.etree.ElementTree as ET

import pytest

import sw2robot.editor.core as c

MINI_URDF = """<?xml version="1.0"?>
<robot name="mini">
  <link name="base">
    <visual>
      <geometry><box size="1 1 1"/></geometry>
    </visual>
    <inertial>
      <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.1"/>
      <inertia ixx="0.0001" ixy="0" ixz="0" iyy="0.0001" iyz="0" izz="0.0001"/>
    </inertial>
  </link>
  <link name="tip">
    <visual>
      <geometry><box size="1 1 1"/></geometry>
    </visual>
  </link>
  <link name="ghost"/>
  <joint name="j" type="revolute">
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <parent link="base"/>
    <child link="tip"/>
    <axis xyz="0 0 1"/>
    <limit lower="-1" upper="1" effort="10" velocity="3"/>
  </joint>
</robot>
"""


@pytest.fixture
def state(tmp_path):
    p = tmp_path / "urdf" / "mini.urdf"
    p.parent.mkdir(parents=True)
    p.write_text(MINI_URDF, encoding="utf-8")
    return c.load_module(str(p))


def _link(urdf_str, name):
    return next(l for l in ET.fromstring(urdf_str).findall("link")
               if l.get("name") == name)


# --------------------------------------------------------------- colour
def test_set_color_bakes_material(state):
    c.set_color(state, "base", "#ff0000")
    mat = _link(c.build_urdf(state), "base").find("visual/material")
    assert mat is not None
    assert mat.find("color").get("rgba") == "1 0 0 1"


def test_set_color_accepts_bare_hex_and_normalizes(state):
    c.set_color(state, "base", "00FF00")     # no '#', upper case
    assert state.link_edits["base"].color == "#00ff00"
    rgba = _link(c.build_urdf(state), "base").find("visual/material/color").get("rgba")
    assert rgba == "0 1 0 1"


def test_set_color_none_clears(state):
    c.set_color(state, "base", "#ff0000")
    c.set_color(state, "base", None)
    assert state.link_edits["base"].color is None
    assert _link(c.build_urdf(state), "base").find("visual/material") is None


def test_set_color_invalid_raises(state):
    with pytest.raises(ValueError):
        c.set_color(state, "base", "#12")
    with pytest.raises(ValueError):
        c.set_color(state, "base", "nothex0")


def test_set_color_link_without_visual_is_noop(state):
    c.set_color(state, "ghost", "#ff0000")     # ghost has no <visual>
    assert _link(c.build_urdf(state), "ghost").find("visual") is None


# --------------------------------------------------------------- inertial
def test_set_inertial_full_on_link_without_inertial(state):
    c.set_inertial(state, "tip", mass=2.0, com=[1, 2, 3],
                   inertia=[3, 0.1, 0.2, 4, 0.3, 5])
    ine = _link(c.build_urdf(state), "tip").find("inertial")
    assert ine is not None
    assert float(ine.find("mass").get("value")) == 2.0
    assert ine.find("origin").get("xyz") == "1 2 3"
    I = ine.find("inertia")
    assert (I.get("ixx"), I.get("iyy"), I.get("izz")) == ("3", "4", "5")
    assert (I.get("ixy"), I.get("ixz"), I.get("iyz")) == ("0.1", "0.2", "0.3")


def test_set_inertial_rejects_nonphysical_tensor(state):
    """A right-length but physically impossible tensor must be rejected (the same
    physics check the exporter applies to generated inertials)."""
    with pytest.raises(ValueError):
        c.set_inertial(state, "base", inertia=[-1, 0, 0, 1, 0, 1])   # not pos-def
    with pytest.raises(ValueError):
        c.set_inertial(state, "base", inertia=[1, 0, 0, 1, 0, 10])   # I1+I2 < I3
    # a rejected tensor leaves no partial override behind
    assert state.link_edits.get("base") is None \
        or state.link_edits["base"].inertia is None


def test_set_inertial_partial_keeps_existing(state):
    # base already has mass 0.1 and inertia 1e-4 on the diagonal
    c.set_inertial(state, "base", mass=5.0)
    ine = _link(c.build_urdf(state), "base").find("inertial")
    assert float(ine.find("mass").get("value")) == 5.0
    # untouched fields preserved
    assert ine.find("inertia").get("ixx") == "0.0001"
    assert ine.find("origin").get("xyz") == "0 0 0"


def test_set_inertial_partial_on_link_without_inertial_is_valid_urdf(state):
    """A partial edit (mass only) on a link with NO base <inertial> must still
    emit a complete, valid block -- the missing <inertia> is backfilled."""
    c.set_inertial(state, "tip", mass=1.0)     # tip has no <inertial>
    ine = _link(c.build_urdf(state), "tip").find("inertial")
    assert float(ine.find("mass").get("value")) == 1.0
    tensor = ine.find("inertia")
    assert tensor is not None and tensor.get("ixx") == "0.0001"


def test_set_inertial_rejects_bad_shapes(state):
    with pytest.raises(ValueError):
        c.set_inertial(state, "base", mass=0)          # must be > 0
    with pytest.raises(ValueError):
        c.set_inertial(state, "base", com=[1, 2])      # need 3
    with pytest.raises(ValueError):
        c.set_inertial(state, "base", inertia=[1, 2, 3])   # need 6


def test_set_inertial_rejects_nonfinite(state):
    for bad in (float("nan"), float("inf")):
        with pytest.raises(ValueError):
            c.set_inertial(state, "base", mass=bad)
        with pytest.raises(ValueError):
            c.set_inertial(state, "base", com=[0, 0, bad])


# --------------------------------------------------------------- guards
def test_set_inertial_invalid_leaves_no_partial_state(state):
    """A rejected request must not have applied any of its fields."""
    with pytest.raises(ValueError):
        c.set_inertial(state, "base", mass=1.0, com=[1, 2])   # mass ok, com bad
    assert state.link_edits.get("base") is None \
        or state.link_edits["base"].mass is None


def test_unknown_link_raises(state):
    with pytest.raises(ValueError):
        c.set_color(state, "nope", "#ffffff")
    with pytest.raises(ValueError):
        c.set_inertial(state, "nope", mass=1.0)


def test_no_link_edits_leaves_urdf_structurally_unchanged(state):
    """build_urdf with an empty link overlay must not invent <material> or change
    inertials -- the existing joint-only behaviour is preserved."""
    out = c.build_urdf(state)
    assert _link(out, "base").find("visual/material") is None
    assert _link(out, "tip").find("inertial") is None
    assert _link(out, "ghost").find("inertial") is None


def test_link_edits_survive_json_roundtrip(state, tmp_path):
    c.set_color(state, "base", "#abcdef")
    c.set_inertial(state, "tip", mass=1.5, com=[0, 0, 0.1])
    path = c.save_state(state, tmp_path / "s.json")
    reloaded = c.load_state(path)
    assert reloaded.link_edits["base"].color == "#abcdef"
    assert reloaded.link_edits["tip"].mass == 1.5
    assert reloaded.link_edits["tip"].com == [0, 0, 0.1]


def test_load_edits_merges_link_overlay(state, tmp_path):
    """A saved sidecar's link_edits must be re-applied on the refresh path
    (load_edits), not just the joint edits -- else colours/inertials vanish on
    every rebuild."""
    c.set_color(state, "base", "#abcdef")
    c.set_inertial(state, "tip", mass=2.0)
    c.set_limits(state, "j", -0.5, 0.5)
    sidecar = c.save_state(state, tmp_path / "side.json")

    # a fresh state for the same module (as if just rebuilt) starts edit-free
    fresh = c.load_module(state.urdf_path)
    assert fresh.link_edits == {} and fresh.edits == {}
    n = c.load_edits(fresh, sidecar)
    assert n == 3                                  # 1 joint + 2 link edits
    assert fresh.link_edits["base"].color == "#abcdef"
    assert fresh.link_edits["tip"].mass == 2.0
    assert fresh.edits["j"].lower == -0.5
