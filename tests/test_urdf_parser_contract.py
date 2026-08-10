"""The contract ``parse_urdf_content`` owes the editor.

``core.load_module`` and ``webserver._parse_urdf`` both go through it, so the
key names, the defaults for everything a URDF may leave out, and the root-link
choice on a partial document are all load-bearing for link/joint listing, the
edit overlay and the rename routing.  The parser's docstring explains why this
is hand-rolled rather than ``skrobot.utils.urdf.URDF``; the tolerance cases
below are the inputs that loader rejects, and are what makes the difference
observable rather than a matter of taste.
"""

import pytest

from sw2robot.editor._vendor.rc_config.urdf_parser import parse_urdf_content

FULL = """<?xml version="1.0"?>
<robot name="full">
  <link name="base">
    <visual><geometry><box size="1 1 1"/></geometry></visual>
    <collision><geometry><box size="1 1 1"/></geometry></collision>
    <inertial>
      <mass value="0.1"/>
      <inertia ixx="1e-4" ixy="0" ixz="0" iyy="1e-4" iyz="0" izz="1e-4"/>
    </inertial>
  </link>
  <link name="tip"/>
  <joint name="j" type="revolute">
    <parent link="base"/>
    <child link="tip"/>
    <axis xyz="0 0 2"/>
    <limit lower="-1.5" upper="1.5" effort="10" velocity="3"/>
  </joint>
</robot>
"""


def test_keys_and_values():
    parsed = parse_urdf_content(FULL)

    assert set(parsed) == {"joints", "links", "root_link"}
    assert parsed["root_link"] == "base"
    assert parsed["joints"] == [{
        "name": "j", "type": "revolute",
        "parentLink": "base", "childLink": "tip",
        # kept as written: the axis is not renormalized to a unit vector
        "axis": [0.0, 0.0, 2.0],
        "lowerLimit": -1.5, "upperLimit": 1.5,
        "velocityLimit": 3.0, "effortLimit": 10.0,
    }]
    assert parsed["links"] == [
        {"name": "base", "hasVisual": True, "hasCollision": True,
         "hasInertial": True},
        {"name": "tip", "hasVisual": False, "hasCollision": False,
         "hasInertial": False},
    ]


def test_absent_optionals_get_defaults():
    """A continuous joint carries no ``<limit>`` and often no ``<axis>``; both
    come back as numbers, since the limit fields feed arithmetic
    (``core.reverse_direction``) that None would break."""
    parsed = parse_urdf_content("""<robot name="c">
      <link name="base"/><link name="wheel"/>
      <joint name="spin" type="continuous">
        <parent link="base"/><child link="wheel"/>
      </joint>
    </robot>""")

    j = parsed["joints"][0]
    assert j["axis"] == [0.0, 0.0, 1.0]
    assert (j["lowerLimit"], j["upperLimit"]) == (0.0, 0.0)
    assert (j["velocityLimit"], j["effortLimit"]) == (0.0, 0.0)


def test_document_order_is_preserved():
    parsed = parse_urdf_content("""<robot name="o">
      <link name="c"/><link name="a"/><link name="b"/>
      <joint name="j2" type="fixed"><parent link="c"/><child link="a"/></joint>
      <joint name="j1" type="fixed"><parent link="a"/><child link="b"/></joint>
    </robot>""")

    assert [ln["name"] for ln in parsed["links"]] == ["c", "a", "b"]
    assert [j["name"] for j in parsed["joints"]] == ["j2", "j1"]


def test_root_is_the_link_no_joint_claims():
    parsed = parse_urdf_content("""<robot name="r">
      <link name="tip"/><link name="base"/>
      <joint name="j" type="fixed"><parent link="base"/><child link="tip"/></joint>
    </robot>""")

    assert parsed["root_link"] == "base"


def test_multi_root_prefers_the_first_link_that_drives_a_joint():
    """A disconnected/imported URDF has several rootless links.  The pick has to
    be a real kinematic root, and stable across processes -- the editor's rename
    routing sends the root link to ``root_link_name:`` and every other link to
    ``link_names:``, so a root that moved between two calls would corrupt the
    config."""
    parsed = parse_urdf_content("""<robot name="m">
      <link name="stray"/>
      <link name="real_root"/><link name="kid"/>
      <link name="other_root"/><link name="other_kid"/>
      <joint name="j" type="fixed">
        <parent link="real_root"/><child link="kid"/></joint>
      <joint name="k" type="fixed">
        <parent link="other_root"/><child link="other_kid"/></joint>
    </robot>""")

    assert parsed["root_link"] == "real_root"


def test_root_falls_back_to_the_first_candidate():
    parsed = parse_urdf_content(
        '<robot name="s"><link name="lonely"/><link name="also"/></robot>')

    assert parsed["root_link"] == "lonely"


def test_every_link_is_a_child_leaves_no_root():
    parsed = parse_urdf_content("""<robot name="cy">
      <link name="a"/><link name="b"/>
      <joint name="j1" type="fixed"><parent link="a"/><child link="b"/></joint>
      <joint name="j2" type="fixed"><parent link="b"/><child link="a"/></joint>
    </robot>""")

    assert parsed["root_link"] is None


@pytest.mark.parametrize("urdf, expected", [
    # an <inertial/> with no <mass>, as a URDF written before a mass pass
    ('<robot name="x"><link name="a"><inertial/></link></robot>',
     {"name": "a", "hasVisual": False, "hasCollision": False,
      "hasInertial": True}),
    ('<robot name="x"><link/></robot>',
     {"name": "unnamed_link", "hasVisual": False, "hasCollision": False,
      "hasInertial": False}),
])
def test_tolerates_partial_links(urdf, expected):
    assert parse_urdf_content(urdf)["links"] == [expected]


@pytest.mark.parametrize("joint_xml, name, jtype, parent, child", [
    ('<joint name="j"><parent link="a"/><child link="b"/></joint>',
     "j", "fixed", "a", "b"),
    ('<joint name="j" type="wobbly"><parent link="a"/><child link="b"/></joint>',
     "j", "wobbly", "a", "b"),
    ('<joint name="j" type="fixed"><child link="b"/></joint>',
     "j", "fixed", "", "b"),
    ('<joint type="fixed"><parent link="a"/><child link="b"/></joint>',
     "unnamed_joint", "fixed", "a", "b"),
])
def test_tolerates_partial_joints(joint_xml, name, jtype, parent, child):
    """A user's URDF, or one mid-edit, may leave any of these out; the editor
    still has to list the joint so it can be fixed."""
    j = parse_urdf_content(
        f'<robot name="x"><link name="a"/><link name="b"/>{joint_xml}</robot>'
    )["joints"][0]

    assert (j["name"], j["type"]) == (name, jtype)
    assert (j["parentLink"], j["childLink"]) == (parent, child)


@pytest.mark.parametrize("bad", [
    '<robot name="t"><link name="a"/>',            # truncated
    '<sdf><model name="m"/></sdf>',                # not a URDF
])
def test_unparseable_input_raises(bad):
    """It must raise, not return an empty robot: ``webserver._parse_urdf``
    turns the exception into None and keeps the previous view, whereas a robot
    with zero links reads as a URDF that genuinely lost its contents."""
    with pytest.raises(ValueError):
        parse_urdf_content(bad)
