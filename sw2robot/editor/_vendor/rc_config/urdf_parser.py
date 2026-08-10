"""
URDF Parser for configuration page.

Extracts joint and link information from URDF content.

Deliberately NOT ``skrobot.utils.urdf.URDF``, which the editor does use
wherever it needs kinematics.  That loader validates as much as it parses, and
the documents reaching this function are the ones that fail validation: the
editor opens whatever URDF a user points it at, and its own in-progress URDFs
are routinely partial (see ``webserver._urdf_link_world_poses``).  Concretely
``URDF.load`` raises on an empty ``<inertial/>``, on a joint missing
``type``/``<parent>``/``<child>``, on a link with no ``name`` and on any joint
type outside its fixed list; on an XML syntax error it returns
``URDF(name='INVALID_URDF', links=[])``, so a broken file would reach the UI
as an empty robot instead of the ValueError callers already handle; and its
``base_link`` is None for every multi-root or disconnected document, which is
precisely the case the tie-break below exists to decide (a None root sends the
webserver's root-link rename down the ``link_names:`` branch instead of
``root_link_name:``).  It also normalises the axis and drops absent limits to
None, neither of which round-trips back to what this returns.

``skrobot.urdf.structure`` holds a tolerant ElementTree equivalent of the graph
walk below, with the same document-order base-link semantics, but it is
private, and carries no effort limit and no per-link geometry flags.
"""

import xml.etree.ElementTree as ET
from typing import Any


def parse_urdf_content(urdf_content: str) -> dict[str, Any]:
    """
    Parse URDF content and extract joint/link information.

    Parameters
    ----------
    urdf_content : str
        The URDF XML content as a string.

    Returns
    -------
    dict
        Dictionary containing:
        - joints: List of joint information
        - links: List of link information
        - root_link: Name of the root link

    Raises
    ------
    ValueError
        If the URDF content is invalid.
    """
    try:
        root = ET.fromstring(urdf_content)
    except ET.ParseError as e:
        raise ValueError(f"Invalid URDF XML: {e}") from e

    if root.tag != "robot":
        raise ValueError(f"Expected <robot> root element, got <{root.tag}>")

    # Extract joints
    joints = []
    for joint_elem in root.findall("joint"):
        joint_info = _parse_joint(joint_elem)
        joints.append(joint_info)

    # Extract links
    links = []
    for link_elem in root.findall("link"):
        link_info = _parse_link(link_elem)
        links.append(link_info)

    # Find root link (link that is not a child of any joint).  Keep DOCUMENT
    # ORDER throughout -- a set.pop() here is non-deterministic across runs,
    # which would make root detection (and the editor's rename/root handling
    # built on it) flip arbitrarily on a multi-root / disconnected URDF.
    child_links = {j["childLink"] for j in joints}
    root_candidates = [link["name"] for link in links
                       if link["name"] not in child_links]

    root_link = None
    if len(root_candidates) == 1:
        root_link = root_candidates[0]
    elif len(root_candidates) > 1:
        # multiple roots (disconnected / imported URDF): prefer the first
        # candidate that actually drives a joint (a real kinematic root) over a
        # stray link with no joints, else just the first -- both in document
        # order so the choice is stable and identical for every caller
        # (core.load_module and the webserver's root detection).
        parent_links = {j["parentLink"] for j in joints}
        root_link = next((c for c in root_candidates if c in parent_links),
                         root_candidates[0])

    return {
        "joints": joints,
        "links": links,
        "root_link": root_link,
    }


def _parse_joint(joint_elem: ET.Element) -> dict[str, Any]:
    """Parse a single joint element."""
    joint_name = joint_elem.get("name", "unnamed_joint")
    joint_type = joint_elem.get("type", "fixed")

    # Parent and child links
    parent_elem = joint_elem.find("parent")
    child_elem = joint_elem.find("child")
    parent_link = parent_elem.get("link", "") if parent_elem is not None else ""
    child_link = child_elem.get("link", "") if child_elem is not None else ""

    # Axis
    axis_elem = joint_elem.find("axis")
    if axis_elem is not None:
        xyz = axis_elem.get("xyz", "0 0 1")
        axis = [float(x) for x in xyz.split()]
    else:
        axis = [0.0, 0.0, 1.0]

    # Limits
    limit_elem = joint_elem.find("limit")
    lower_limit = 0.0
    upper_limit = 0.0
    velocity_limit = 0.0
    effort_limit = 0.0

    if limit_elem is not None:
        lower_limit = float(limit_elem.get("lower", "0"))
        upper_limit = float(limit_elem.get("upper", "0"))
        velocity_limit = float(limit_elem.get("velocity", "0"))
        effort_limit = float(limit_elem.get("effort", "0"))

    return {
        "name": joint_name,
        "type": joint_type,
        "parentLink": parent_link,
        "childLink": child_link,
        "axis": axis,
        "lowerLimit": lower_limit,
        "upperLimit": upper_limit,
        "velocityLimit": velocity_limit,
        "effortLimit": effort_limit,
    }


def _parse_link(link_elem: ET.Element) -> dict[str, Any]:
    """Parse a single link element."""
    link_name = link_elem.get("name", "unnamed_link")

    has_visual = link_elem.find("visual") is not None
    has_collision = link_elem.find("collision") is not None
    has_inertial = link_elem.find("inertial") is not None

    return {
        "name": link_name,
        "hasVisual": has_visual,
        "hasCollision": has_collision,
        "hasInertial": has_inertial,
    }
