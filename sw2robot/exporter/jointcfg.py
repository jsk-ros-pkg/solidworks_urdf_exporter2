"""Joint configuration: a YAML the user edits to pin down the real kinematics.

Auto mate-based detection can only guess (a concentric mate might be a hinge, a
bolt circle, a bearing or a servo output -- indistinguishable from mates alone).
So ``export`` writes a template listing every mate-connected pair with its mate
types and a suggested joint type; edit it and re-run with ``--config``.

The ``joints`` list IS the kinematic tree: each entry's ``parent``/``child``
defines an edge (child moves relative to parent).  Re-wire ``parent``/``child``
to fix the chain (e.g. make the arm a child of the servo horn so it follows the
horn), and set ``type`` per edge.  Components not listed are attached to the
base with a fixed joint.

Format::

    base: KHFS5                       # base/root link (name or substring)
    joints:
      - parent: KRS3300_scale_12
        child:  small_diameter_hornB_2
        type:   revolute              # fixed | revolute | prismatic | continuous
        # lower: -1.57                 # optional limits (rad/m)
        # upper:  1.57
        # axis_point: [x, y, z]        # optional: override world axis point
        # axis_dir:   [x, y, z]        # optional: override world axis direction

Rename links / joints in the emitted URDF (the web editor writes these when you
rename in the viewer).  Keyed by the COMPONENT link name / the ``parent__child``
joint name; the value is the display name (sanitised on write)::

    link_names:
      fingertip_back_1: distal          # <link name="distal"> + refs follow
    joint_names:
      fingertip_front_2__fingertip_back_1: distal_joint

Module interface (robot-compiler / NejiNeji).  The emitted URDF renames the root
link to ``base_link`` (= input port / ``from_coords``); override or disable::

    root_link_name: base_link          # set falsy ('') to keep the part's name

Declare output ports (``to_coords``) -- each becomes an empty ``dummy_link`` on a
fixed joint that robot-compiler picks up as a connectable port::

    ports:
      - parent: linkB_2                # tip link the connector hangs off
        xyz: [0, 0, 0.05]              # dummy_link origin in that link's frame
        rpy: [0, 0, 0]                 # orient so +Z = outgoing connector axis
        # name: dummy_link             # optional (auto: dummy_link, dummy_link2..)
"""

from __future__ import annotations

import yaml


def write_template(model, path):
    lines = [
        "# Joint config -- this IS the kinematic tree.  Edit parent/child to",
        "# re-wire the chain and 'type' per edge, then re-run:",
        "#   uv run python -m sw2robot.exporter.export <asm> --config this_file.yaml",
        "# type: fixed | revolute | prismatic | continuous",
        "# Sub-assemblies with moving internals expand automatically;",
        "# override with  expand: [name-substr]  /  no_expand: [name-substr]",
        f"base: {model.base_link}",
        "joints:",
    ]
    for j in model.joints:
        hint = (" # mates: " + ", ".join(j.mate_types)) if j.mate_types else ""
        if getattr(j, "geo_note", None):
            hint += (" | " if hint else " # ") + j.geo_note
        lines.append(f"  - parent: {j.parent}")
        lines.append(f"    child:  {j.child}")
        lines.append(f"    type:   {j.jtype}{hint}")
        # round-trip the travel range so re-running --config keeps it -- a joint
        # WITHOUT a SolidWorks limit mate (a promoted concentric slide) has no
        # other home for its limits, so omitting them here silently reset it to
        # the default on the next build.  continuous joints have no endpoints.
        if j.jtype in ("revolute", "prismatic") \
                and j.lower is not None and j.upper is not None:
            lines.append(f"    lower: {float(j.lower):.5f}")
            lines.append(f"    upper: {float(j.upper):.5f}")
        # round-trip the mimic coupling: a config rebuild (the editor's path)
        # takes the directed branch, which does NOT re-run the auto four-bar /
        # gear detection -- so without this the auto-detected <mimic> (and its
        # cubic ``poly`` for the ROS 2 loop relay) silently vanishes on the next
        # build.  Loader: resolve_directed reads this dict back verbatim.
        m = getattr(j, "mimic", None)
        if m and m.get("joint"):
            lines.append("    mimic:")
            lines.append(f"      joint: {m['joint']}")
            lines.append(f"      multiplier: {float(m.get('multiplier', 1.0)):g}")
            lines.append(f"      offset: {float(m.get('offset', 0.0)):g}")
            if m.get("poly"):
                poly = ", ".join(repr(float(x)) for x in m["poly"])
                lines.append(f"      poly: [{poly}]")
    # robot-compiler module interface (root is emitted as base_link by default)
    lines.append("")
    lines.append("# --- robot-compiler module interface (optional) ---")
    lines.append("# root_link_name: base_link   # input port; '' keeps the part name")
    lines.append("# ports:                       # output connectors (-> dummy_link)")
    lines.append("#   - parent: <tip link>       # xyz/rpy in that link's frame,")
    lines.append("#     xyz: [0, 0, 0]           # +Z = outgoing connector axis")
    lines.append("#     rpy: [0, 0, 0]")
    # reference: every mate-connected pair, so the chain can be re-wired
    lines.append("")
    lines.append("# --- all mate-connected pairs (for reference; re-wire above) ---")
    for e in sorted(model.detected_edges, key=lambda x: x["between"]):
        a, b = e["between"]
        lines.append(f"#   {a}  <->  {b}   ({', '.join(e['mates'])})")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")
    return path


def load(path):
    with open(path, encoding="utf-8") as f:
        return yaml.safe_load(f) or {}
