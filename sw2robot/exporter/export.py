"""SolidWorks assembly (or single part) -> URDF/ROS package, split into two phases.

  extract  (SLOW, needs SolidWorks): open a throwaway copy, pull the CAD graph
           + per-link coloured 3DXML meshes, write ``graph.json``.  Run once.
  build    (FAST, no SolidWorks): graph.json + joint config -> URDF/package.
           Re-run freely while tweaking base / exclude / axes / limits / root.

  export   = extract + build (one shot).

    uv run python -m sw2robot.exporter.export  <assembly.sldasm> [-o OUT] [-n NAME]
    uv run python -m sw2robot.exporter.extract <assembly.sldasm> [-o OUT] [-n NAME]
    uv run python -m sw2robot.exporter.build   <pkg_dir> [--config c.yaml] [--base ..] [--exclude ..]

A single ``.SLDPRT`` is also accepted anywhere a ``.SLDASM`` is: a lone part has
no mates to infer a kinematic tree from, so it yields a trivial 1-link, 0-joint
URDF carrying the part's SolidWorks-native mass/COM/inertia -- handy for a static
prop / environment object / single rigid body in a simulator.
"""

from __future__ import annotations

import argparse
import os
import sys

from . import jointcfg
from .mesh import (
    _SAVE_OPTS,
    export_meshes,
    export_part_mesh,
    export_subgraph_meshes,
)
from .model import (
    build_model,
    capture_deep_worlds,
    extract_graph,
    extract_limit_joints,
    extract_part_graph,
    extract_subgraphs,
    safe_name,
    to_graph_state,
)
from .state import GraphState
from .swcom import SW_DOC_PART, SolidWorks, as_iface, doc_type_for
from .urdf_writer import write_ros_package, write_urdf

GRAPH_FILE = "graph.json"


def _tolerant_console():
    """Don't let a non-ASCII component name (e.g. a Turkish 'gövde') crash a
    print on a legacy console code page (Japanese cp932, ...)."""
    for stream in (sys.stdout, sys.stderr):
        try:
            stream.reconfigure(errors="backslashreplace")
        except Exception:
            pass


def _pkg_paths(assembly_path, out_dir, robot_name):
    if robot_name is None:
        robot_name = safe_name(os.path.splitext(os.path.basename(assembly_path))[0])
    else:
        robot_name = safe_name(robot_name)
    if out_dir is None:
        out_dir = os.path.join(os.getcwd(), "output")
    pkg_dir = os.path.join(out_dir, robot_name)
    return robot_name, pkg_dir


# ---------------------------------------------------------------- extract
def _extract_part_into(sw, part_path, pkg_dir, meshes_dir, robot_name, _say):
    """Extraction body for a single ``.SLDPRT``: one link, no joints.

    A lone part carries no assembly structure, so there is nothing to infer a
    kinematic tree from -- we emit a trivial one-link robot with the part's
    SolidWorks-native mass/COM/inertia and its mesh.  See
    :func:`sw2robot.exporter.model.extract_part_graph`."""
    _say(f"opening copy of {os.path.basename(part_path)} (loading the part) ...")
    doc = sw.open_copy(part_path)

    _say("reading part mass properties ...")
    comps, adjacency, ground = extract_part_graph(doc, robot_name, part_path)

    _say("exporting part mesh ...")
    export_part_mesh(doc, comps[0], meshes_dir)

    _say("saving graph.json ...")
    graph = to_graph_state(comps, adjacency, ground, robot_name, part_path)
    graph.save(os.path.join(pkg_dir, GRAPH_FILE))
    sw.close_doc(doc)
    return pkg_dir


def _extract_into(sw, assembly_path, pkg_dir, meshes_dir, robot_name, _say,
                  _part=None):
    """Extraction body against an already-running SolidWorks session.  ``_part``
    (optional) reports the part currently being read, for the load indicator."""
    if doc_type_for(assembly_path) == SW_DOC_PART:
        return _extract_part_into(sw, assembly_path, pkg_dir, meshes_dir,
                                  robot_name, _say)
    _say(f"opening copy of {os.path.basename(assembly_path)} "
         f"(loading the assembly) ...")
    doc = sw.open_copy(assembly_path)

    _say("reading components + mates ...")
    comps, adjacency, ground = extract_graph(doc, robot_name, assembly_path,
                                             progress=_part)
    _say(f"found {len(comps)} components, {len(adjacency)} mate pairs")
    if not comps:
        sw.close_doc(doc)
        raise ValueError(
            "no usable components -- SolidWorks opened the assembly but every "
            "component is suppressed or unresolved (see the 'skipped ...' note "
            "above). This almost always means the .SLDASM's referenced part / "
            "sub-assembly files could not be found next to it: a lone .SLDASM "
            "copied into Downloads has no .SLDPRT parts to resolve against. "
            "Open it from its original folder (with its parts present), or use "
            "SolidWorks 'Pack and Go' to gather the assembly + all references "
            "into one folder and point sw2robot at the .SLDASM inside it.")

    _say("reading limit mates (sliders/hinges) ...")
    limit_joints = extract_limit_joints(doc, comps)
    if limit_joints:
        _say(f"found {len(limit_joints)} limit-mate joint(s)")

    _say("reading sub-assembly internals ...")
    subgraphs = extract_subgraphs(doc, comps, sw=sw, progress=_part)
    deep_worlds, hidden = capture_deep_worlds(doc)

    by_path = {}
    n = export_meshes(
        sw.app, doc, comps, meshes_dir,
        progress=lambda i, total, name: _say(
            f"exporting mesh {i}/{total}: {name}"),
        by_path=by_path)
    if subgraphs:
        _say("exporting sub-assembly internal meshes ...")
        n += export_subgraph_meshes(sw.app, subgraphs, meshes_dir,
                                    by_path=by_path)
    _say("exporting full assembly mesh ...")
    whole_rel = None
    whole = os.path.join(pkg_dir, robot_name + "_assembly.3dxml")
    try:
        ext = as_iface(doc.Extension, "IModelDocExtension")
        res = ext.SaveAs(whole, 0, _SAVE_OPTS, None, 0, 0)
        ok = res[0] if isinstance(res, (tuple, list)) else res
        if ok and os.path.exists(whole):
            whole_rel = os.path.basename(whole)
    except Exception as e:
        print(f"      assembly 3dxml raised {e!r}")
    _say(f"{n} meshes exported; saving graph.json ...")

    graph = to_graph_state(comps, adjacency, ground, robot_name,
                           assembly_path, assembly_mesh=whole_rel,
                           subassemblies=subgraphs, deep_worlds=deep_worlds,
                           hidden=hidden, limit_joints=limit_joints)
    graph.save(os.path.join(pkg_dir, GRAPH_FILE))
    sw.close_doc(doc)
    return pkg_dir


def extract(assembly_path, out_dir=None, robot_name=None, visible=False,
            progress=None, sw=None):
    """SolidWorks -> graph.json (+ per-link 3DXML).  ``progress(msg)`` -- if
    given -- receives short human-readable status strings at each stage and once
    per exported mesh, so a UI can show how far along the (multi-minute) extract
    is.  Pass an existing ``SolidWorks`` session as ``sw`` to reuse it across
    many assemblies (batch); otherwise a private one is started and shut down."""
    _tolerant_console()
    import time as _time
    t_state = {"last": _time.time()}

    def _say(msg):
        now = _time.time()
        dt = now - t_state["last"]
        t_state["last"] = now
        stamp = f" [+{dt:.1f}s]" if dt >= 0.05 else ""
        print("[extract] " + msg + stamp)
        if progress:
            progress(msg + stamp)

    # per-part read progress -- a transient "now reading <part>" for the load
    # indicator.  THROTTLED (200+ parts would otherwise flood the log) and routed
    # straight to ``progress`` (not _say): no console line, no timing stamp, and
    # tagged 'reading part:' so the UI shows the name without a stage banner.
    part_state = {"last": 0.0}

    def _part(name):
        if not progress:
            return
        now = _time.time()
        if now - part_state["last"] < 0.2:
            return
        part_state["last"] = now
        progress(f"reading part: {name}")

    assembly_path = os.path.abspath(assembly_path)
    robot_name, pkg_dir = _pkg_paths(assembly_path, out_dir, robot_name)
    meshes_dir = os.path.join(pkg_dir, "meshes")
    os.makedirs(meshes_dir, exist_ok=True)

    if sw is not None:
        _extract_into(sw, assembly_path, pkg_dir, meshes_dir, robot_name, _say,
                      _part)
    else:
        with SolidWorks(visible=visible) as sw_own:
            _extract_into(sw_own, assembly_path, pkg_dir, meshes_dir,
                          robot_name, _say, _part)

    print(f"  graph: {os.path.join(pkg_dir, GRAPH_FILE)}")
    return pkg_dir


def _loop_closures_cfg(model, joint_overrides):
    """The runtime-IK relay config from ``model.loop_closures``, with dependent/
    independent joint names mapped to the SAME final names the URDF emits (the
    closures' link names are already final)."""
    lc = getattr(model, "loop_closures", None)
    if not lc:
        return None
    from .model import safe_name
    jo = joint_overrides or {}

    def jn(n):
        return safe_name(jo.get(n, n))

    return {
        "closures": lc["closures"],
        "dependent": [jn(n) for n in lc["dependent"]],
        "independent": [jn(n) for n in lc["independent"]],
    }


# ---------------------------------------------------------------- build
def build(pkg_dir, config_path=None, base_hint=None, exclude=None,
          ros_pkg=False, density=None, ros_version=1, ros_pkg_name=None,
          ros_urdf_name=None, collision="copy", coacd_quality="balanced",
          merge_fixed=False, ros_mesh_dir=None):
    _tolerant_console()
    graph = GraphState.load(os.path.join(pkg_dir, GRAPH_FILE))
    robot_name = graph.robot_name
    urdf_path = os.path.join(pkg_dir, "urdf", robot_name + ".urdf")

    config = jointcfg.load(config_path) if config_path else None
    # density (kg/m^3) for the auto-computed link inertias: explicit arg wins,
    # else a top-level `density:` in the joint config, else the writer default.
    if density is None and isinstance(config, dict):
        density = config.get("density")
    print("[build] model from graph ...")
    model = build_model(graph, base_hint=base_hint, config=config,
                        exclude=exclude)
    print(f"      {len(model.components)} links, {len(model.joints)} joints")

    urdf_kwargs = {} if density is None else {"density": float(density)}
    # editor rename overlay: component link/joint name -> user-chosen display name
    if isinstance(config, dict):
        urdf_kwargs["link_overrides"] = config.get("link_names") or {}
        urdf_kwargs["joint_overrides"] = config.get("joint_names") or {}
    # the working URDF keeps URDF-relative mesh paths (our viewer + skrobot
    # auto-limits resolve those); the portable ROS variant is a SEPARATE package.
    # The working URDF KEEPS each mass-only link (geometry stripped) so it stays
    # selectable in the editor tree; only the exported package folds it away.
    mass_only_links = write_urdf(model, urdf_path, **urdf_kwargs)
    write_ros_package(model, pkg_dir)
    tmpl = os.path.join(pkg_dir, robot_name + ".joints.yaml")
    if not config_path:
        jointcfg.write_template(model, tmpl)

    # Persist any closed-loop data beside the package so a LATER ROS 2 export --
    # the CLI here OR the editor's ZIP download, which both only see the on-disk
    # package, not this model -- can ship the loop-closure relay + its config.
    # Refresh every build; drop stale files once a re-wire removes the loop.
    # (The editor re-runs build() on every edit, so this stays current.)
    import yaml as _yaml
    joint_overrides = (config.get("joint_names") or {}
                       if isinstance(config, dict) else {})
    closures = _loop_closures_cfg(model, joint_overrides)
    cside = os.path.join(pkg_dir, "loop_closures.yaml")
    if closures:
        with open(cside, "w", encoding="utf-8") as f:
            f.write("# Closed-loop closures for loop_closure_relay (runtime IK).\n")
            _yaml.safe_dump(closures, f, sort_keys=False, default_flow_style=None)
    elif os.path.exists(cside):
        os.remove(cside)

    # Persist the mass-only link names beside the package, same rationale as the
    # loop-closure sidecar: the detached ROS export (CLI here OR the editor ZIP)
    # only sees the on-disk package, so it reads this list to know which
    # geometry-less links to fold into their fixed parent.  Names are the final
    # (emitted) URDF link names, so they match the on-disk URDF directly.
    mside = os.path.join(pkg_dir, "mass_only.yaml")
    if mass_only_links:
        with open(mside, "w", encoding="utf-8") as f:
            f.write("# Mass-only links: weight kept, geometry dropped; folded "
                    "into their fixed parent on export.\n")
            _yaml.safe_dump(sorted(mass_only_links), f)
    elif os.path.exists(mside):
        os.remove(mside)

    desc_dir = None
    if ros_pkg:
        # a standalone package next to pkg_dir (default <robot_name>_description,
        # or --ros-pkg-name): package:// URLs + COLLADA .dae meshes
        # (RViz/Gazebo-ready).  ros_version 2 also bundles launch/ + rviz/.
        from .ros_export import write_ros_description_package
        # per-link colour overrides (joints.yaml `colors:`) repaint <visual>
        # meshes in the exported package
        colors = config.get("colors") if isinstance(config, dict) else None
        desc_dir = write_ros_description_package(
            pkg_dir, robot_name, os.path.dirname(os.path.abspath(pkg_dir)),
            ros_version=ros_version, pkg_name=ros_pkg_name,
            urdf_name=ros_urdf_name, colors=colors,
            collision=collision, coacd_quality=coacd_quality,
            merge_fixed=merge_fixed, mesh_dir=ros_mesh_dir,
            loop_closures=closures)

    print(f"\nDONE. Package: {pkg_dir}")
    print(f"  URDF:   {urdf_path}")
    if desc_dir:
        print(f"  ROS pkg: {desc_dir}  (ROS {ros_version}, package:// + .dae)")
    if not config_path:
        print(f"  Config: {tmpl}  (edit, re-run: "
              "python -m sw2robot.exporter.build with --config)")
    print(f"  View:   uv run visualize-urdf \"{urdf_path}\"")
    return urdf_path


# ---------------------------------------------------------------- export
def export(assembly_path, out_dir=None, robot_name=None, visible=False,
           config_path=None, base_hint=None, exclude=None, ros_pkg=False,
           ros_version=1, ros_pkg_name=None, ros_urdf_name=None,
           collision="copy", coacd_quality="balanced", merge_fixed=False,
           ros_mesh_dir=None):
    pkg_dir = extract(assembly_path, out_dir, robot_name, visible)
    return build(pkg_dir, config_path=config_path, base_hint=base_hint,
                 exclude=exclude, ros_pkg=ros_pkg, ros_version=ros_version,
                 ros_pkg_name=ros_pkg_name, ros_urdf_name=ros_urdf_name,
                 collision=collision, coacd_quality=coacd_quality,
                 merge_fixed=merge_fixed, ros_mesh_dir=ros_mesh_dir)


def _exclude_list(s):
    return [x.strip() for x in s.split(",")] if s else None


def main():
    ap = argparse.ArgumentParser(description="extract + build (full export)")
    ap.add_argument("assembly",
                    help="path to a .SLDASM assembly, or a single .SLDPRT part "
                         "(a lone part exports as a 1-link, 0-joint URDF)")
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("-n", "--name", default=None)
    ap.add_argument("--visible", action="store_true")
    ap.add_argument("--config", default=None)
    ap.add_argument("--base", default=None)
    ap.add_argument("--exclude", default=None)
    ap.add_argument("--ros-pkg", action="store_true",
                    help="also write a portable <name>_description package "
                         "(package:// URLs + COLLADA .dae meshes) next to the "
                         "output; the working URDF stays mesh-relative")
    ap.add_argument("--ros2", action="store_true",
                    help="make the --ros-pkg an ament_cmake (ROS 2) package "
                         "with launch/ + rviz/ instead of catkin (ROS 1); "
                         "implies --ros-pkg")
    ap.add_argument("--ros-pkg-name", default=None,
                    help="name for the --ros-pkg package (default "
                         "<name>_description); must be a valid ROS package "
                         "name: lowercase letters, digits, underscores")
    ap.add_argument("--ros-urdf-name", default=None,
                    help="stem for the URDF file inside the --ros-pkg package "
                         "(default: the package name)")
    ap.add_argument("--ros-mesh-dir", default=None,
                    help="package-relative directory the --ros-pkg meshes go in "
                         "and that the URDF's package:// refs point at (default: "
                         "'meshes'); e.g. 'urdf/mesh' for a different layout")
    ap.add_argument("--collision",
                    choices=("copy", "hull", "coacd",
                             "primitive", "box", "cylinder", "sphere"),
                    default="copy",
                    help="--ros-pkg <collision> geometry: 'copy' (default) "
                         "reuses the visual mesh as one STL; 'hull' replaces it "
                         "with a single convex hull STL; 'coacd' runs approximate "
                         "convex decomposition into convex part STLs (needs: pip "
                         "install coacd); 'primitive'/'box'/'cylinder'/'sphere' "
                         "fit a native URDF primitive per link, no mesh file "
                         "('primitive' auto-picks the best shape; needs: pip "
                         "install -U scikit-robot)")
    ap.add_argument("--coacd-quality", choices=("balanced", "fine"),
                    default="balanced",
                    help="CoACD preset for --collision coacd: 'balanced' "
                         "(default, ~5-6 parts/link, ~8-60s) or 'fine' "
                         "(~8 parts, tighter fit, ~2-3x slower)")
    ap.add_argument("--merge-fixed", action="store_true",
                    help="lump fixed-joint child links (with geometry) into "
                         "their parents in the --ros-pkg URDF -- one rigid link "
                         "per moving body; mesh-less coordinate frames are kept")
    args = ap.parse_args()
    export(args.assembly, args.out, args.name, args.visible,
           config_path=args.config, base_hint=args.base,
           exclude=_exclude_list(args.exclude),
           ros_pkg=args.ros_pkg or args.ros2,
           ros_version=2 if args.ros2 else 1,
           ros_pkg_name=args.ros_pkg_name, ros_urdf_name=args.ros_urdf_name,
           collision=args.collision, coacd_quality=args.coacd_quality,
           merge_fixed=args.merge_fixed, ros_mesh_dir=args.ros_mesh_dir)


if __name__ == "__main__":
    main()
