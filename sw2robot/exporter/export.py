"""SolidWorks assembly -> URDF/ROS package, split into two phases.

  extract  (SLOW, needs SolidWorks): open a throwaway copy, pull the CAD graph
           + per-link coloured 3DXML meshes, write ``graph.json``.  Run once.
  build    (FAST, no SolidWorks): graph.json + joint config -> URDF/package.
           Re-run freely while tweaking base / exclude / axes / limits / root.

  export   = extract + build (one shot).

    uv run python -m sw2robot.exporter.export  <assembly.sldasm> [-o OUT] [-n NAME]
    uv run python -m sw2robot.exporter.extract <assembly.sldasm> [-o OUT] [-n NAME]
    uv run python -m sw2robot.exporter.build   <pkg_dir> [--config c.yaml] [--base ..] [--exclude ..]
"""

from __future__ import annotations

import argparse
import os

from .swcom import SolidWorks, as_iface
from .model import (extract_graph, extract_subgraphs, to_graph_state,
                    capture_deep_worlds, build_model, safe_name)
from .mesh import export_meshes, export_subgraph_meshes, _SAVE_OPTS
from .state import GraphState
from .urdf_writer import write_urdf, write_ros_package
from . import jointcfg

GRAPH_FILE = "graph.json"


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
def _extract_into(sw, assembly_path, pkg_dir, meshes_dir, robot_name, _say):
    """Extraction body against an already-running SolidWorks session."""
    _say(f"opening copy of {os.path.basename(assembly_path)} "
         f"(loading the assembly) ...")
    doc = sw.open_copy(assembly_path)

    _say("reading components + mates ...")
    comps, adjacency, ground = extract_graph(doc, robot_name, assembly_path)
    _say(f"found {len(comps)} components, {len(adjacency)} mate pairs")

    _say("reading sub-assembly internals ...")
    subgraphs = extract_subgraphs(doc, comps, sw=sw)
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
                           hidden=hidden)
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

    assembly_path = os.path.abspath(assembly_path)
    robot_name, pkg_dir = _pkg_paths(assembly_path, out_dir, robot_name)
    meshes_dir = os.path.join(pkg_dir, "meshes")
    os.makedirs(meshes_dir, exist_ok=True)

    if sw is not None:
        _extract_into(sw, assembly_path, pkg_dir, meshes_dir, robot_name, _say)
    else:
        with SolidWorks(visible=visible) as sw_own:
            _extract_into(sw_own, assembly_path, pkg_dir, meshes_dir,
                          robot_name, _say)

    print(f"  graph: {os.path.join(pkg_dir, GRAPH_FILE)}")
    return pkg_dir


# ---------------------------------------------------------------- build
def build(pkg_dir, config_path=None, base_hint=None, exclude=None,
          ros_pkg=False, density=None):
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
    write_urdf(model, urdf_path, ros_pkg=robot_name if ros_pkg else None,
               **urdf_kwargs)
    write_ros_package(model, pkg_dir)
    tmpl = os.path.join(pkg_dir, robot_name + ".joints.yaml")
    if not config_path:
        jointcfg.write_template(model, tmpl)

    print(f"\nDONE. Package: {pkg_dir}")
    print(f"  URDF:   {urdf_path}")
    if not config_path:
        print(f"  Config: {tmpl}  (edit, re-run: "
              "python -m sw2robot.exporter.build with --config)")
    print(f"  View:   uv run visualize-urdf \"{urdf_path}\" --viewer viser")
    return urdf_path


# ---------------------------------------------------------------- export
def export(assembly_path, out_dir=None, robot_name=None, visible=False,
           config_path=None, base_hint=None, exclude=None, ros_pkg=False):
    pkg_dir = extract(assembly_path, out_dir, robot_name, visible)
    return build(pkg_dir, config_path=config_path, base_hint=base_hint,
                 exclude=exclude, ros_pkg=ros_pkg)


def _exclude_list(s):
    return [x.strip() for x in s.split(",")] if s else None


def main():
    ap = argparse.ArgumentParser(description="extract + build (full export)")
    ap.add_argument("assembly")
    ap.add_argument("-o", "--out", default=None)
    ap.add_argument("-n", "--name", default=None)
    ap.add_argument("--visible", action="store_true")
    ap.add_argument("--config", default=None)
    ap.add_argument("--base", default=None)
    ap.add_argument("--exclude", default=None)
    ap.add_argument("--ros-pkg", action="store_true")
    args = ap.parse_args()
    export(args.assembly, args.out, args.name, args.visible,
           config_path=args.config, base_hint=args.base,
           exclude=_exclude_list(args.exclude), ros_pkg=args.ros_pkg)


if __name__ == "__main__":
    main()
