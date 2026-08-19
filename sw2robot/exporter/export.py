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
    _unique_part_names,
    export_meshes,
    export_part_mesh,
    export_subgraph_meshes,
    verify_meshes,
)
from .model import (
    build_model,
    capture_deep_worlds,
    extract_coordinate_systems,
    extract_graph,
    extract_limit_joints,
    extract_part_graph,
    extract_reference_axes,
    extract_reference_geometry,
    extract_subgraphs,
    extract_sw2urdf_attribute,
    safe_name,
    to_graph_state,
)
from .state import GraphState
from .swcom import (
    SW_DOC_PART,
    SolidWorks,
    as_iface,
    doc_type_for,
    safe_call,
    safe_prop,
)
from .urdf_writer import write_ros_package, write_urdf

GRAPH_FILE = "graph.json"


def configuration_names(cad_path, sw=None, visible=False):
    """Configuration names in a .SLDASM/.SLDPRT, WITHOUT opening it.

    ``ISldWorks::GetConfigurationNames`` reads them straight off the closed
    file, so a UI can offer the choice up front instead of paying the
    multi-minute assembly load first.  Pass a live ``SolidWorks`` session as
    ``sw`` to reuse it; otherwise a private one is started and shut down.

    Returns ``[]`` when the file has none to report -- callers should treat
    that as "just use the saved-active one", not as an error.  Which one IS
    saved-active is not knowable without opening the document; that is why the
    extract's ``configuration=None`` (keep whatever the file was saved on)
    stays the default.
    """
    cad_path = os.path.abspath(cad_path)
    if sw is not None:
        names = safe_call(sw.app, "GetConfigurationNames", cad_path)
    else:
        with SolidWorks(visible=visible) as own:
            names = safe_call(own.app, "GetConfigurationNames", cad_path)
    return [str(n) for n in (names or [])]


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
    (coordinate_systems,
     reference_axes,
     sw2urdf_marker,
     sw2urdf_config_xml) = extract_reference_geometry(doc)
    graph = to_graph_state(
        comps, adjacency, ground, robot_name, part_path,
        coordinate_systems=coordinate_systems,
        reference_axes=reference_axes,
        sw2urdf_marker=sw2urdf_marker,
        sw2urdf_config_xml=sw2urdf_config_xml)
    graph.save(os.path.join(pkg_dir, GRAPH_FILE))
    sw.close_doc(doc)
    return pkg_dir


def _extract_into(sw, assembly_path, pkg_dir, meshes_dir, robot_name, _say,
                  _part=None, configuration=None):
    """Extraction body against an already-running SolidWorks session.  ``_part``
    (optional) reports the part currently being read, for the load indicator.
    ``configuration`` -- extract THIS assembly configuration instead of the
    file's saved-active one (configs can suppress whole components, e.g. a
    bench-mount frame present in one variant only)."""
    if doc_type_for(assembly_path) == SW_DOC_PART:
        return _extract_part_into(sw, assembly_path, pkg_dir, meshes_dir,
                                  robot_name, _say)
    _say(f"opening copy of {os.path.basename(assembly_path)} "
         f"(loading the assembly) ...")
    doc = sw.open_copy(assembly_path)
    # surface the choice: extracts silently follow the file's SAVED-ACTIVE
    # configuration, which is not necessarily the one on the user's screen
    cfgs, used_cfg = [], None
    try:
        md = as_iface(doc, "IModelDoc2")
        cfgs = [str(c) for c in (safe_call(md, "GetConfigurationNames") or [])]
        active = md.ConfigurationManager.ActiveConfiguration.Name
        if len(cfgs) > 1:
            _say(f"assembly configurations: {cfgs} (saved-active: {active!r})")
        if configuration and configuration != active:
            if md.ShowConfiguration2(configuration):
                _say(f"switched to configuration {configuration!r}")
            else:
                print(f"      WARN: configuration {configuration!r} not found; "
                      f"staying on {active!r}")
        # what is ACTUALLY in effect -- a rejected switch must not be recorded
        # as if it had happened
        used_cfg = str(md.ConfigurationManager.ActiveConfiguration.Name)
        if len(cfgs) > 1:
            _say(f"extracting configuration {used_cfg!r}")
    except Exception as e:
        if configuration:
            print(f"      WARN: could not switch configuration ({e!r})")

    _say("reading components + mates ...")
    # frames authored INSIDE part files, collected during the per-part
    # mass-property read (no extra document loads)
    part_coordinate_systems = {}
    comps, adjacency, ground = extract_graph(
        doc, robot_name, assembly_path, progress=_part,
        part_coordinate_systems_out=part_coordinate_systems)
    (coordinate_systems,
     reference_axes,
     sw2urdf_marker,
     sw2urdf_config_xml) = extract_reference_geometry(doc)
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
    subassembly_coordinate_systems = {}
    subgraphs = extract_subgraphs(
        doc, comps, sw=sw, progress=_part,
        coordinate_systems_out=subassembly_coordinate_systems,
        part_coordinate_systems_out=part_coordinate_systems)
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
    # absolute: SolidWorks resolves a relative SaveAs path against its OWN cwd,
    # not ours, so a relative `-o output` dir fails the export (see _save_3dxml)
    whole = os.path.abspath(
        os.path.join(pkg_dir, robot_name + "_assembly.3dxml"))
    try:
        ext = as_iface(doc.Extension, "IModelDocExtension")
        res = ext.SaveAs(whole, 0, _SAVE_OPTS, None, 0, 0)
        ok = res[0] if isinstance(res, (tuple, list)) else res
        if ok and os.path.exists(whole):
            # this one SaveAs does not go through _save_3dxml, so give it the
            # same treatment: a part used with two configurations lands as two
            # same-named Reference3D nodes that collapse into one when the file
            # is read back (see mesh._dedupe_reference_names)
            n_dup = _unique_part_names(whole)
            if n_dup:
                _say(f"assembly mesh: disambiguated {n_dup} config "
                     f"variant(s) sharing a part name")
            whole_rel = os.path.basename(whole)
    except Exception as e:
        print(f"      assembly 3dxml raised {e!r}")

    # Read every mesh back and compare it with what the file says it holds.
    # An export that hands geometry to a loader and never checks it can be
    # silently WRONG -- two configuration variants of one part collapsing into
    # one shape, superimposed, with the part COUNT still correct.  This makes
    # that class of failure visible instead of shipping it into the URDF.
    _say("verifying meshes read back whole ...")
    written = sorted(
        os.path.join(meshes_dir, f) for f in os.listdir(meshes_dir)
        if f.lower().endswith(".3dxml"))
    if whole_rel:
        written.append(whole)
    verify_meshes(written, say=_say)

    _say(f"{n} meshes exported; saving graph.json ...")

    graph = to_graph_state(comps, adjacency, ground, robot_name,
                           assembly_path, assembly_mesh=whole_rel,
                           subassemblies=subgraphs, deep_worlds=deep_worlds,
                           hidden=hidden, limit_joints=limit_joints,
                           coordinate_systems=coordinate_systems,
                           reference_axes=reference_axes,
                           sw2urdf_marker=sw2urdf_marker,
                           sw2urdf_config_xml=sw2urdf_config_xml,
                           subassembly_coordinate_systems=
                           subassembly_coordinate_systems,
                           part_coordinate_systems=part_coordinate_systems,
                           configuration=used_cfg, configurations=cfgs)
    graph.save(os.path.join(pkg_dir, GRAPH_FILE))
    sw.close_doc(doc)
    return pkg_dir


def extract(assembly_path, out_dir=None, robot_name=None, visible=False,
            progress=None, sw=None, configuration=None, attach=False):
    """SolidWorks -> graph.json (+ per-link 3DXML).  ``progress(msg)`` -- if
    given -- receives short human-readable status strings at each stage and once
    per exported mesh, so a UI can show how far along the (multi-minute) extract
    is.  Pass an existing ``SolidWorks`` session as ``sw`` to reuse it across
    many assemblies (batch); otherwise a private one is started and shut down.
    ``attach=True`` connects to the USER'S already-running SolidWorks instead
    (their documents are left untouched); if the assembly is already open
    there, the multi-minute reopen is skipped entirely.  No fallback: with no
    running SolidWorks to attach to this raises ``SolidWorksUnavailable``."""
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
                      _part, configuration=configuration)
    else:
        sw_ctx = (SolidWorks(attach=True) if attach
                  else SolidWorks(visible=visible))
        with sw_ctx as sw_own:
            _extract_into(sw_own, assembly_path, pkg_dir, meshes_dir,
                          robot_name, _say, _part,
                          configuration=configuration)

    print(f"  graph: {os.path.join(pkg_dir, GRAPH_FILE)}")
    return pkg_dir


# ---------------------------------------------------------------- refresh
def _file_key(path):
    """Lower-cased file name of ``path``, split on EITHER separator --
    graph.json carries Windows paths even when it is read on POSIX (the
    build/test path), where os.path.basename would keep the whole string."""
    return str(path).replace("\\", "/").rsplit("/", 1)[-1].lower()


def _live_subassembly_docs(doc):
    """``basename(path).lower() -> open ModelDoc2`` for every loaded
    sub-assembly occurrence in ``doc``, at any depth.  Matched by file name
    because a copied session records temp-dir paths in graph.json while an
    attached one records the originals -- the basename is the stable part."""
    out = {}
    for c in list(safe_call(doc, "GetComponents", False) or []):
        ct = as_iface(c, "IComponent2")
        path = safe_prop(ct, "GetPathName") or ""
        if not str(path).lower().endswith(".sldasm"):
            continue
        key = _file_key(path)
        if key in out:
            continue
        md = safe_call(ct, "GetModelDoc2")
        if md is not None:
            out[key] = md
    return out


def refresh_frames(target, out_dir=None, robot_name=None, visible=False,
                   attach=False, sw=None, progress=None):
    """Partial re-extract: re-read ONLY the named frames -- the top-level
    coordinate systems + reference axes, and the coordinate systems of already
    -extracted sub-assemblies -- into an existing ``graph.json``.

    Components, mates, mass properties, deep worlds and meshes are left
    untouched, so this is a handful of feature-tree reads instead of a full
    assembly walk: seconds instead of minutes when iterating on frame
    selection (e.g. the PR #178 coordinate-frame work).  Combine with
    ``attach=True`` while the assembly is open in the user's SolidWorks and
    even the document open is skipped.

    ``target`` is either the extracted package dir (the one holding
    ``graph.json``) or the source assembly path (the package dir is then
    derived exactly like :func:`extract` does from ``out_dir``/``robot_name``).
    The frames are read from whatever configuration the document opens with
    (in attach mode: the one on the user's screen).  Requires a prior full
    extract -- there is deliberately NO fallback to one."""
    _tolerant_console()
    import time as _time
    t0 = _time.time()

    def _say(msg):
        print("[refresh] " + msg)
        if progress:
            progress(msg)

    target = os.path.abspath(target)
    if os.path.isdir(target):
        pkg_dir = target
        assembly_path = None
    else:
        assembly_path = target
        _, pkg_dir = _pkg_paths(target, out_dir, robot_name)
    graph_path = os.path.join(pkg_dir, GRAPH_FILE)
    if not os.path.exists(graph_path):
        raise FileNotFoundError(
            f"{graph_path} not found -- '--refresh frames' only updates an "
            f"existing extraction; run a full extract once first")
    graph = GraphState.load(graph_path)
    if assembly_path is None:
        assembly_path = graph.source_assembly
    if not os.path.exists(assembly_path):
        raise FileNotFoundError(
            f"source assembly not found: {assembly_path}")

    def _refresh_into(sw_sess):
        _say(f"opening {os.path.basename(assembly_path)} (instant when it is "
             f"already open in an attached SolidWorks) ...")
        doc = sw_sess.open_copy(assembly_path)
        try:
            _say("re-reading coordinate systems + reference axes ...")
            graph.coordinate_systems = extract_coordinate_systems(
                doc, owners=True)
            graph.reference_axes = extract_reference_axes(doc)
            (graph.sw2urdf_marker,
             graph.sw2urdf_config_xml) = extract_sw2urdf_attribute(doc)
            if graph.subassemblies:
                live = _live_subassembly_docs(doc)
                for path, sub in graph.subassemblies.items():
                    md = live.get(_file_key(path))
                    if md is None:
                        print(f"      WARN: {os.path.basename(path)} is not "
                              f"loaded in this session; keeping its cached "
                              f"coordinate systems")
                        continue
                    sub.coordinate_systems = extract_coordinate_systems(
                        md, owners=True)
            # frames drawn inside PART files: re-read from the loaded part
            # documents.  This session's paths may be temp-dir copies while
            # graph.json holds the originals, so key them back onto the paths
            # the graph already uses (same basename rule as sub-assemblies).
            known = {}
            for cs in graph.components:
                if cs.part_path:
                    known.setdefault(_file_key(cs.part_path), cs.part_path)
            for sub in graph.subassemblies.values():
                for cs in sub.components:
                    if cs.part_path:
                        known.setdefault(_file_key(cs.part_path), cs.part_path)
            # start from what the graph already knows: a part whose document is
            # not loaded in this session keeps its cached frames instead of
            # silently losing them (same rule as the sub-assemblies above)
            part_frames = dict(graph.part_coordinate_systems)
            seen_parts, unloaded = set(), 0
            for c in list(safe_call(doc, "GetComponents", False) or []):
                ct = as_iface(c, "IComponent2")
                path = str(safe_prop(ct, "GetPathName") or "")
                if not path or path.lower().endswith(".sldasm"):
                    continue
                key = _file_key(path)
                if key in seen_parts:
                    continue
                seen_parts.add(key)
                md = safe_call(ct, "GetModelDoc2")
                if md is None:
                    unloaded += 1
                    continue
                frames = extract_coordinate_systems(md)
                target = known.get(key, path)
                if frames:
                    part_frames[target] = frames
                else:
                    # read successfully and it has none any more: a frame the
                    # author DELETED must disappear from graph.json too
                    part_frames.pop(target, None)
            if unloaded:
                print(f"      WARN: {unloaded} part document(s) are not loaded "
                      f"in this session; keeping their cached coordinate "
                      f"systems")
            graph.part_coordinate_systems = part_frames
        finally:
            sw_sess.close_doc(doc)

    if sw is not None:
        _refresh_into(sw)
    else:
        sw_ctx = (SolidWorks(attach=True) if attach
                  else SolidWorks(visible=visible))
        with sw_ctx as sw_own:
            _refresh_into(sw_own)

    graph.save(graph_path)
    n_sub = sum(len(s.coordinate_systems)
                for s in graph.subassemblies.values())
    n_part = sum(len(v) for v in graph.part_coordinate_systems.values())
    _say(f"graph.json frames updated in {_time.time() - t0:.1f}s: "
         f"{len(graph.coordinate_systems)} coordinate system(s), "
         f"{len(graph.reference_axes)} reference axis/axes"
         + (f", {n_sub} sub-assembly frame(s)" if graph.subassemblies else "")
         + (f", {n_part} part frame(s)" if n_part else ""))
    print(f"  graph: {graph_path}")
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
          ros_urdf_name=None, ros_robot_name=None, collision="copy",
          coacd_quality="balanced", merge_fixed=False, ros_mesh_dir=None):
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
    # An SW2URDF config may bundle several loose components into ONE link;
    # they leave the tree as fixed children, so lump exactly those back into
    # their parent to reproduce the authored link granularity.
    if getattr(model, "lumped_links", None):
        import xml.etree.ElementTree as _ET

        from skrobot.urdf import merge_fixed_links
        _tree = _ET.parse(urdf_path)
        _root = _tree.getroot()          # skrobot merges IN PLACE
        _res = merge_fixed_links(_root, only=list(model.lumped_links))
        _n = _res[0] if isinstance(_res, tuple) else _res
        if _n:
            _tree.write(urdf_path, encoding="utf-8", xml_declaration=True)
            print(f"      SW2URDF config: lumped {_n} member link(s) into "
                  "their configured link")
    # Surface parts that resolved + got a mesh but whose geometry never makes it
    # into the URDF (classic case: a sub-assembly kept as one composed mesh whose
    # 3DXML export config suppresses some children).  Best-effort: warn_dropped_
    # geometry itself returns [] when trimesh/scipy/skrobot are absent, so this
    # is a silent no-op there; the guard only catches unexpected failures and is
    # advisory -- it must never break the build.
    try:
        import json as _json

        from .validate import warn_dropped_geometry
        with open(os.path.join(pkg_dir, GRAPH_FILE), encoding="utf-8") as _f:
            _graph = _json.load(_f)
        _no_geo = {c.name for c in model.components
                   if getattr(c, "frame_only", False)
                   or getattr(c, "mass_only", False)}
        _dropped = warn_dropped_geometry(pkg_dir, urdf_path, _graph,
                                         skip_components=_no_geo)
        if _dropped:
            print("      -> add the parent sub-assembly to the joint config's "
                  "`expand:` list to bring these parts into the URDF.")
    except Exception as _e:
        print(f"      (dropped-geometry check skipped: {_e!r})")
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
            urdf_name=ros_urdf_name, robot_tag=ros_robot_name, colors=colors,
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
           ros_robot_name=None,
           collision="copy", coacd_quality="balanced", merge_fixed=False,
           ros_mesh_dir=None, configuration=None, attach=False):
    pkg_dir = extract(assembly_path, out_dir, robot_name, visible,
                      configuration=configuration, attach=attach)
    return build(pkg_dir, config_path=config_path, base_hint=base_hint,
                 exclude=exclude, ros_pkg=ros_pkg, ros_version=ros_version,
                 ros_pkg_name=ros_pkg_name, ros_urdf_name=ros_urdf_name,
                 ros_robot_name=ros_robot_name,
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
    ap.add_argument("--attach", action="store_true",
                    help="reuse the USER'S already-running SolidWorks instead "
                         "of starting a hidden private instance; if the "
                         "assembly is already open there, the multi-minute "
                         "reopen is skipped entirely.  Errors out (no "
                         "fallback) when no running SolidWorks is found")
    ap.add_argument("--configuration", default=None, metavar="NAME",
                    help="extract this ASSEMBLY configuration instead of the "
                         "file's saved-active one (configs can suppress whole "
                         "components, e.g. a bench-mount frame)")
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
    ap.add_argument("--ros-robot-name", default=None,
                    help="name written into the exported URDF's "
                         "<robot name=\"...\"> (default: the URDF stem)")
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
                         "('primitive' auto-picks the best shape)")
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
           configuration=args.configuration, attach=args.attach,
           config_path=args.config, base_hint=args.base,
           exclude=_exclude_list(args.exclude),
           ros_pkg=args.ros_pkg or args.ros2,
           ros_version=2 if args.ros2 else 1,
           ros_pkg_name=args.ros_pkg_name, ros_urdf_name=args.ros_urdf_name,
           ros_robot_name=args.ros_robot_name,
           collision=args.collision, coacd_quality=args.coacd_quality,
           merge_fixed=args.merge_fixed, ros_mesh_dir=args.ros_mesh_dir)


if __name__ == "__main__":
    main()
