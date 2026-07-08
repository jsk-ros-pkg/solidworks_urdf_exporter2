"""Export each link's geometry to a coloured .3dxml (part-local coordinates).

Meshes are emitted ONCE per unique part file (instances share geometry).  We
prefer the already-loaded ``GetModelDoc2`` of a component; when that is not
available (sub-assemblies, lightweight parts) we open the referenced file
directly with the full-load flag.  Nothing is ever saved -- exports use the
Copy option so the source documents are never dirtied, and opened files are
closed afterwards.
"""

from __future__ import annotations

import os

from .swcom import (
    SW_OPEN_SILENT,
    SW_SAVEAS_COPY,
    SW_SAVEAS_SILENT,
    as_iface,
    doc_type_for,
    open_doc6,
    safe_call,
    safe_prop,
)

_SAVE_OPTS = SW_SAVEAS_SILENT | SW_SAVEAS_COPY  # 3
# a 3DXML below this is just the empty-document envelope (no tessellation);
# lightweight sub-assemblies produce ~850 B files that LOOK successful
_MIN_MESH_BYTES = 2000

# Opening certain (usually imported/downloaded) part files CRASHES the whole
# SolidWorks process; every later COM call then fails with RPC disconnect.
# Remember the files that were in flight when a crash happened so the batch
# retry (fresh session) skips just their meshes instead of dying again.
_RPC_DISCONNECTED = -2147417848
_crash_suspects = set()
_recent_opens = []
# meshes/<file>.glb -> source part path, for per-part meshes persisted while
# composing a sub-assembly (so two parts that sanitize to the same name never
# clobber each other across the several compose passes in one extract)
_persisted = {}


def _open_doc(app, path):
    """OpenDoc6 with crash bookkeeping; None on (non-fatal) failure."""
    if path in _crash_suspects:
        print(f"  (skipping {os.path.basename(path)} -- it crashed "
              f"SolidWorks earlier; no mesh)")
        return None
    _recent_opens.append(path)
    del _recent_opens[:-2]                  # keep the last two
    try:
        doc, _err, _warn = open_doc6(app, path, doc_type_for(path),
                                     SW_OPEN_SILENT | 0x80)
        return doc
    except Exception as e:
        if getattr(e, "hresult", None) == _RPC_DISCONNECTED:
            _crash_suspects.update(_recent_opens)
            print(f"  SolidWorks DIED around "
                  f"{os.path.basename(path)}; blacklisting recent file(s) "
                  f"for the retry: "
                  f"{[os.path.basename(p) for p in _recent_opens]}")
            raise
        print(f"  open failed for {os.path.basename(path)}: {e!r}")
        return None


def _save_3dxml(model_doc, out_path):
    # write to a temp name and os.replace on success: a SolidWorks crash
    # mid-SaveAs must not leave a partial file that later passes the
    # size-based reuse checks
    tmp = out_path + ".part.3dxml"
    ext = as_iface(model_doc.Extension, "IModelDocExtension")
    try:
        res = ext.SaveAs(tmp, 0, _SAVE_OPTS, None, 0, 0)
    except Exception:
        try:
            os.remove(tmp)
        except OSError:
            pass
        raise
    ok = bool(res[0]) if isinstance(res, (tuple, list)) else bool(res)
    if ok and os.path.exists(tmp) and os.path.getsize(tmp) >= _MIN_MESH_BYTES:
        os.replace(tmp, out_path)
        return True
    try:
        os.remove(tmp)
    except OSError:
        pass
    return False


def _cache_is_fresh(cand, source_path):
    """Whether a cached per-part mesh may be reused instead of re-exported.

    Reuse only when the cache is real geometry (``>= _MIN_MESH_BYTES``) AND at
    least as new as its source part file.  The reuse-by-name shortcut speeds up
    re-runs, but keyed on the part name alone it also survives a CAD EDIT: the
    edited part re-exports to the same ``<name>.3dxml`` the old (stale) file
    already occupies, so without this mtime gate the change is silently masked
    until the user wipes ``%TEMP%\\sw2robot\\output``.  Comparing mtimes lets an
    edit since the last extract force a fresh export.

    If the source mtime can't be read (part moved/renamed away), fall back to
    reusing the cache -- we could not re-export it anyway."""
    try:
        if not (os.path.exists(cand)
                and os.path.getsize(cand) >= _MIN_MESH_BYTES):
            return False
    except OSError:
        return False
    try:
        return os.path.getmtime(cand) >= os.path.getmtime(source_path)
    except OSError:
        return True


def export_meshes(app, doc, comps, meshes_dir, progress=None, by_path=None):
    """Fill ``component.mesh_file`` for every component; return mesh count.

    ``progress(done, total, name)`` -- if given -- is called as each unique part
    is about to be exported, so a UI can show "mesh 7/34: <link>".  ``total`` is
    the number of distinct part files (instances share geometry, so it is < the
    component count).  ``by_path`` (part_path -> relative mesh file) may be
    shared with :func:`export_subgraph_meshes` so a part used both at the top
    level and inside a sub-assembly is exported once."""
    os.makedirs(meshes_dir, exist_ok=True)

    # map component Name2 -> live typed IComponent2 (for GetModelDoc2)
    live = {}
    for c in list(safe_call(doc, "GetComponents", True) or []):
        ct = as_iface(c, "IComponent2")
        live[safe_prop(ct, "Name2")] = ct

    total = len({c.part_path for c in comps if c.part_path})
    if by_path is None:
        by_path = {}   # part_path -> relative mesh file
    n = 0
    for comp in comps:
        path = comp.part_path
        if not path:
            print(f"  WARN: no part path for {comp.name}; skipping mesh")
            continue
        if path in by_path:
            comp.mesh_file = by_path[path]
            continue
        if progress:
            progress(len(by_path) + 1, total, comp.link_name)
        out = os.path.join(meshes_dir, comp.link_name + ".3dxml")
        reused = False
        for cand in (out, os.path.join(meshes_dir, comp.link_name + ".glb")):
            if _cache_is_fresh(cand, path):
                rel = os.path.join("meshes", os.path.basename(cand))
                by_path[path] = rel
                comp.mesh_file = rel
                n += 1
                reused = True
                break  # reuse existing mesh (fast re-runs, unless CAD is newer)
        if reused:
            continue
        ok = False
        why = None                       # specific reason, surfaced on failure
        ct = live.get(comp.name)
        md = safe_call(ct, "GetModelDoc2") if ct else None
        if md is not None:
            # in-session doc first -- for SUB-ASSEMBLIES this is the copy the
            # parent already fully resolved, so it exports real geometry where
            # a standalone OpenDoc6 of the same file comes up hollow
            try:
                ok = _save_3dxml(md, out)
                if not ok:
                    why = "in-session 3DXML export was empty"
            except Exception as e:
                why = f"in-session export raised {e!r}"
                print(f"  {comp.name}: in-session export failed ({e!r}); "
                      f"opening file")
        if not ok:
            if not os.path.exists(path):
                why = "referenced part file not found on disk (unresolved reference)"
            else:
                ok = _export_by_opening(app, path, out)
                if not ok:
                    why = ("opened but produced no geometry -- no solid bodies "
                           "(an imported STEP / graphics body?) or it would not open")
        if not ok and comp.is_subassembly:
            # 3DXML of a sub-assembly doc reliably comes out EMPTY however it
            # is opened; compose the mesh from its child PARTS instead (parts
            # always export) into a single .glb in sub-assembly coordinates
            out = os.path.join(meshes_dir, comp.link_name + ".glb")
            print(f"  composing {comp.link_name}.glb from child parts ...")
            ok = _compose_from_parts(app, md, path, out,
                                     meshes_dir=meshes_dir, by_path=by_path)
            if not ok:
                why = "sub-assembly mesh could not be composed from its child parts"
        if ok:
            rel = os.path.join("meshes", os.path.basename(out))
            by_path[path] = rel
            comp.mesh_file = rel
            n += 1
            print(f"  mesh: {comp.link_name} <- {os.path.basename(path)} "
                  f"({os.path.getsize(out)} B)")
        else:
            # surface WHY instead of a bare "FAILED mesh": distinguishes an
            # unresolved reference from a bodiless / STEP part from a compose miss
            print(f"  FAILED mesh for {comp.name} "
                  f"({os.path.basename(path)}): {why or 'unknown reason'}")
    return n


def export_part_mesh(md, comp, meshes_dir):
    """Export a single PART doc's geometry to ``meshes/<link>.3dxml`` and set
    ``comp.mesh_file``; return 1 on success, 0 otherwise.

    ``md`` is the already-open ``IModelDoc2`` of the ``.SLDPRT`` (part-local
    frame, the same frame its inertial is in), so no re-open is needed.  A fresh
    cache of real geometry is reused when present (see :func:`_cache_is_fresh`)."""
    os.makedirs(meshes_dir, exist_ok=True)
    out = os.path.join(meshes_dir, comp.link_name + ".3dxml")
    if _cache_is_fresh(out, comp.part_path):
        comp.mesh_file = os.path.join("meshes", os.path.basename(out))
        return 1
    ok = False
    try:
        ok = _save_3dxml(md, out)
    except Exception as e:
        print(f"  part mesh in-session export failed ({e!r})")
    if ok:
        comp.mesh_file = os.path.join("meshes", os.path.basename(out))
        print(f"  mesh: {comp.link_name} <- {os.path.basename(comp.part_path)} "
              f"({os.path.getsize(out)} B)")
        return 1
    print(f"  FAILED part mesh for {comp.name} "
          f"({os.path.basename(comp.part_path)})")
    return 0


def export_subgraph_meshes(app, subgraphs, meshes_dir, by_path=None):
    """Meshes for every sub-assembly-internal component, so a build-time
    expansion has per-child visuals.  ``subgraphs`` is
    ``{part_path: (comps, adjacency, ground)}``; fills each child's
    ``mesh_file``.  Unique part files shared with the top level (via
    ``by_path``) are not exported twice."""
    from .model import safe_name

    os.makedirs(meshes_dir, exist_ok=True)
    if by_path is None:
        by_path = {}
    n = 0
    for path, (scomps, _adj, _ground) in (subgraphs or {}).items():
        prefix = safe_name(os.path.splitext(os.path.basename(path))[0])
        for sc in scomps:
            p = sc.part_path
            if not p:
                continue
            if p in by_path:
                sc.mesh_file = by_path[p]
                n += 1
                continue
            base = f"{prefix}__{sc.link_name}"
            out = os.path.join(meshes_dir, base + ".3dxml")
            ok = False
            for cand in (out, os.path.join(meshes_dir, base + ".glb")):
                if _cache_is_fresh(cand, p):
                    out, ok = cand, True
                    break
            if not ok:
                ok = _export_by_opening(app, p, out)
            if not ok and p.lower().endswith(".sldasm"):
                out = os.path.join(meshes_dir, base + ".glb")
                print(f"  composing {base}.glb from child parts ...")
                ok = _compose_from_parts(app, None, p, out,
                                         meshes_dir=meshes_dir, by_path=by_path)
            if ok:
                rel = os.path.join("meshes", os.path.basename(out))
                by_path[p] = rel
                sc.mesh_file = rel
                n += 1
                print(f"  sub-mesh: {base} <- {os.path.basename(p)} "
                      f"({os.path.getsize(out)} B)")
            else:
                print(f"  FAILED sub-mesh for {sc.name} "
                      f"({os.path.basename(p)})")
    return n


def _compose_from_parts(app, md, path, out_glb, meshes_dir=None, by_path=None):
    """Merge a sub-assembly's child PART meshes into one .glb (sub-assembly
    local coordinates, metres).  Used when the sub-assembly's own 3DXML
    export is empty.  ``md`` may be None -- then ``path`` is opened.

    When ``meshes_dir``/``by_path`` are given, each child PART is ALSO persisted
    as its own part-local ``.glb`` and registered in ``by_path`` (keyed by its
    file path).  A standalone cold open of these parts reliably comes up empty,
    so this in-session walk is the only place their geometry exports -- saving
    it here gives a build-time sub-assembly expansion real per-child visuals
    (otherwise every expanded leaf link is mesh-less and vanishes)."""
    import tempfile

    import numpy as np
    import trimesh

    from .geometry import transform_to_matrix
    from .model import safe_name

    opened = None
    if md is None:
        md = _open_doc(app, path)
        opened = md
        if md is None:
            return False
    tmpd = tempfile.mkdtemp(prefix="sw2urdf_sub_")
    meshes = []

    def _persist_part(cpath, m_local):
        """Save a part-local trimesh as its own meshes/<name>.glb (once)."""
        if meshes_dir is None or by_path is None or cpath in by_path:
            return
        stem = safe_name(os.path.splitext(os.path.basename(cpath))[0])
        dst = os.path.join(meshes_dir, stem + ".glb")
        # two different part files may sanitize to the same stem -- never clobber
        i = 1
        while dst in _persisted and _persisted[dst] != cpath:
            i += 1
            dst = os.path.join(meshes_dir, f"{stem}_{i}.glb")
        try:
            tmp = dst + ".part.glb"
            m_local.export(tmp, file_type="glb")
            if os.path.exists(tmp) and os.path.getsize(tmp) > 500:
                os.replace(tmp, dst)
                _persisted[dst] = cpath
                by_path[cpath] = os.path.join("meshes", os.path.basename(dst))
        except Exception as e:
            print(f"    compose: could not persist part mesh "
                  f"{os.path.basename(cpath)}: {e!r}")

    def walk(doc_md, T_parent):
        for c in list(safe_call(doc_md, "GetComponents", True) or []):
            ct = as_iface(c, "IComponent2")
            if safe_call(ct, "GetSuppression") == 0:
                continue
            cpath = safe_prop(ct, "GetPathName")
            if not cpath:
                continue
            T = T_parent
            try:
                T = T_parent @ transform_to_matrix(
                    safe_prop(ct, "Transform2").ArrayData)
            except Exception:
                pass
            cmd = safe_call(ct, "GetModelDoc2")
            if cpath.lower().endswith(".sldasm"):
                if cmd is not None:
                    walk(cmd, T)
                else:
                    # nested sub-assembly not in memory: open it ourselves,
                    # otherwise its whole branch (motors etc.) silently
                    # disappears from the composed mesh
                    sub = _open_doc(app, cpath)
                    if sub is not None:
                        try:
                            walk(sub, T)
                        finally:
                            try:
                                app.CloseDoc(safe_prop(sub, "GetTitle"))
                            except Exception:
                                pass
                    else:
                        print(f"    compose: could NOT open nested "
                              f"sub-assembly {os.path.basename(cpath)} -- "
                              f"branch missing from mesh")
                continue
            f = os.path.join(tmpd, f"p{len(meshes)}.3dxml")
            ok = False
            if cmd is not None:
                try:
                    ok = _save_3dxml(cmd, f)
                except Exception:
                    pass
            if not ok:
                ok = _export_by_opening(app, cpath, f)
            if not ok:
                print(f"    compose: part export failed: "
                      f"{os.path.basename(cpath)}")
                continue
            try:
                m = trimesh.load(f)
                if isinstance(m, trimesh.Scene):
                    m = m.to_mesh() if hasattr(m, "to_mesh") \
                        else m.dump(concatenate=True)
                m.apply_scale(0.001)        # 3DXML tessellation is mm
                # the 'mm' units tag survives apply_scale; left as-is it
                # makes unit-aware loaders (skrobot) shrink the mesh 1000x
                m.units = "meter"
                # persist the part-local mesh (pre-transform) for reuse as an
                # expanded child's own visual, THEN place it for the merge
                _persist_part(cpath, m)
                m = m.copy()
                m.apply_transform(T)
                meshes.append(m)
            except Exception as e:
                print(f"    compose: mesh load failed "
                      f"{os.path.basename(cpath)}: {e!r}")

    try:
        walk(md, np.eye(4))
    finally:
        if opened is not None:
            try:
                app.CloseDoc(safe_prop(opened, "GetTitle"))
            except Exception:
                pass
    if not meshes:
        return False
    print(f"    compose: merged {len(meshes)} part meshes")
    merged = trimesh.util.concatenate(meshes)
    merged.units = "meter"
    tmp = out_glb + ".part.glb"
    merged.export(tmp, file_type="glb")
    if os.path.exists(tmp) and os.path.getsize(tmp) > 500:
        os.replace(tmp, out_glb)
        return True
    try:
        os.remove(tmp)
    except OSError:
        pass
    return False


def _export_by_opening(app, path, out):
    if not os.path.exists(path):
        print(f"  part file missing: {path}")
        return False
    # 0x80 inside _open_doc = swOpenDocOptions_OverrideDefaultLoadLightweight:
    # force a fully resolved load even when the system default is lightweight
    md = _open_doc(app, path)
    if md is None:
        return False
    try:
        # a sub-assembly opened silent may come up LIGHTWEIGHT; saving it
        # then yields an empty (~850 B) 3DXML envelope.  Resolve first.
        from .swcom import SW_DOC_ASSEMBLY
        if doc_type_for(path) == SW_DOC_ASSEMBLY:
            try:
                as_iface(md, "IAssemblyDoc") \
                    .ResolveAllLightWeightComponents(True)
            except Exception:
                pass
        return _save_3dxml(md, out)
    except Exception as e:
        if getattr(e, "hresult", None) == _RPC_DISCONNECTED:
            _crash_suspects.update(_recent_opens)
            print(f"  SolidWorks DIED exporting "
                  f"{os.path.basename(path)}; blacklisted for the retry")
            raise
        print(f"  export raised for {os.path.basename(path)}: {e!r}")
        return False
    finally:
        try:
            app.CloseDoc(safe_prop(md, "GetTitle"))
        except Exception:
            pass
