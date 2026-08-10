"""Post-build validation: warn when a RESOLVED component's geometry is silently
missing from the final URDF.

The classic cause is a sub-assembly kept as ONE composed mesh whose 3DXML export
ran in a configuration that suppresses some children -- those children have an
exported per-part mesh AND a world in graph.json, yet no geometry of theirs ends
up in the render.  Nothing warned about that before; this does.

Geometry-based so it catches every mechanism, not just config suppression: it
places each component's own exported mesh at the component's world and checks
whether the assembled URDF actually has surface there.
"""
import os
import xml.etree.ElementTree as ET

import numpy as np


def _load_glb_verts(path):
    try:
        import trimesh
    except Exception:
        return None
    stem = os.path.splitext(path)[0]
    for cand in (path, stem + ".3dxml.glb", stem + ".glb", path + ".glb"):
        if os.path.exists(cand):
            try:
                m = trimesh.load(cand, force="mesh")
                if len(m.vertices):
                    return np.asarray(m.vertices, float)
            except Exception:
                pass
    return None


def _origin_mat(el):
    from .geometry import urdf_origin_matrix
    return urdf_origin_matrix(el)


def warn_dropped_geometry(pkg_dir, urdf_path, graph, tol_mm=3.0, min_frac=0.15):
    """Print a WARNING per component whose geometry is absent from the URDF.

    Returns the list of dropped component names (empty if all present)."""
    try:
        from scipy.spatial import cKDTree
    except Exception:
        return []                       # scipy/trimesh optional -- skip silently
    meshes_dir = os.path.join(pkg_dir, "meshes")

    # --- assembled scene point cloud, in the URDF root frame ---------------
    try:
        from skrobot.models.urdf import RobotModelFromURDF
    except Exception:
        return []
    # strip visuals so skrobot loads even with .3dxml refs, then FK link frames
    tree = ET.parse(urdf_path)
    root = tree.getroot()
    link_mesh = {}                      # link name -> [(glb_verts, visual_origin)]
    for link in root.findall("link"):
        items = []
        for vis in link.findall("visual"):
            me = vis.find("geometry/mesh")
            if me is None:
                continue
            fn = os.path.basename(me.get("filename") or "")
            verts = _load_glb_verts(os.path.join(meshes_dir, fn))
            if verts is not None:
                items.append((verts, _origin_mat(vis.find("origin"))))
        if items:
            link_mesh[link.get("name")] = items
    for link in root.findall("link"):
        for tag in ("visual", "collision"):
            for e in link.findall(tag):
                link.remove(e)
    nm = urdf_path + ".novis.urdf"
    ET.ElementTree(root).write(nm)
    try:
        rb = RobotModelFromURDF(urdf_file=nm)
    finally:
        try:
            os.remove(nm)
        except OSError:
            pass
    L = {l.name: l for l in rb.link_list}
    pts = []
    for ln, items in link_mesh.items():
        if ln not in L:
            continue
        W = L[ln].worldcoords().T()
        for verts, vo in items:
            T = W @ vo
            pts.append((T[:3, :3] @ verts.T).T + T[:3, 3])
    if not pts:
        return []
    scene = np.vstack(pts)
    kd = cKDTree(scene)

    # The URDF is rooted at base_link (skrobot frame); graph worlds are in the
    # SolidWorks world frame.  Recover the align transform from ONE present link
    # so component placements below land in the URDF scene frame.
    import re as _re
    def _san(s):
        return _re.sub(r"[^a-z0-9]", "", s.lower())
    dwld = graph.get("deep_worlds") or {}
    dw_by_leaf = {_san(k.split("/")[-1]): np.array(v, float).reshape(4, 4)
                  for k, v in dwld.items()}
    T_align = None
    for ln, items in link_mesh.items():
        if ln not in L:
            continue
        key = None
        for lk in dw_by_leaf:
            if len(lk) >= 6 and (_san(ln).endswith(lk) or lk.endswith(_san(ln))):
                key = lk
                break
        if key is None:
            continue
        # scene pose of this link's first mesh vs its graph world
        W = L[ln].worldcoords().T() @ items[0][1]
        T_align = W @ np.linalg.inv(dw_by_leaf[key])
        break
    if T_align is None:
        T_align = np.eye(4)

    # --- every LEAF component with an exported mesh + its world -------------
    # sub-assembly roots are represented by their composed mesh OR their
    # expanded children, so skip them (checking their composed mesh at the SA
    # world false-positives whenever the SA was expanded).
    mesh_of = {}
    for sa in (graph.get("subassemblies") or {}).values():
        for c in sa.get("components", []):
            if c.get("mesh_file") and not c.get("is_subassembly"):
                mesh_of[c["name"]] = c["mesh_file"]
    for c in graph.get("components", []):
        if c.get("mesh_file") and not c.get("is_subassembly"):
            mesh_of[c["name"]] = c["mesh_file"]

    rng = np.random.RandomState(0)
    dropped = []
    for key, wl in (graph.get("deep_worlds") or {}).items():
        nm2 = key.split("/")[-1]
        mf = mesh_of.get(nm2)
        if not mf:
            continue
        verts = _load_glb_verts(os.path.join(pkg_dir, mf.replace("\\", "/")))
        if verts is None:
            continue
        W = T_align @ np.array(wl, float).reshape(4, 4)
        sample = verts[rng.choice(len(verts), min(60, len(verts)), replace=False)]
        wp = (W[:3, :3] @ sample.T).T + W[:3, 3]
        d, _ = kd.query(wp)
        frac = float((d < tol_mm / 1000.0).mean())
        if frac < min_frac:
            dropped.append(nm2)
    # de-dup, keep order
    seen = set()
    uniq = [d for d in dropped if not (d in seen or seen.add(d))]
    if uniq:
        print(f"      WARNING: {len(uniq)} component(s) have geometry that is "
              f"MISSING from the URDF (likely hidden in a non-expanded "
              f"sub-assembly whose composed mesh omits them -- add them to "
              f"`expand:`):")
        for nm2 in uniq:
            print(f"        - {nm2}")
    return uniq
