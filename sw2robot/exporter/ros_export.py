"""Turn a built package into a portable ROS *description* package.

The in-house URDF (``urdf/<name>.urdf``) references meshes by a path relative to
the URDF (``../meshes/<link>.3dxml`` / ``.glb``).  That loads in our viewer and in
skrobot, but it is NOT a portable ROS package: RViz/Gazebo cannot read 3DXML/GLB,
and ROS tooling expects ``package://<pkg>/...`` URLs.

This module produces a SEPARATE file set -- it never touches the working package --
named ``<robot_name>_description`` with:

* COLLADA ``.dae`` meshes (metres, colours preserved -- unlike STL), and
* ``package://<robot_name>_description/meshes/<link>.dae`` references in the URDF,
* a ``package.xml`` / ``CMakeLists.txt`` whose package name is that same
  ``<robot_name>_description``.

``build_ros_description`` returns ``[(arcname, bytes), ...]`` so the web server can
zip it in memory and the CLI can write it to disk.
"""

from __future__ import annotations

import os
import re

from .urdf_writer import CMAKELISTS, PACKAGE_XML

# mesh references in the URDF: <mesh filename="..."/> (quotes/spacing preserved)
_MESH_RE = re.compile(r'(<mesh\b[^>]*\bfilename\s*=\s*)(["\'])([^"\']+)(\2)')
# extensions we convert; anything else (already .dae/.stl, abs URLs) is left alone
_CONVERTIBLE = (".3dxml", ".glb")


def _mesh_to_dae_bytes(src):
    """Load a ``.3dxml`` (mm) or ``.glb`` (m) mesh and return COLLADA (.dae) bytes
    in metres, flattening any scene to material-coloured geometry."""
    import trimesh

    loaded = trimesh.load(src)
    if isinstance(loaded, trimesh.Scene):
        # to_geometry() is trimesh's current single-mesh flatten; fall back to
        # the older dump() on versions that predate it
        if hasattr(loaded, "to_geometry"):
            mesh = loaded.to_geometry()
        elif hasattr(loaded, "to_mesh"):
            mesh = loaded.to_mesh()
        else:
            mesh = loaded.dump(concatenate=True)
    else:
        mesh = loaded
    if src.lower().endswith(".3dxml"):
        mesh.apply_scale(0.001)            # 3DXML tessellation is in mm
    # the 'mm' units tag survives apply_scale; leaving it makes unit-aware
    # loaders shrink the mesh 1000x, so pin it to metres
    mesh.units = "meter"
    meshes = _collada_meshes(mesh)
    if len(meshes) == 1:
        return meshes[0].export(file_type="dae")
    from trimesh.exchange.dae import export_collada
    return export_collada(meshes)


def _collada_meshes(mesh):
    """Meshes to hand to trimesh's DAE writer, with texture colours baked into
    material-coloured geometry because COLLADA texture export is not supported."""
    import numpy as np
    from trimesh.visual import color as vcolor

    if getattr(mesh.visual, "kind", None) == "texture":
        mesh.visual = mesh.visual.to_color()
    kind = getattr(mesh.visual, "kind", None)
    if kind == "vertex":
        face_colors = vcolor.vertex_to_face_color(mesh.visual.vertex_colors,
                                                  mesh.faces)
    elif kind == "face":
        face_colors = mesh.visual.face_colors
    else:
        return [mesh]

    face_colors = np.asarray(face_colors, dtype=np.uint8)
    if len(face_colors) != len(mesh.faces):
        return [mesh]
    unique, inverse = np.unique(face_colors, axis=0, return_inverse=True)
    if len(unique) <= 1:
        mesh.visual.face_colors = np.tile(unique[0], (len(mesh.faces), 1))
        return [mesh]

    out = []
    for i, rgba in enumerate(unique):
        faces = np.nonzero(inverse == i)[0]
        part = mesh.submesh([faces], append=True, repair=False)
        part.visual.face_colors = np.tile(rgba, (len(part.faces), 1))
        part.units = mesh.units
        out.append(part)
    return out


def _resolve_mesh(pkg_dir, ref):
    """Absolute path of a URDF mesh ref's source file, or None if it is not one
    of our convertible meshes / cannot be found."""
    if "://" in ref:
        return None, None, None
    base, ext = os.path.splitext(os.path.basename(ref))
    if ext.lower() not in _CONVERTIBLE:
        return None, None, None
    meshes = os.path.join(pkg_dir, "meshes")
    src = os.path.join(meshes, base + ext)
    if not os.path.exists(src):
        # the URDF may name one extension while only the other was produced
        # (a sub-assembly composed to .glb, say); accept whichever exists
        for alt_ext in _CONVERTIBLE:
            alt = os.path.join(meshes, base + alt_ext)
            if os.path.exists(alt):
                src = alt
                break
        else:
            return base, None, ext
    return base, src, ext


def build_ros_description(pkg_dir, robot_name, email="auto@example.com"):
    """``pkg_dir`` (a built package) -> ``[(arcname, bytes), ...]`` for a portable
    ``<robot_name>_description`` ROS package (package:// + colour .dae).

    Reads the on-disk ``urdf/<robot_name>.urdf`` (which already carries the
    editor's applied edits), so the export reflects whatever the working package
    currently is.  Meshes that fail to convert abort the export before any
    package/zip entries are emitted, avoiding a half-rewritten ROS package."""
    pkg = f"{robot_name}_description"
    urdf_path = os.path.join(pkg_dir, "urdf", robot_name + ".urdf")
    with open(urdf_path, encoding="utf-8") as f:
        urdf = f.read()

    files = []
    done = {}          # base -> "<pkg>/meshes/<base>.dae"  (instances share a mesh)
    errors = []

    def _repl(match):
        ref = match.group(3)
        base, src, _ext = _resolve_mesh(pkg_dir, ref)
        if base is None:                       # not a convertible mesh ref
            return match.group(0)
        if base not in done:
            if src is None:
                errors.append(f"no source mesh for '{base}'")
                return match.group(0)
            try:
                data = _mesh_to_dae_bytes(src)
            except Exception as e:             # noqa: BLE001 -- report, don't die
                errors.append(f"{base}: dae convert failed ({e!r})")
                return match.group(0)
            arc = f"{pkg}/meshes/{base}.dae"
            files.append((arc, data))
            done[base] = arc
        return (f'{match.group(1)}{match.group(2)}'
                f'package://{pkg}/meshes/{base}.dae{match.group(2)}')

    new_urdf = _MESH_RE.sub(_repl, urdf)
    if errors:
        raise RuntimeError("ROS description export failed: " + "; ".join(errors))
    files.append((f"{pkg}/urdf/{robot_name}.urdf", new_urdf.encode("utf-8")))
    files.append((f"{pkg}/package.xml",
                  PACKAGE_XML.format(name=pkg, email=email).encode("utf-8")))
    files.append((f"{pkg}/CMakeLists.txt",
                  CMAKELISTS.format(name=pkg).encode("utf-8")))
    return files


def write_ros_description_package(pkg_dir, robot_name, dest_dir,
                                  email="auto@example.com"):
    """Write the ``<robot_name>_description`` package under ``dest_dir`` and return
    its directory path."""
    files = build_ros_description(pkg_dir, robot_name, email=email)
    root = os.path.abspath(dest_dir)
    for arc, data in files:
        dst = os.path.join(root, *arc.split("/"))
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        with open(dst, "wb") as f:
            f.write(data)
    return os.path.join(root, f"{robot_name}_description")
