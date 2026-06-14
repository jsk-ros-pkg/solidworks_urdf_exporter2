"""Turn a built package into a portable ROS *description* package.

The in-house URDF (``urdf/<name>.urdf``) references meshes by a path relative to
the URDF (``../meshes/<link>.3dxml`` / ``.glb``).  That loads in our viewer and in
skrobot, but it is NOT a portable ROS package: RViz/Gazebo cannot read 3DXML/GLB,
and ROS tooling expects ``package://<pkg>/...`` URLs.

This module produces a SEPARATE file set -- it never touches the working package --
named ``<robot_name>_description`` with:

* ``<visual>`` meshes as COLLADA ``.dae`` (metres, colours preserved -- unlike STL),
* ``<collision>`` meshes as plain ``.stl`` (lighter, no colour -- the usual ROS
  collision convention),
* ``package://<robot_name>_description/meshes/<link>.{dae,stl}`` references, and
* a ``package.xml`` / ``CMakeLists.txt`` whose package name is that same
  ``<robot_name>_description``.

``build_ros_description`` returns ``[(arcname, bytes), ...]`` so the web server can
zip it in memory and the CLI can write it to disk.
"""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET

from .urdf_writer import CMAKELISTS, PACKAGE_XML

# extensions we convert; anything else (already .dae/.stl, abs URLs) is left alone
_CONVERTIBLE = (".3dxml", ".glb")


def _load_mesh_metres(src):
    """Load a ``.3dxml`` (mm) or ``.glb`` (m) mesh, flatten any scene to a single
    geometry, and return it pinned to metres."""
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
    return mesh


def _mesh_to_dae_bytes(src):
    """COLLADA (.dae) bytes in metres, with colours baked into materials."""
    meshes = _collada_meshes(_load_mesh_metres(src))
    if len(meshes) == 1:
        return meshes[0].export(file_type="dae")
    from trimesh.exchange.dae import export_collada
    return export_collada(meshes)


def _mesh_to_stl_bytes(src):
    """STL bytes in metres (colour is dropped -- collision geometry needs none)."""
    return _load_mesh_metres(src).export(file_type="stl")


def _mesh_to_glb_bytes(src):
    """GLB bytes in metres, keeping the original material/texture (for
    three.js / skrobot consumers, not RViz)."""
    return _load_mesh_metres(src).export(file_type="glb")


_CONVERT = {"dae": _mesh_to_dae_bytes, "stl": _mesh_to_stl_bytes,
            "glb": _mesh_to_glb_bytes}

# default: <visual> as colour COLLADA, <collision> as plain STL (the ROS split)
_CTX_FMT = (("visual", "dae"), ("collision", "stl"))
# a uniform GLB variant (both contexts) for native-mesh / three.js consumers
GLB_CTX_FMT = (("visual", "glb"), ("collision", "glb"))


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
    """``(base, source_path, ext)`` for a URDF mesh ref, or ``(None, ...)`` when it
    is not one of our convertible meshes; ``source_path`` is None if the file is
    missing."""
    if not ref or "://" in ref:
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


def build_ros_description(pkg_dir, robot_name, email="auto@example.com",
                          ctx_fmt=_CTX_FMT):
    """``pkg_dir`` (a built package) -> ``[(arcname, bytes), ...]`` for a portable
    ``<robot_name>_description`` ROS package, all behind ``package://`` URLs.

    ``ctx_fmt`` maps each URDF context to a mesh format; the default emits
    ``<visual>`` as colour ``.dae`` and ``<collision>`` as plain ``.stl`` (the
    usual ROS split).  Pass :data:`GLB_CTX_FMT` for a uniform ``.glb`` package.

    Reads the on-disk ``urdf/<robot_name>.urdf`` (which already carries the
    editor's applied edits).  A missing/unconvertible mesh aborts the export
    before any entries are emitted, so a half-rewritten package never ships."""
    pkg = f"{robot_name}_description"
    urdf_path = os.path.join(pkg_dir, "urdf", robot_name + ".urdf")
    root = ET.parse(urdf_path).getroot()

    files = []
    done = {}          # (base, fmt) -> arcname  (instances + visual/collision share)
    errors = []

    def _emit(base, src, fmt):
        if (base, fmt) in done:
            return
        try:
            data = _CONVERT[fmt](src)
        except Exception as e:             # noqa: BLE001 -- report, don't die
            errors.append(f"{base}: {fmt} convert failed ({e!r})")
            return
        arc = f"{pkg}/meshes/{base}.{fmt}"
        files.append((arc, data))
        done[(base, fmt)] = arc

    for link in root.findall("link"):
        for ctx, fmt in ctx_fmt:
            for block in link.findall(ctx):
                for mesh in block.iter("mesh"):
                    base, src, _ext = _resolve_mesh(pkg_dir, mesh.get("filename"))
                    if base is None:
                        continue           # not a convertible ref -- leave as-is
                    if src is None:
                        errors.append(f"no source mesh for '{base}'")
                        continue
                    _emit(base, src, fmt)
                    mesh.set("filename",
                             f"package://{pkg}/meshes/{base}.{fmt}")

    if errors:
        raise RuntimeError("ROS description export failed: " + "; ".join(errors))

    new_urdf = '<?xml version="1.0"?>\n' + ET.tostring(root, encoding="unicode")
    if not new_urdf.endswith("\n"):
        new_urdf += "\n"
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
