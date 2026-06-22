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
import re
import xml.etree.ElementTree as ET

from .urdf_writer import (
    CMAKELISTS,
    CMAKELISTS_ROS2,
    DISPLAY_LAUNCH_ROS2,
    PACKAGE_XML,
    PACKAGE_XML_ROS2,
    RVIZ_CONFIG_ROS2,
)

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


def _hex_to_rgba(hex_color):
    """``'#RRGGBB'`` / ``'#RRGGBBAA'`` (the leading ``#`` optional) -> a uint8
    ``[R, G, B, A]`` numpy array, or ``None`` for a missing/malformed value."""
    if not hex_color:
        return None
    import numpy as np

    h = str(hex_color).strip().lstrip("#")
    if len(h) == 6:
        h += "ff"
    if len(h) != 8:
        return None
    try:
        return np.array([int(h[i:i + 2], 16) for i in (0, 2, 4, 6)],
                        dtype=np.uint8)
    except ValueError:
        return None


def _apply_uniform_color(mesh, hex_color):
    """Repaint every face of ``mesh`` one solid colour (dropping any original
    texture/material), so a per-link override wins in every output format.  A
    falsy/invalid ``hex_color`` leaves the mesh untouched."""
    rgba = _hex_to_rgba(hex_color)
    if rgba is None:
        return mesh
    import numpy as np
    import trimesh

    mesh.visual = trimesh.visual.ColorVisuals(
        mesh=mesh, face_colors=np.tile(rgba, (len(mesh.faces), 1)))
    return mesh


def _mesh_to_dae_bytes(src, color=None):
    """COLLADA (.dae) bytes in metres, with colours baked into materials.
    ``color`` (``'#RRGGBB'``) overrides the mesh's own colours when given."""
    meshes = _collada_meshes(_apply_uniform_color(_load_mesh_metres(src), color))
    if len(meshes) == 1:
        return meshes[0].export(file_type="dae")
    from trimesh.exchange.dae import export_collada
    return export_collada(meshes)


def _mesh_to_stl_bytes(src, color=None):
    """STL bytes in metres (colour is dropped -- collision geometry needs none,
    so ``color`` is accepted for a uniform converter signature but ignored)."""
    return _load_mesh_metres(src).export(file_type="stl")


def _mesh_to_glb_bytes(src, color=None):
    """GLB bytes in metres, keeping the original material/texture (for
    three.js / skrobot consumers, not RViz).  ``color`` (``'#RRGGBB'``)
    overrides the mesh's own colours when given."""
    return _apply_uniform_color(_load_mesh_metres(src), color).export(
        file_type="glb")


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


def ros_pkg_name(robot_name, pkg_name=None):
    """The ROS package name to emit: an explicit ``pkg_name`` (validated) or the
    default ``<robot_name>_description``.  catkin/ament require a name starting
    with a lowercase letter and containing only lowercase, digits and
    underscores.

    The default is SANITIZED to that form (a SolidWorks assembly like 'Assem1'
    -> 'assem1_description'), since the assembly name often has capitals; an
    EXPLICIT name is validated strictly and rejected if malformed, so a typo
    surfaces instead of silently changing."""
    if not pkg_name:
        base = re.sub(r"[^a-z0-9_]", "_", robot_name.lower()).strip("_")
        if not base or not base[0].isalpha():
            base = "robot_" + base if base else "robot"
        return f"{base}_description"
    pkg = pkg_name.strip()
    if not re.fullmatch(r"[a-z][a-z0-9_]*", pkg):
        raise ValueError(
            f"invalid ROS package name {pkg_name!r}: must start with a "
            "lowercase letter and contain only lowercase letters, digits and "
            "underscores (e.g. 'bambu_a1_description')")
    return pkg


def ros_urdf_stem(pkg, urdf_name=None):
    """The URDF file stem to emit inside the package: an explicit ``urdf_name``
    (validated) or, when blank, the package name itself -- so the package ships
    ``<pkg>/urdf/<pkg>.urdf`` by default instead of leaking the SolidWorks
    assembly name."""
    if not urdf_name:
        return pkg
    stem = urdf_name.strip()
    if stem.lower().endswith(".urdf"):
        stem = stem[:-5]
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]*", stem):
        raise ValueError(
            f"invalid URDF name {urdf_name!r}: use letters, digits, '_', '-' "
            "or '.' and start with a letter or digit (e.g. 'bambu_a1')")
    return stem


def build_ros_description(pkg_dir, robot_name, email="auto@example.com",
                          ctx_fmt=_CTX_FMT, ros_version=1, pkg_name=None,
                          urdf_name=None, colors=None):
    """``pkg_dir`` (a built package) -> ``[(arcname, bytes), ...]`` for a portable
    ROS package, all behind ``package://`` URLs.  The package is named
    ``pkg_name`` if given (validated, see :func:`ros_pkg_name`), else
    ``<robot_name>_description``.  The URDF inside is named ``urdf_name`` if
    given, else the package name (so ``<pkg>/urdf/<pkg>.urdf`` by default).

    ``ctx_fmt`` maps each URDF context to a mesh format; the default emits
    ``<visual>`` as colour ``.dae`` and ``<collision>`` as plain ``.stl`` (the
    usual ROS split).  Pass :data:`GLB_CTX_FMT` for a uniform ``.glb`` package.

    ``colors`` is an optional ``{component link name -> '#RRGGBB'}`` map (the
    mesh basename equals the component link name) that repaints that link's
    ``<visual>`` mesh a solid colour, overriding its CAD colours; collision STL
    is colourless and unaffected.

    ``ros_version`` picks the build system: ``1`` (default) writes a catkin
    ``package.xml`` (format 2) + ``CMakeLists.txt``; ``2`` writes an ament_cmake
    ``package.xml`` (format 3) + ``CMakeLists.txt`` and bundles a
    ``launch/display.launch.py`` + ``rviz/<urdf>.rviz`` so the package runs with
    ``ros2 launch <name> display.launch.py``.

    Reads the on-disk ``urdf/<robot_name>.urdf`` (which already carries the
    editor's applied edits).  A missing/unconvertible mesh aborts the export
    before any entries are emitted, so a half-rewritten package never ships."""
    if ros_version not in (1, 2):
        raise ValueError(f"unsupported ros_version: {ros_version}")
    pkg = ros_pkg_name(robot_name, pkg_name)
    urdf_stem = ros_urdf_stem(pkg, urdf_name)
    urdf_path = os.path.join(pkg_dir, "urdf", robot_name + ".urdf")
    root = ET.parse(urdf_path).getroot()
    colors = colors or {}

    files = []
    done = {}          # (base, fmt) -> arcname  (instances + visual/collision share)
    errors = []

    def _emit(base, src, fmt):
        if (base, fmt) in done:
            return
        try:
            data = _CONVERT[fmt](src, color=colors.get(base))
        except Exception as e:
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
    files.append((f"{pkg}/urdf/{urdf_stem}.urdf", new_urdf.encode("utf-8")))

    if ros_version == 2:
        # the root (first) link is the natural RViz fixed frame
        first_link = root.find("link")
        fixed_frame = (first_link.get("name") if first_link is not None
                       else "base_link") or "base_link"
        files.append((f"{pkg}/package.xml",
                      PACKAGE_XML_ROS2.format(name=pkg, email=email)
                      .encode("utf-8")))
        files.append((f"{pkg}/CMakeLists.txt",
                      CMAKELISTS_ROS2.format(name=pkg).encode("utf-8")))
        files.append((f"{pkg}/launch/display.launch.py",
                      DISPLAY_LAUNCH_ROS2.format(name=pkg, robot=urdf_stem)
                      .encode("utf-8")))
        files.append((f"{pkg}/rviz/{urdf_stem}.rviz",
                      RVIZ_CONFIG_ROS2.format(fixed_frame=fixed_frame)
                      .encode("utf-8")))
    else:
        files.append((f"{pkg}/package.xml",
                      PACKAGE_XML.format(name=pkg, email=email).encode("utf-8")))
        files.append((f"{pkg}/CMakeLists.txt",
                      CMAKELISTS.format(name=pkg).encode("utf-8")))
    return files


def write_ros_description_package(pkg_dir, robot_name, dest_dir,
                                  email="auto@example.com", ros_version=1,
                                  pkg_name=None, urdf_name=None, colors=None):
    """Write the ROS package under ``dest_dir`` and return its directory path.
    The package is named ``pkg_name`` if given, else ``<robot_name>_description``;
    the URDF inside is named ``urdf_name`` if given, else the package name.
    ``ros_version`` (1 = catkin, 2 = ament_cmake) and ``colors`` (per-link
    colour overrides) are passed through to :func:`build_ros_description`."""
    pkg = ros_pkg_name(robot_name, pkg_name)
    files = build_ros_description(pkg_dir, robot_name, email=email,
                                  ros_version=ros_version, pkg_name=pkg,
                                  urdf_name=urdf_name, colors=colors)
    root = os.path.abspath(dest_dir)
    for arc, data in files:
        dst = os.path.join(root, *arc.split("/"))
        os.makedirs(os.path.dirname(dst), exist_ok=True)
        with open(dst, "wb") as f:
            f.write(data)
    return os.path.join(root, pkg)
