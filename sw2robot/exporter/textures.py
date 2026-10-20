"""Image textures of a SolidWorks ``.3dxml``, for the glTF (``.glb``) export.

A face that carries a texture appearance in SolidWorks is written into the
3DXML as its own tessellated rep with per-vertex ``TextureCoordinates`` and a
material whose ``TextureImage`` points at an image stored in the archive.
trimesh's 3DXML loader keeps one image per part at most and, once the reps of a
part are flattened into one mesh (as every converter here does), only face
colours survive -- so a textured face came out as a plain white material.

:func:`read_3dxml_textures` reads those textured reps straight from the archive
(their vertices in the 3DXML's millimetres, triangles, UVs and image), and
:func:`split_textured` cuts the same triangles out of the flattened mesh, so the
glb writer can emit them as textured primitives next to the per-colour ones.

Only reps in the file's own frame are read: a 3DXML whose instances carry a
placement (an assembly-level export) is skipped with a warning, its faces
keeping their colour.  A texture that cannot be resolved (a missing image, an
unreadable material) is skipped the same way; nothing here fails an export.
"""

from __future__ import annotations

import io
import xml.etree.ElementTree as ET
import zipfile

import numpy as np

_URN = "urn:3DXML:"


def _parse(archive, name):
    try:
        return ET.fromstring(archive.read(name))
    except (KeyError, ET.ParseError):
        return None


def _local(tag):
    return tag.rsplit("}", 1)[-1]


def _images_by_material(archive):
    """``{CATMatReference id: PIL image}`` for every material with a texture."""
    from PIL import Image

    ref = _parse(archive, "CATMaterialRef.3dxml")
    if ref is None:
        return {}
    image_of_domain = {}
    for dom in ref.iter():
        if _local(dom.tag) != "MaterialDomain":
            continue
        rep = _parse(archive, dom.get("associatedFile", "").replace(_URN, ""))
        if rep is None:
            continue
        tex = next((a for a in rep.iter() if _local(a.tag) == "Attr"
                    and a.get("Name") == "TextureImage"), None)
        if tex is None:
            continue
        img_file, _, img_id = tex.get("Value", "").replace(_URN, "").partition("#")
        img_doc = _parse(archive, img_file)
        if img_doc is None:
            continue
        node = next((n for n in img_doc.iter()
                     if _local(n.tag) == "CATRepresentationImage"
                     and n.get("id") == img_id), None)
        if node is None:
            continue
        try:
            data = archive.read(node.get("associatedFile", "").replace(_URN, ""))
            image = Image.open(io.BytesIO(data))
            image.load()
        except (KeyError, OSError) as e:
            print(f"      WARN: texture image for material domain "
                  f"{dom.get('id')} unreadable ({e!r}); keeping its colour")
            continue
        image_of_domain[dom.get("id")] = image
    out = {}
    for inst in ref.iter():
        if _local(inst.tag) != "MaterialDomainInstance":
            continue
        of = next((c.text for c in inst if _local(c.tag) == "IsInstanceOf"), None)
        by = next((c.text for c in inst if _local(c.tag) == "IsAggregatedBy"), None)
        if of in image_of_domain and by:
            out[by] = image_of_domain[of]
    return out


def _has_placed_instances(archive):
    """True when any ``Instance3D`` carries a non-identity ``RelativeMatrix``."""
    for name in archive.namelist():
        if not name.lower().endswith(".3dxml"):
            continue
        doc = _parse(archive, name)
        if doc is None:
            continue
        for m in doc.iter():
            if _local(m.tag) != "RelativeMatrix" or not m.text:
                continue
            v = np.array(m.text.split(), float)
            if v.size == 12 and not np.allclose(
                    v, [1, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0], atol=1e-9):
                return True
    return False


def _triangles(face):
    from trimesh import util

    tris = []
    if face.get("triangles"):
        tris.extend(np.fromstring(face.get("triangles"), sep=" ",
                                  dtype=np.int64).reshape(-1, 3))
    if face.get("strips"):
        strips = [np.fromstring(s, sep=" ", dtype=np.int64)
                  for s in face.get("strips").split(",")]
        tris.extend(util.triangle_strips_to_faces(strips))
    if face.get("fans"):
        fans = [np.fromstring(s, sep=" ", dtype=np.int64)
                for s in face.get("fans").split(",")]
        tris.extend(util.triangle_fans_to_faces(fans))
    return tris


def read_3dxml_textures(path):
    """Textured reps of a SolidWorks ``.3dxml``.

    Returns
    -------
    list of dict
        ``{"vertices": (n, 3) mm, "faces": (m, 3), "uv": (n, 2), "image": PIL
        image}`` per rep that carries ``TextureCoordinates`` and a material with
        a resolvable texture image; ``[]`` when there are none, or when the file
        places its instances (see the module docstring).  UVs are as stored:
        origin at the image's bottom-left, ``v`` up, which is trimesh's
        convention (its glTF writer flips ``v`` itself).
    """
    try:
        archive = zipfile.ZipFile(path)
    except (OSError, zipfile.BadZipFile):
        return []
    with archive:
        images = _images_by_material(archive)
        if not images:
            return []
        if _has_placed_instances(archive):
            print(f"      WARN: {path}: placed instances -- its textures are not "
                  f"carried into the glb (faces keep their colour)")
            return []
        out = []
        for name in archive.namelist():
            if not name.lower().endswith(".3drep"):
                continue
            doc = _parse(archive, name)
            if doc is None:
                continue
            for rep in doc.iter():
                if _local(rep.tag) != "Rep" or not rep.get(
                        "{http://www.w3.org/2001/XMLSchema-instance}type",
                        "").endswith("PolygonalRepType"):
                    continue
                vb = next((c for c in rep if _local(c.tag) == "VertexBuffer"), None)
                if vb is None:
                    continue
                pos = next((c for c in vb if _local(c.tag) == "Positions"), None)
                uvs = next((c for c in vb if _local(c.tag) == "TextureCoordinates"), None)
                mat = next((m for m in rep.iter() if _local(m.tag) == "MaterialId"), None)
                if pos is None or uvs is None or mat is None:
                    continue
                image = images.get(mat.get("id", "").rpartition("#")[2])
                if image is None:
                    continue
                verts = np.fromstring(pos.text.replace(",", " "), sep=" ").reshape(-1, 3)
                uv = np.fromstring(uvs.text.replace(",", " "), sep=" ")
                if uvs.get("dimension") == "1D":
                    uv = np.stack([uv, np.zeros(len(uv))], axis=1)
                else:
                    uv = uv.reshape(-1, 2)
                faces = [t for f in rep.iter() if _local(f.tag) == "Face"
                         for t in _triangles(f)]
                if not faces or len(uv) != len(verts):
                    continue
                out.append({"vertices": verts, "faces": np.asarray(faces, np.int64),
                            "uv": uv, "image": image})
        return out


def _tri_keys(vertices, faces, decimals):
    """One hashable key per triangle: its corner positions, rounded, in sorted
    order (so the winding and the vertex order do not matter)."""
    corners = np.round(np.asarray(vertices, float)[np.asarray(faces)], decimals)
    return [tuple(sorted(map(tuple, tri))) for tri in corners.tolist()]


def split_textured(mesh, patches, scale=0.001, decimals=9):
    """Cut the textured patches' triangles out of ``mesh`` (a flattened copy
    of the same 3DXML, already multiplied by ``scale``).

    Returns ``(rest, textured)``: ``rest`` is ``mesh`` without them (still
    face-coloured), ``textured`` the trimesh pieces with ``TextureVisuals`` (no
    material yet).  A triangle is matched by its three corner positions, so the
    order of reps or vertices does not matter.  If too few of a patch's
    triangles are found (a loader that re-meshed the faces), that patch is left
    to its colour rather than drawn twice.
    """
    import trimesh
    from trimesh.visual import TextureVisuals

    where = {}
    for i, k in enumerate(_tri_keys(mesh.vertices, mesh.faces, decimals)):
        where.setdefault(k, []).append(i)
    drop = set()
    textured = []
    for p in patches:
        verts = np.asarray(p["vertices"], float) * scale
        keys = _tri_keys(verts, p["faces"], decimals)
        hits = [where[k][0] for k in keys if k in where]
        if len(hits) < 0.9 * len(keys):
            print(f"      WARN: a textured face matched {len(hits)}/{len(keys)} "
                  f"triangles of the mesh; keeping its colour")
            continue
        drop.update(hits)
        piece = trimesh.Trimesh(vertices=verts, faces=p["faces"], process=False)
        piece.visual = TextureVisuals(uv=p["uv"], image=p["image"])
        textured.append(piece)
    if not drop:
        return mesh, []
    keep = np.setdiff1d(np.arange(len(mesh.faces)), sorted(drop))
    rest = mesh.submesh([keep], append=True, repair=False) if len(keep) else None
    if rest is not None:
        rest.units = mesh.units
    return rest, textured
