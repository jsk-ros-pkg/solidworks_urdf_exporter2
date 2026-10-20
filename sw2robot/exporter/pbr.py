"""Metallic / roughness materials for the glTF (``.glb``) export.

A ``.glb`` with no material is, by the glTF spec, a fully rough metal
(``metallicFactor 1, roughnessFactor 1``): that is what every mesh this
package wrote so far looked like in a PBR renderer (Isaac Sim, three.js,
Blender) -- dull grey whatever the CAD said.  The 3DXML SolidWorks writes
does carry a Phong material, but its specular terms are the appearance
defaults on nearly every part (``SpecularCoef 0.5, SpecularExponent 0.216``
across six robots' exports), so they cannot tell chrome from ABS.

What can: the SolidWorks **appearance** applied to the part (a ``.p2m`` under
``data/graphics/materials/<family>/<kind>/<finish>.p2m``, read at extract
time -- see ``model._part_appearance``) and, failing that, the SolidWorks
**material** name (``6061 Alloy``, ``ABS PC``, ...).  Both are keyword
matched here into a small preset table; an explicit per-link override
(``materials:`` in joints.yaml) wins over either.  The result is written as
one glTF material per face colour (``glb_with_materials``), so a two-tone
part keeps its colours and gains a finish.

The presets are ordinary look-dev starting values, not measurements.
"""

from __future__ import annotations

import re

#: ``name -> (metallicFactor, roughnessFactor)``.
PRESETS = {
    "aluminum_satin": (1.0, 0.45),
    "aluminum_brushed": (1.0, 0.35),
    "aluminum_polished": (1.0, 0.15),
    "aluminum_cast": (1.0, 0.65),
    "steel_satin": (1.0, 0.4),
    "steel_polished": (1.0, 0.15),
    "stainless": (1.0, 0.3),
    "chrome": (1.0, 0.08),
    "brass": (1.0, 0.3),
    "copper": (1.0, 0.3),
    "titanium": (1.0, 0.4),
    "gold": (1.0, 0.2),
    "plastic_matte": (0.0, 0.6),
    "plastic_glossy": (0.0, 0.25),
    "rubber": (0.0, 0.85),
    "paint_matte": (0.0, 0.7),
    "paint_gloss": (0.0, 0.2),
    "glass": (0.0, 0.05),
    "wood": (0.0, 0.6),
    "default": (0.0, 0.6),
}

# family keyword -> (preset for a plain/satin finish, preset when polished).
# The Japanese names are SolidWorks' own material library as a Japanese-UI
# install reports it ("6061 合金", "炭素鋼(普通)"): the appearance paths stay
# English, the material names do not.
_FAMILIES = (
    (("chrome", "chromium", "mirror", "クロム"), "chrome", "chrome"),
    (("stainless", "ステンレス"), "stainless", "steel_polished"),
    (("aluminum", "aluminium", "alloy", "anodi", "アルミ", "合金"),
     "aluminum_satin", "aluminum_polished"),
    (("steel", "iron", "aisi", "carbon", "鋼", "鉄"), "steel_satin",
     "steel_polished"),
    (("brass", "bronze", "真鍮", "黄銅", "青銅"), "brass", "brass"),
    (("copper", "銅"), "copper", "copper"),
    (("titanium", "チタン"), "titanium", "titanium"),
    (("gold", "silver", "nickel", "zinc", "tin"), "gold", "gold"),
    (("rubber", "silicone", "epdm", "urethane", "neoprene", "ゴム",
      "シリコン"), "rubber", "rubber"),
    (("glass", "acrylic", "polycarbonate", "pmma"), "glass", "glass"),
    (("wood", "oak", "pine", "mdf", "plywood"), "wood", "wood"),
    (("paint", "powder coat", "enamel"), "paint_matte", "paint_gloss"),
    (("plastic", "abs", "pla", "petg", "nylon", "pa6", "pa12", "pom",
      "delrin", "acetal", "pvc", "pp", "pe ", "polyethylene",
      "polypropylene", "resin", "pet", "ptfe", "teflon", "fiber",
      "fibre", "樹脂", "プラスチック", "ナイロン"), "plastic_matte",
     "plastic_glossy"),
)
_POLISHED = ("polish", "high gloss", "glossy", "shiny", "mirror", "bright",
             "burnished")
_ROUGH = ("matte", "matt", "cast", "sand", "rough", "textured", "blast",
          "low gloss", "soft touch")
_BRUSHED = ("brush", "machined", "turned", "circular")


def _match(text, words):
    t = " " + re.sub(r"[_\-/\\.]+", " ", text.lower()) + " "
    return any(w in t for w in words)


def from_appearance(appearance):
    """Preset name for a SolidWorks appearance path, or None.

    ``appearance`` is the ``.p2m`` path (or the part of it under the
    ``materials`` library folder, which is how the extract records it).
    ``color.p2m`` -- the appearance SolidWorks assigns when only a colour was
    picked -- says nothing about the finish and yields None.
    """
    if not appearance:
        return None
    text = str(appearance)
    base = re.split(r"[/\\]", text)[-1].lower()
    if base in ("color.p2m", "color"):
        return None
    return _guess(text)


def from_material(material):
    """Preset name for a SolidWorks material name (``6061 Alloy``), or None."""
    if not material:
        return None
    return _guess(str(material))


def _guess(text):
    for words, plain, polished in _FAMILIES:
        if _match(text, words):
            if _match(text, _POLISHED):
                return polished
            if plain.startswith("aluminum") and _match(text, _ROUGH):
                return "aluminum_cast"
            if plain.startswith("aluminum") and _match(text, _BRUSHED):
                return "aluminum_brushed"
            if plain == "plastic_matte" and _match(text, _POLISHED):
                return "plastic_glossy"
            return plain
    return None


def resolve(appearance=None, material=None, override=None):
    """``{"metallic", "roughness", "preset", "source"}`` for one link.

    Priority: ``override`` (a ``{"metallic":, "roughness":}`` dict, or a
    preset name, or ``{"preset": name}``) > the appearance > the material
    name > ``default``.  A partial override fills the rest from the next
    source down.
    """
    chosen = None
    source = "default"
    if override is not None:
        if isinstance(override, str):
            override = {"preset": override}
        if not isinstance(override, dict):
            raise ValueError("a materials: entry must be a preset name or a "
                             f"{{metallic, roughness}} mapping, not {override!r}")
        preset = override.get("preset")
        if preset is not None and preset not in PRESETS:
            raise ValueError(f"unknown material preset {preset!r} "
                             f"(known: {', '.join(sorted(PRESETS))})")
        chosen = preset
        if chosen:
            source = "override"
    if chosen is None:
        chosen = from_appearance(appearance)
        if chosen:
            source = "appearance"
    if chosen is None:
        chosen = from_material(material)
        if chosen:
            source = "material"
    if chosen is None:
        chosen = "default"
    metallic, roughness = PRESETS[chosen]
    if isinstance(override, dict):
        if override.get("metallic") is not None:
            metallic = float(override["metallic"])
            source = "override"
        if override.get("roughness") is not None:
            roughness = float(override["roughness"])
            source = "override"
    for name, value in (("metallic", metallic), ("roughness", roughness)):
        if not 0.0 <= value <= 1.0:
            raise ValueError(f"{name} must be within 0..1, got {value:g}")
    return {"metallic": round(metallic, 4), "roughness": round(roughness, 4),
            "preset": chosen, "source": source}


_PROVENANCE = re.compile(r'(\w+)="([^"]*)"')


def provenance(link):
    """The ``<!-- sw2robot key="value" ... -->`` fields of a URDF ``<link>``
    element (parsed with comments kept), as a dict; ``{}`` when absent."""
    import xml.etree.ElementTree as ET
    out = {}
    for child in link:
        if child.tag is not ET.Comment:
            continue
        text = (child.text or "").strip()
        if text.startswith("sw2robot "):
            out.update(dict(_PROVENANCE.findall(text)))
    return out


# ------------------------------------------------------------------ GLB write
def glb_with_materials(mesh, metallic, roughness, name=None):
    """GLB bytes of ``mesh`` split into one primitive per face colour, each
    bound to a glTF PBR material: ``baseColorFactor`` = that colour, plus the
    given ``metallic`` / ``roughness`` (and ``name``, so a consumer can dedupe).

    Why split rather than keep ``COLOR_0``: trimesh exports vertex colours and
    a material as alternatives (``ColorVisuals`` writes ``COLOR_0`` and no
    material; ``TextureVisuals`` writes the material and no colours) and its
    reader drops ``COLOR_0`` once a material is present, so scikit-robot's
    URDF->USD and any trimesh-based loader would see a white part.  Colours
    carried as ``baseColorFactor`` survive trimesh, the Omniverse asset
    converter (Isaac Sim) and three.js alike.  A one-colour part (the usual
    case) is a single primitive.
    """
    import numpy as np
    import trimesh
    from trimesh.visual import TextureVisuals
    from trimesh.visual.material import PBRMaterial

    if mesh.visual.kind in ("vertex", "face"):
        face_colors = np.asarray(mesh.visual.face_colors, dtype=np.uint8)
    else:
        face_colors = np.tile(np.array([191, 191, 191, 255], np.uint8),
                              (len(mesh.faces), 1))
    unique, inverse = np.unique(face_colors, axis=0, return_inverse=True)
    inverse = np.asarray(inverse).ravel()
    scene = trimesh.Scene()
    for i, rgba in enumerate(unique):
        faces = np.nonzero(inverse == i)[0]
        part = (mesh.submesh([faces], append=True, repair=False)
                if len(unique) > 1 else mesh.copy())
        part.units = mesh.units
        part.visual = TextureVisuals(material=PBRMaterial(
            baseColorFactor=[int(c) for c in rgba],
            metallicFactor=float(metallic), roughnessFactor=float(roughness),
            name=str(name) if name else None))
        label = f"{name or 'material'}_{i}"
        scene.add_geometry(part, node_name=label, geom_name=label)
    return scene.export(file_type="glb")


def glb_pbr(glb):
    """``(metallic, roughness, name)`` of the first material in ``glb``, or
    None when it has none -- the read-back for tests and verification."""
    import json
    import struct
    json_len, _t = struct.unpack_from("<II", glb, 12)
    tree = json.loads(glb[20:20 + json_len])
    mats = tree.get("materials") or []
    if not mats:
        return None
    p = mats[0].get("pbrMetallicRoughness") or {}
    return (p.get("metallicFactor"), p.get("roughnessFactor"), mats[0].get("name"))


def glb_materials(glb):
    """Every material of ``glb`` as ``(name, baseColorFactor, metallic, roughness)``."""
    import json
    import struct
    json_len, _t = struct.unpack_from("<II", glb, 12)
    tree = json.loads(glb[20:20 + json_len])
    out = []
    for m in tree.get("materials") or []:
        p = m.get("pbrMetallicRoughness") or {}
        out.append((m.get("name"), p.get("baseColorFactor"), p.get("metallicFactor"),
                    p.get("roughnessFactor")))
    return out
