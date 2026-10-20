"""Faces textured in SolidWorks keep their image in the glb export.

A SolidWorks 3DXML stores a textured face as its own rep with per-vertex
TextureCoordinates and a material whose TextureImage is an image in the archive;
the flattened mesh the converters use only keeps colours.  These tests build a
minimal archive of that shape (one textured quad, one plain quad) and check that
the reader finds the textured rep and that the glb carries the image + UVs.
"""

import io
import json
import struct
import zipfile

import numpy as np
import trimesh
from PIL import Image

from sw2robot.exporter import pbr, textures

_XSI = 'xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"'


def _rep(positions, faces_strip, uv=None, material="0"):
    tc = ("" if uv is None else
          '<TextureCoordinates dimension="2D" channel="0">'
          + ",".join(f"{u} {v}" for u, v in uv) + "</TextureCoordinates>")
    return (f'<Rep xsi:type="PolygonalRepType"><Faces><Face strips="{faces_strip}"/>'
            f'</Faces><VertexBuffer><Positions>'
            + ",".join(" ".join(str(c) for c in p) for p in positions)
            + f"</Positions>{tc}</VertexBuffer><SurfaceAttributes>"
            f'<MaterialApplication><MaterialId id="urn:3DXML:CATMaterialRef.3dxml#{material}"/>'
            f"</MaterialApplication></SurfaceAttributes></Rep>")


def _archive(path):
    top = [(0, 0, 1), (10, 0, 1), (0, 20, 1), (10, 20, 1)]
    bottom = [(0, 0, 0), (10, 0, 0), (0, 20, 0), (10, 20, 0)]
    rep = (f'<XMLRepresentation {_XSI} xmlns="http://www.3ds.com/xsd/3DXML">'
           f'<Root xsi:type="BagRepType">'
           + _rep(top, "0 1 2 3", uv=[(0, 0), (1, 0), (0, 1), (1, 1)], material="3")
           + _rep(bottom, "0 1 2 3")
           + "</Root></XMLRepresentation>")
    matref = (f'<Model_3dxml {_XSI} xmlns="http://www.3ds.com/xsd/3DXML"><CATMaterialRef>'
              '<MaterialDomain id="4" associatedFile="urn:3DXML:MaterialDef_3.3DRep"/>'
              '<MaterialDomainInstance id="5"><IsAggregatedBy>3</IsAggregatedBy>'
              "<IsInstanceOf>4</IsInstanceOf></MaterialDomainInstance>"
              "</CATMaterialRef></Model_3dxml>")
    matdef = ('<Osm xmlns="http://www.3ds.com/xsd/osm.xsd"><Feature Alias="RenderingFeature">'
              '<Attr Name="TextureImage" Type="external" '
              'Value="urn:3DXML:CATRepImage.3dxml#1"/></Feature></Osm>')
    repimg = ('<Model_3dxml xmlns="http://www.3ds.com/xsd/3DXML"><CATRepImage>'
              '<CATRepresentationImage id="1" associatedFile="urn:3DXML:image_1.png"/>'
              "</CATRepImage></Model_3dxml>")
    png = io.BytesIO()
    Image.new("RGB", (8, 4), (200, 30, 30)).save(png, format="PNG")
    with zipfile.ZipFile(path, "w") as z:
        z.writestr("TessPart_1.3DRep", rep)
        z.writestr("CATMaterialRef.3dxml", matref)
        z.writestr("MaterialDef_3.3DRep", matdef)
        z.writestr("CATRepImage.3dxml", repimg)
        z.writestr("image_1.png", png.getvalue())
    return top, bottom


def test_reader_finds_the_textured_rep_only(tmp_path):
    src = tmp_path / "part.3dxml"
    _archive(src)
    patches = textures.read_3dxml_textures(str(src))
    assert len(patches) == 1
    p = patches[0]
    assert p["vertices"].shape == (4, 3) and p["uv"].shape == (4, 2)
    assert len(p["faces"]) == 2
    assert p["image"].size == (8, 4)


def test_glb_carries_the_texture_and_keeps_every_face(tmp_path):
    src = tmp_path / "part.3dxml"
    top, bottom = _archive(src)
    # the flattened mesh a loader would hand over: both quads, face-coloured, metres
    verts = np.array(top + bottom, float) * 0.001
    faces = np.array([[0, 1, 2], [2, 1, 3], [4, 5, 6], [6, 5, 7]])
    mesh = trimesh.Trimesh(verts, faces, process=False,
                           face_colors=np.tile([255, 255, 255, 255], (4, 1)))
    rest, pieces = textures.split_textured(mesh, textures.read_3dxml_textures(str(src)))
    assert len(rest.faces) == 2 and len(pieces) == 1

    glb = pbr.glb_with_materials(rest, 1.0, 0.3, name="steel", textured=pieces)
    n = struct.unpack_from("<I", glb, 12)[0]
    tree = json.loads(glb[20:20 + n])
    assert len(tree["images"]) == 1
    textured = [m for m in tree["materials"]
                if "baseColorTexture" in m["pbrMetallicRoughness"]]
    assert len(textured) == 1
    assert textured[0]["pbrMetallicRoughness"]["metallicFactor"] == 1.0
    loaded = trimesh.load(io.BytesIO(glb), file_type="glb")
    assert sum(len(g.faces) for g in loaded.geometry.values()) == 4


def test_unmatched_texture_keeps_the_colour(tmp_path):
    src = tmp_path / "part.3dxml"
    _archive(src)
    # a mesh with none of the textured triangles: nothing is cut, nothing duplicated
    mesh = trimesh.creation.box()
    rest, pieces = textures.split_textured(mesh, textures.read_3dxml_textures(str(src)))
    assert rest is mesh and pieces == []
