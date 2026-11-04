"""Build the tutorial quadruped in SolidWorks from scratch.

This script does, over COM, exactly what the tutorial walks through by hand
in the SolidWorks GUI: three parts made of one or two extrusions each, then
an assembly where every joint is a *concentric* mate (the axis) plus a
*coincident* mate (the side faces touching).  That pair leaves one rotational
degree of freedom, which is what sw2robot reads back as a revolute joint.

The robot is modelled Z-up, X-forward, Y-left (ROS REP-103), so the URDF
needs no extra rotation::

    body   220 x 120 x 40 mm box, with an axle on each side at each hip
    thigh  100 mm (hole to hole) slot link, 8 mm thick
    shin   100 mm slot link with a hole at the knee and a round foot

Run it on Windows with SolidWorks installed (2024 or later)::

    uv run python examples/quadruped/build_cad.py

It writes into ``examples/quadruped/cad/`` and refuses to overwrite
files that are already there (pass ``--overwrite`` to replace them).
``--screenshots DIR`` saves a still of each part and the assembly;
``--record DIR`` saves a frame after every modelling step and turns them
into one animated GIF per part and one for the assembly.
"""

from __future__ import annotations

import argparse
import math
import os
import types

import pythoncom
import win32com.client

MM = 1e-3

# swconst values used below (from swconst.tlb; spelled out to keep the script
# free of the makepy wrappers).
SW_DEFAULT_TEMPLATE_PART = 8
SW_DEFAULT_TEMPLATE_ASSEMBLY = 9
SW_END_COND_BLIND = 0
SW_END_COND_MID_PLANE = 6
SW_START_SKETCH_PLANE = 0
SW_START_OFFSET = 3
SW_SAVE_AS_CURRENT_VERSION = 0
SW_SAVE_AS_OPTIONS_SILENT = 1
SW_MATE_COINCIDENT = 0
SW_MATE_CONCENTRIC = 1
SW_MATE_ALIGN_CLOSEST = 2

# Dimensions in millimetres.
BODY = {"length": 220.0, "width": 120.0, "height": 40.0}
HIP_X = 85.0             # hips at x = +/-HIP_X
LINK = {"length": 100.0, "width": 24.0, "thickness": 8.0, "hole": 10.0}
# one axle per hip, ending flush with the outer face of the thigh
PIN = {"diameter": 10.0, "length": LINK["thickness"]}
SHIN_WIDTH = 20.0
MATERIAL = ("SOLIDWORKS Materials", "ABS")
# part colours (RGB 0-1), so the links are told apart in screenshots and in
# the exported meshes
COLOR = {"body": (0.42, 0.55, 0.72), "thigh": (0.95, 0.55, 0.15),
         "shin": (0.85, 0.85, 0.82)}

# Insertion order fixes the instance numbers, and so the link names
# sw2robot writes: thigh_1 = front left, thigh_2 = front right,
# thigh_3 = rear left, thigh_4 = rear right (shins likewise).
LEGS = (
    # hip x, side (+1 = left, -1 = right)
    (+HIP_X, +1),
    (+HIP_X, -1),
    (-HIP_X, +1),
    (-HIP_X, -1),
)

# How far (mm) a leg part is inserted from where it belongs when recording,
# so the frames show the mates pulling it in: outward along Y (taken up by
# the coincident mate) and up along Z (taken up by the concentric one).
EXPLODE = {"out": 40.0, "up": 30.0}
# Z-up three-quarter view: SolidWorks' named views assume Y is up, so the
# Front view (looking down -Z here) is pitched, then yawed.
VIEW = {"pitch_deg": -60.0, "yaw_deg": 140.0}
# The assembly frames keep ONE camera, set on the first frame (the body
# alone): fitted to the body, zoomed out by this factor and panned by this
# much (metres, screen x/y) so the finished robot -- legs reaching 212 mm
# below the body -- stays in view without the picture jumping.
ASSEMBLY_CAMERA = {"zoom": 0.6, "pan_m": (0.0, 0.06)}
GIF = {"width": 640, "margin_px": 40, "frame_ms": 700, "last_frame_ms": 2500}


def byref_long():
    return win32com.client.VARIANT(pythoncom.VT_BYREF | pythoncom.VT_I4, 0)


def null_dispatch():
    return win32com.client.VARIANT(pythoncom.VT_DISPATCH, None)


def member(obj, name, *args):
    """Read a member the late-bound API exposes as property OR method.

    Only a bound method is called: a returned COM object is callable too
    (its default member), and calling that fails with "member not found".
    """
    value = getattr(obj, name)
    return value(*args) if isinstance(value, types.MethodType) else value


def snap(record, doc, tag):
    """Record a frame of ``doc`` under ``tag``, when recording."""
    if record is not None:
        record(doc, tag)


def ref_planes(doc):
    """The part's Front / Top / Right planes, whatever the UI language."""
    planes = [f for f in member(doc.FeatureManager, "GetFeatures", True)
              if member(f, "GetTypeName2") == "RefPlane"]
    front, top, right = planes[:3]
    return {"front": front, "top": top, "right": right}


def new_document(sw, template_pref):
    doc = sw.NewDocument(sw.GetUserPreferenceStringValue(template_pref),
                         0, 0, 0)
    if doc is None:
        raise RuntimeError(f"NewDocument failed (template pref {template_pref})")
    return doc


def start_sketch(doc, plane):
    doc.ClearSelection2(True)
    plane.Select2(False, 0)
    doc.SketchManager.InsertSketch(True)
    doc.SketchManager.AddToDB = True   # no snapping to what is on screen


def finish_sketch(doc):
    doc.SketchManager.AddToDB = False
    doc.SketchManager.InsertSketch(True)
    sketch = member(doc, "GetActiveSketch2")
    if sketch is not None:
        raise RuntimeError("sketch is still open")
    return doc.FeatureByPositionReverse(0)


def extrude(doc, sketch_feature, depth_mm, mid_plane=False, flip=False,
            start_offset_mm=0.0):
    """Boss-extrude a closed sketch; inner loops become holes.

    ``flip`` extrudes against the sketch normal.  ``start_offset_mm`` starts
    the extrusion that far from the sketch plane (on the same side), which is
    how the hip axles grow out of the body's side faces while being sketched
    on the Top plane.  (FeatureExtrusion3's second argument, "Flip", is the
    side a CUT removes; the direction is the third, "Dir".)
    """
    doc.ClearSelection2(True)
    sketch_feature.Select2(False, 0)
    feature = doc.FeatureManager.FeatureExtrusion3(
        True, False, flip,
        SW_END_COND_MID_PLANE if mid_plane else SW_END_COND_BLIND, 0,
        depth_mm * MM, 0.0,
        False, False, False, False, 0.0, 0.0,
        False, False, False, False,
        True, True, True,
        SW_START_OFFSET if start_offset_mm else SW_START_SKETCH_PLANE,
        start_offset_mm * MM, flip)
    if feature is None:
        raise RuntimeError(
            f"extrusion of {member(sketch_feature, 'Name')} failed")
    # the consumed sketch stays drawn on top of the solid otherwise
    doc.ClearSelection2(True)
    sketch_feature.Select2(False, 0)
    member(doc, "BlankSketch")
    doc.ClearSelection2(True)
    return feature


def slot(doc, x1, y1, x2, y2, width_mm):
    """A straight slot (two half circles joined by lines), sketch coords mm."""
    r = width_mm / 2.0
    dx, dy = x2 - x1, y2 - y1
    length = math.hypot(dx, dy)
    nx, ny = -dy / length * r, dx / length * r
    sm = doc.SketchManager
    sm.CreateLine((x1 + nx) * MM, (y1 + ny) * MM, 0, (x2 + nx) * MM,
                  (y2 + ny) * MM, 0)
    sm.CreateLine((x2 - nx) * MM, (y2 - ny) * MM, 0, (x1 - nx) * MM,
                  (y1 - ny) * MM, 0)
    # arcs: centre, start, end, direction (+1 counter-clockwise)
    sm.CreateArc(x2 * MM, y2 * MM, 0, (x2 + nx) * MM, (y2 + ny) * MM, 0,
                 (x2 - nx) * MM, (y2 - ny) * MM, 0, -1)
    sm.CreateArc(x1 * MM, y1 * MM, 0, (x1 - nx) * MM, (y1 - ny) * MM, 0,
                 (x1 + nx) * MM, (y1 + ny) * MM, 0, -1)


def circle(doc, x, y, diameter_mm):
    doc.SketchManager.CreateCircleByRadius(x * MM, y * MM, 0,
                                           diameter_mm / 2.0 * MM)


def set_material(doc, rgb):
    """ABS for the mass properties, and a plain colour for the looks."""
    doc.SetMaterialPropertyName2("", MATERIAL[0], MATERIAL[1])
    # r, g, b, ambient, diffuse, specular, shininess, transparency, emission
    values = [*rgb, 1.0, 1.0, 0.3, 0.3, 0.0, 0.0]
    doc.MaterialPropertyValues = win32com.client.VARIANT(
        pythoncom.VT_ARRAY | pythoncom.VT_R8, values)


def build_body(sw, record=None):
    """Box on the Front (XY) plane, then one axle on each side face.

    In the GUI you would sketch the two circles on the side face itself; the
    script sketches them on the Top plane (the XZ plane through the middle of
    the body) and starts the extrusion at the side face instead, which gives
    the same solid without having to work out the face's sketch axes.

    Each side gets its own axle rather than one pin through the whole body:
    sw2robot places a revolute joint's origin at the concentric mate's point,
    and for a single through-pin that point is the same for the left and the
    right leg.
    """
    doc = new_document(sw, SW_DEFAULT_TEMPLATE_PART)
    # coloured from the start, so the frames show the final look
    set_material(doc, COLOR["body"])
    planes = ref_planes(doc)
    start_sketch(doc, planes["front"])
    doc.SketchManager.CreateCenterRectangle(
        0, 0, 0, BODY["length"] / 2 * MM, BODY["width"] / 2 * MM, 0)
    snap(record, doc, "body")
    extrude(doc, finish_sketch(doc), BODY["height"], mid_plane=True)
    snap(record, doc, "body")
    # Top plane: sketch x = model X, sketch y = model -Z; the pins sit at z=0.
    # The Top plane's normal is +Y: flip=False grows the left axles.
    for flip in (False, True):
        start_sketch(doc, planes["top"])
        for x in (+HIP_X, -HIP_X):
            circle(doc, x, 0.0, PIN["diameter"])
        extrude(doc, finish_sketch(doc), PIN["length"], flip=flip,
                start_offset_mm=BODY["width"] / 2)
        snap(record, doc, "body")
    return doc


def build_link(sw, width_mm, knee_hole, rgb, record=None, tag=""):
    """A slot link in the XZ plane, the top hole at the part origin.

    The link hangs down -Z and is extruded ``thickness`` along +Y, so its
    face at y=0 is the one that sits against the parent.
    """
    doc = new_document(sw, SW_DEFAULT_TEMPLATE_PART)
    set_material(doc, rgb)
    planes = ref_planes(doc)
    start_sketch(doc, planes["top"])
    # sketch y = model -Z, so +length in sketch y hangs the link downwards
    slot(doc, 0.0, 0.0, 0.0, LINK["length"], width_mm)
    snap(record, doc, tag)
    circle(doc, 0.0, 0.0, LINK["hole"])
    if knee_hole:
        circle(doc, 0.0, LINK["length"], LINK["hole"])
    snap(record, doc, tag)
    extrude(doc, finish_sketch(doc), LINK["thickness"])
    snap(record, doc, tag)
    return doc


def bbox_mm(doc):
    """World-ish bounding box of a part's solid bodies, mm."""
    lo = [math.inf] * 3
    hi = [-math.inf] * 3
    for body in member(doc, "GetBodies2", 0, True) or []:
        box = member(body, "GetBodyBox")
        for i in range(3):
            lo[i] = min(lo[i], box[i] / MM)
            hi[i] = max(hi[i], box[i + 3] / MM)
    return [round(v, 3) for v in lo], [round(v, 3) for v in hi]


def save_as(doc, path):
    """SaveAs, verified by the file's mtime.

    SaveAs can return without writing (an image over an existing file, a
    target that is open elsewhere), and a file left over from an earlier run
    would pass an existence check.
    """
    before = os.path.getmtime(path) if os.path.exists(path) else None
    errors, warnings = byref_long(), byref_long()
    doc.Extension.SaveAs(path, SW_SAVE_AS_CURRENT_VERSION,
                         SW_SAVE_AS_OPTIONS_SILENT, null_dispatch(), errors,
                         warnings)
    if not os.path.isfile(path) or os.path.getmtime(path) == before:
        raise RuntimeError(
            f"SaveAs wrote nothing to {path} (errors={errors.value})")


# --------------------------------------------------------------------------
# assembly
# --------------------------------------------------------------------------

def place(sw, assembly, path, rotation_z_deg, translation_mm):
    """Insert a part and pre-position it.

    Pre-positioning matters: a mate is solved from the component's current
    pose, so a part left at the origin may be mated onto the wrong side.
    """
    component = assembly.AddComponent5(os.path.normpath(path), 0, "", False,
                                       "", 0.0, 0.0, 0.0)
    if component is None:
        raise RuntimeError(f"AddComponent5 failed for {path}")
    c = math.cos(math.radians(rotation_z_deg))
    s = math.sin(math.radians(rotation_z_deg))
    # IMathTransform layout: 3x3 rotation (columns), translation (m),
    # scale, 3 unused.  The array MUST be a VARIANT or the translation is
    # silently dropped.
    data = [c, s, 0.0, -s, c, 0.0, 0.0, 0.0, 1.0,
            translation_mm[0] * MM, translation_mm[1] * MM,
            translation_mm[2] * MM, 1.0, 0.0, 0.0, 0.0]
    utility = member(sw, "GetMathUtility")
    utility._FlagAsMethod("CreateTransform")
    component.Transform2 = utility.CreateTransform(
        win32com.client.VARIANT(pythoncom.VT_ARRAY | pythoncom.VT_R8, data))
    return component


def find_face(component, description, predicate):
    """The one face of ``component`` whose local geometry matches.

    ``predicate(kind, params)`` gets ``"plane"`` with PlaneParams
    (normal, point) or ``"cylinder"`` with CylinderParams (origin, axis,
    radius), all in the part's own frame and in metres.
    """
    matches = []
    for body in member(component, "GetBodies2", 0) or []:
        for face in member(body, "GetFaces") or []:
            surface = member(face, "GetSurface")
            if member(surface, "IsPlane"):
                # PlaneParams' normal belongs to the SURFACE and ignores the
                # face's sense (both sides of a plate read -Y); IFace2.Normal
                # is the outward one.
                kind = "plane"
                params = (list(member(face, "Normal"))
                          + list(member(surface, "PlaneParams")[3:6]))
            elif member(surface, "IsCylinder"):
                kind, params = "cylinder", member(surface, "CylinderParams")
            else:
                continue
            if predicate(kind, [v / MM for v in params]):
                matches.append(face)
    if not matches:
        raise RuntimeError(
            f"no face on {member(component, 'Name2')}: {description}")
    return matches[0]


def near(a, b, tol=1e-3):
    return abs(a - b) < tol


def plane_y(y_mm, normal_sign):
    """Plane face at local y = y_mm whose normal points along +/-Y."""
    def predicate(kind, p):
        return (kind == "plane" and near(p[1], normal_sign * 1000.0)
                and near(p[4], y_mm))
    return predicate


def hole_at(x_mm, z_mm, diameter_mm=LINK["hole"]):
    """Cylinder of ``diameter_mm`` with its axis along Y through local (x, z).

    The diameter matters: a link's rounded end is a cylinder on the very same
    axis as its hole, and mating that instead would still give the right
    joint while not being the hole-on-axle the tutorial describes.
    """
    def predicate(kind, p):
        return (kind == "cylinder" and near(abs(p[4]), 1000.0)
                and near(p[0], x_mm) and near(p[2], z_mm)
                and near(2 * p[6], diameter_mm))
    return predicate


def add_mate(assembly, mate_type, faces, align):
    select_data = member(member(assembly, "SelectionManager"),
                         "CreateSelectData")
    assembly.ClearSelection2(True)
    select_data.Mark = 1
    for face in faces:
        face.Select4(True, select_data)
    status = byref_long()
    mate = assembly.AddMate5(mate_type, align, False, 0.0, 0.0, 0.0, 0.0,
                             0.0, 0.0, 0.0, 0.0, False, False, 0, status)
    assembly.ClearSelection2(True)
    if mate is None:
        raise RuntimeError(f"AddMate5 failed (status {status.value})")
    return mate


def revolute(assembly, parent, parent_hole, parent_face, child, record=None):
    """Concentric (shared axis) + coincident (faces touching) = 1 DOF.

    Both mates use CLOSEST alignment: the child is already placed where it
    belongs, and forcing ALIGNED would flip a right-hand leg (yawed 180 deg,
    so its hole axis is reversed) back onto the wrong side.
    """
    add_mate(assembly, SW_MATE_CONCENTRIC,
             [find_face(parent, "joint axis", parent_hole),
              find_face(child, "joint axis", hole_at(0.0, 0.0))],
             SW_MATE_ALIGN_CLOSEST)
    snap(record, assembly, "assembly")
    add_mate(assembly, SW_MATE_COINCIDENT,
             [find_face(parent, "contact face", parent_face),
              find_face(child, "contact face", plane_y(0.0, -1))],
             SW_MATE_ALIGN_CLOSEST)
    snap(record, assembly, "assembly")


def build_assembly(sw, paths, record=None):
    """Body fixed at the origin, then each leg: thigh on the hip, shin on it.

    While recording, each leg part is inserted ``EXPLODE`` away from where it
    belongs (same orientation, so CLOSEST alignment still picks the right
    side) and the mates pull it in; otherwise it is placed exactly.
    """
    assembly = new_document(sw, SW_DEFAULT_TEMPLATE_ASSEMBLY)
    sw.ActivateDoc3(member(assembly, "GetTitle"), False, 0, byref_long())
    body = place(sw, assembly, paths["body"], 0.0, (0.0, 0.0, 0.0))
    body.Select4(False, null_dispatch(), False)
    member(assembly, "FixComponent")
    snap(record, assembly, "assembly")
    out = EXPLODE["out"] if record is not None else 0.0
    up = EXPLODE["up"] if record is not None else 0.0
    half_width = BODY["width"] / 2
    t = LINK["thickness"]
    for x, side in LEGS:
        yaw = 0.0 if side > 0 else 180.0
        thigh = place(sw, assembly, paths["thigh"], yaw,
                      (x, side * (half_width + out), up))
        snap(record, assembly, "assembly")
        # hip: thigh's top hole on the body's axle, against the body side
        revolute(assembly, body, hole_at(x, 0.0),
                 plane_y(side * half_width, side), thigh, record)
        shin = place(sw, assembly, paths["shin"], yaw,
                     (x, side * (half_width + t + out), up - LINK["length"]))
        snap(record, assembly, "assembly")
        # knee: shin's top hole on the thigh's lower hole, outboard of it
        revolute(assembly, thigh, hole_at(0.0, -LINK["length"]),
                 plane_y(t, +1), shin, record)
    member(assembly, "EditRebuild3")
    return assembly


# --------------------------------------------------------------------------
# images
# --------------------------------------------------------------------------

def orient_view(sw, doc):
    """Make ``doc`` active and turn it to the Z-up three-quarter view."""
    sw.ActivateDoc3(member(doc, "GetTitle"), False, 0, byref_long())
    doc.ShowNamedView2("", 1)
    view = member(doc, "ActiveView")
    view.RotateAboutCenter(math.radians(VIEW["pitch_deg"]), 0.0)
    view.RotateAboutCenter(0.0, math.radians(VIEW["yaw_deg"]))
    member(doc, "ViewZoomtofit2")
    member(doc, "GraphicsRedraw2")


def screenshot(sw, doc, path):
    """Save the Z-up three-quarter view of ``doc`` as an image."""
    orient_view(sw, doc)
    # an image SaveAs never replaces an existing file
    if os.path.exists(path):
        os.remove(path)
    save_as(doc, path)


def make_recorder(sw, frames_dir):
    """A ``record(doc, tag)`` callback saving numbered frames per tag.

    The frame paths collect in ``record.frames`` ({tag: [path, ...]}).  A
    part's view is re-oriented and re-fitted for every frame (SolidWorks may
    turn the view normal to a sketch plane when a sketch opens).  The
    assembly keeps the camera of its first frame, see ``ASSEMBLY_CAMERA``.
    """
    os.makedirs(frames_dir, exist_ok=True)
    frames = {}

    def record(doc, tag):
        paths = frames.setdefault(tag, [])
        path = os.path.join(frames_dir, f"{tag}_{len(paths):03d}.png")
        if tag != "assembly":
            screenshot(sw, doc, path)
        else:
            if not paths:
                orient_view(sw, doc)
                view = member(doc, "ActiveView")
                view.ZoomByFactor(ASSEMBLY_CAMERA["zoom"])
                view.TranslateBy(*ASSEMBLY_CAMERA["pan_m"])
            member(doc, "GraphicsRedraw2")
            save_as(doc, path)
        paths.append(path)

    record.frames = frames
    return record


def write_gif(frame_paths, path):
    """Stitch PNG frames into a looping GIF, holding the last frame.

    Every frame is cropped to the same box: the union, over all frames, of
    the pixels that differ from the background, plus a margin.  SolidWorks'
    zoom-to-fit leaves a wide empty border, and one box for all frames keeps
    the picture from jumping.
    """
    from PIL import Image, ImageChops

    frames = []
    for frame in frame_paths:
        with Image.open(frame) as im:
            frames.append(im.convert("RGB"))
    background = Image.new("RGB", frames[0].size, frames[0].getpixel((2, 2)))
    boxes = [ImageChops.difference(f, background).convert("L")
             .point(lambda v: 255 if v > 24 else 0).getbbox() for f in frames]
    boxes = [b for b in boxes if b]
    margin = GIF["margin_px"]
    box = (max(min(b[0] for b in boxes) - margin, 0),
           max(min(b[1] for b in boxes) - margin, 0),
           min(max(b[2] for b in boxes) + margin, frames[0].width),
           min(max(b[3] for b in boxes) + margin, frames[0].height))
    scale = GIF["width"] / (box[2] - box[0])
    size = (GIF["width"], round((box[3] - box[1]) * scale))
    images = [f.crop(box).resize(size, Image.LANCZOS) for f in frames]
    durations = [GIF["frame_ms"]] * (len(images) - 1) + [GIF["last_frame_ms"]]
    images[0].save(path, save_all=True, append_images=images[1:],
                   duration=durations, loop=0, optimize=True)


def main(sw=None, out_dir=None, overwrite=False, screenshots=None,
         record_dir=None):
    if sw is None:
        sw = win32com.client.Dispatch("SldWorks.Application")
        sw.Visible = True
    out_dir = os.path.abspath(out_dir or os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "cad"))
    paths = {name: os.path.join(out_dir, name + ".SLDPRT")
             for name in ("body", "thigh", "shin")}
    paths["assembly"] = os.path.join(out_dir, "quadruped.SLDASM")
    existing = [p for p in paths.values() if os.path.exists(p)]
    if existing and not overwrite:
        raise SystemExit(
            f"refusing to overwrite: {', '.join(existing)} (use --overwrite)")
    os.makedirs(out_dir, exist_ok=True)
    # SolidWorks' SaveAs writes nothing for a relative path
    screenshots = os.path.abspath(screenshots) if screenshots else None
    record_dir = os.path.abspath(record_dir) if record_dir else None
    record = (make_recorder(sw, os.path.join(record_dir, "frames"))
              if record_dir else None)

    built = {"body": build_body(sw, record),
             "thigh": build_link(sw, LINK["width"], knee_hole=True,
                                 rgb=COLOR["thigh"], record=record,
                                 tag="thigh"),
             "shin": build_link(sw, SHIN_WIDTH, knee_hole=False,
                                rgb=COLOR["shin"], record=record, tag="shin")}
    for name, doc in built.items():
        save_as(doc, paths[name])
    assembly = build_assembly(sw, paths, record)
    save_as(assembly, paths["assembly"])
    if screenshots:
        os.makedirs(screenshots, exist_ok=True)
        for name, doc in built.items():
            screenshot(sw, doc, os.path.join(screenshots,
                                             f"sw_part_{name}.png"))
        screenshot(sw, assembly, os.path.join(screenshots, "sw_assembly.png"))
    if record is not None:
        for tag, frame_paths in record.frames.items():
            write_gif(frame_paths, os.path.join(record_dir, f"{tag}.gif"))
    return paths


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--out", help="output folder (default: ./cad)")
    parser.add_argument("--overwrite", action="store_true")
    parser.add_argument("--screenshots", metavar="DIR",
                        help="also save a PNG of each part and the assembly")
    parser.add_argument("--record", metavar="DIR",
                        help="also save a frame per modelling step and an "
                             "animated GIF per part and for the assembly")
    args = parser.parse_args()
    for path in main(out_dir=args.out, overwrite=args.overwrite,
                     screenshots=args.screenshots,
                     record_dir=args.record).values():
        print(path)
