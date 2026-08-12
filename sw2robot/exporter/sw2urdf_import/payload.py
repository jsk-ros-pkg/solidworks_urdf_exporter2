"""SW2URDF DataContract payload parsing helpers."""

from __future__ import annotations

import base64
import re
import xml.etree.ElementTree as ET

_Z_NS = "http://schemas.microsoft.com/2003/10/Serialization/"
_I_NS = "http://www.w3.org/2001/XMLSchema-instance"
_ZID = f"{{{_Z_NS}}}Id"
_ZREF = f"{{{_Z_NS}}}Ref"
_INIL = f"{{{_I_NS}}}nil"
_ITYPE = f"{{{_I_NS}}}type"

_PID_ENTRY_RE = re.compile(rb"\xff\xfe\xff(.)", re.DOTALL)


def _warn(msg):
    print("      !!! SW2URDF config warning: " + str(msg))


def _local_name(tag):
    if not isinstance(tag, str):
        return ""
    return tag.rsplit("}", 1)[-1]


def _text(elem):
    if elem is None:
        return None
    txt = (elem.text or "").strip()
    return txt if txt != "" else ""


def _is_nil(elem):
    if elem is None:
        return False
    raw = str(elem.attrib.get(_INIL, "")).strip().lower()
    return raw in ("true", "1")


def _child(elem, name):
    if elem is None:
        return None
    for ch in list(elem):
        if _local_name(ch.tag) == name:
            return ch
    return None


def _children(elem, name):
    if elem is None:
        return []
    return [ch for ch in list(elem) if _local_name(ch.tag) == name]


def _resolve_ref(elem, id_map, context="element"):
    if elem is None:
        return None
    ref = elem.attrib.get(_ZREF)
    if not ref:
        return elem
    target = id_map.get(ref)
    if target is None:
        _warn(f"{context} references missing z:Ref={ref!r}")
        return None
    return target


def _parse_value(value_elem, id_map):
    value_elem = _resolve_ref(value_elem, id_map, "Value")
    if value_elem is None:
        return None
    if _is_nil(value_elem):
        return None

    itype = str(value_elem.attrib.get(_ITYPE, "")).lower()
    if "arrayofdouble" in itype:
        vals = []
        for child in list(value_elem):
            if _local_name(child.tag) != "double":
                continue
            txt = (child.text or "").strip()
            if txt == "":
                continue
            try:
                vals.append(float(txt))
            except ValueError:
                _warn(f"invalid array double value {txt!r}")
                return None
        return vals

    txt = (value_elem.text or "").strip()
    if "double" in itype:
        if txt == "":
            return None
        try:
            return float(txt)
        except ValueError:
            _warn(f"invalid numeric value {txt!r}")
            return None

    if txt != "":
        return txt

    if len(list(value_elem)) == 0:
        return ""

    parts = []
    for child in list(value_elem):
        if child.text:
            parts.append(child.text)
    return "".join(parts).strip()


def _parse_attributes(owner_elem, id_map):
    attrs = {}
    attr_block = _resolve_ref(_child(owner_elem, "Attributes"), id_map, "Attributes")
    for attr in _children(attr_block, "Attribute"):
        attr = _resolve_ref(attr, id_map, "Attribute")
        if attr is None:
            continue
        attr_type = _text(_child(attr, "AttributeType"))
        if not attr_type:
            continue
        attrs[str(attr_type)] = _parse_value(_child(attr, "Value"), id_map)
    return attrs


def _decode_pid_blob(blob_text):
    if not blob_text:
        return []
    try:
        raw = base64.b64decode(blob_text.strip())
    except Exception as exc:
        _warn(f"invalid SW PID base64 payload: {exc!r}")
        return []

    out = []
    for match in _PID_ENTRY_RE.finditer(raw):
        nch = match.group(1)[0]
        start = match.end()
        end = start + (2 * nch)
        if end > len(raw):
            _warn("truncated SW PID entry in base64 payload")
            break
        payload = raw[start:end]
        try:
            text = payload.decode("utf-16le")
        except UnicodeDecodeError:
            text = payload.decode("utf-16le", errors="ignore")
        text = text.strip("\x00")
        if text:
            out.append(text)

    deduped = []
    for item in out:
        if item not in deduped:
            deduped.append(item)
    return deduped


def _parse_pid_values(link_elem, field_name, id_map):
    field = _resolve_ref(_child(link_elem, field_name), id_map, field_name)
    if field is None or _is_nil(field):
        return []

    blobs = []
    if _local_name(field.tag) == "base64Binary":
        txt = (field.text or "").strip()
        if txt:
            blobs.append(txt)
    for node in field.iter():
        if _local_name(node.tag) != "base64Binary":
            continue
        txt = (node.text or "").strip()
        if txt:
            blobs.append(txt)

    out = []
    for blob in blobs:
        out.extend(_decode_pid_blob(blob))

    deduped = []
    for item in out:
        if item not in deduped:
            deduped.append(item)
    return deduped


def _joint_type_name(elem):
    raw = str(elem.attrib.get(_ITYPE, "") or "")
    return raw.split(":", 1)[-1]


def _joint_urdf_child(joint_elem, id_map, prop_name, itype_name):
    direct = _resolve_ref(_child(joint_elem, prop_name), id_map, prop_name)
    if direct is not None and _local_name(direct.tag) == "URDFElement":
        return direct

    child_block = _resolve_ref(_child(joint_elem, "ChildElements"), id_map, "ChildElements")
    for child in _children(child_block, "URDFElement"):
        child = _resolve_ref(child, id_map, "URDFElement")
        if child is None:
            continue
        if _joint_type_name(child).lower() == itype_name.lower():
            return child
    return None


def _joint_ref_attribute_value(joint_elem, id_map, field_name):
    attr = _resolve_ref(_child(joint_elem, field_name), id_map, field_name)
    if attr is None:
        return None
    if _local_name(attr.tag) != "Attribute":
        return None
    return _parse_value(_child(attr, "Value"), id_map)


def _non_nil_subset(data, keys):
    if not isinstance(data, dict):
        return None
    out = {}
    for key in keys:
        if key not in data:
            continue
        val = data.get(key)
        if val is None:
            continue
        if isinstance(val, str) and val.strip() == "":
            continue
        out[key] = val
    return out or None


def _parse_joint(link_elem, id_map):
    joint = _resolve_ref(_child(link_elem, "Joint"), id_map, "Joint")
    if joint is None or _local_name(joint.tag) != "URDFElement":
        child_block = _resolve_ref(_child(link_elem, "ChildElements"), id_map, "ChildElements")
        for elem in _children(child_block, "URDFElement"):
            elem = _resolve_ref(elem, id_map, "URDFElement")
            if elem is None:
                continue
            if _joint_type_name(elem).lower() == "joint":
                joint = elem
                break
    if joint is None:
        return None

    name_val = _joint_ref_attribute_value(joint, id_map, "NameAttribute")
    type_val = _joint_ref_attribute_value(joint, id_map, "TypeAttribute")
    name_text = str(name_val).strip() if name_val is not None else ""
    type_text = str(type_val).strip().lower() if type_val is not None else ""

    origin_attrs = _parse_attributes(_joint_urdf_child(joint, id_map, "Origin", "Origin"), id_map)
    axis_attrs = _parse_attributes(_joint_urdf_child(joint, id_map, "Axis", "Axis"), id_map)
    limit_attrs = _parse_attributes(_joint_urdf_child(joint, id_map, "Limit", "Limit"), id_map)
    dynamics_attrs = _parse_attributes(_joint_urdf_child(joint, id_map, "Dynamics", "Dynamics"), id_map)
    mimic_attrs = _parse_attributes(_joint_urdf_child(joint, id_map, "Mimic", "Mimic"), id_map)
    safety_elem = _joint_urdf_child(joint, id_map, "Safety", "SafetyController")
    safety_attrs = _parse_attributes(safety_elem, id_map)
    calib_attrs = _parse_attributes(_joint_urdf_child(joint, id_map, "Calibration", "Calibration"), id_map)

    axis_name = (_text(_child(joint, "AxisName")) or "").strip()
    coordsys_name = (_text(_child(joint, "CoordinateSystemName")) or "").strip()

    return {
        "name": name_text or None,
        "type": type_text or None,
        "axis_name": axis_name or None,
        "coordsys_name": coordsys_name or None,
        "origin_xyz": origin_attrs.get("xyz"),
        "origin_rpy": origin_attrs.get("rpy"),
        "axis_xyz": axis_attrs.get("xyz"),
        "limit": _non_nil_subset(limit_attrs, ("lower", "upper", "effort", "velocity")),
        "dynamics": _non_nil_subset(dynamics_attrs, ("damping", "friction")),
        "mimic": _non_nil_subset(mimic_attrs, ("joint", "multiplier", "offset")),
        "safety": _non_nil_subset(
            safety_attrs,
            ("soft_lower_limit", "soft_upper_limit", "k_position", "k_velocity"),
        ),
        "calibration": _non_nil_subset(calib_attrs, ("rising", "falling")),
    }


def _parse_link_tree(link_elem, id_map, parent_link_name, out_rows):
    link_elem = _resolve_ref(link_elem, id_map, "Link")
    if link_elem is None or _local_name(link_elem.tag) != "Link":
        _warn("encountered invalid Link node in SW2URDF payload")
        return None

    attrs = _parse_attributes(link_elem, id_map)
    link_name = str(attrs.get("name") or "").strip()
    if not link_name:
        _warn("payload link is missing AttributeType='name'")
        return None

    components = _parse_pid_values(link_elem, "SWComponentPIDs", id_map)
    main_components = _parse_pid_values(link_elem, "SWMainComponentPID", id_map)
    for main in reversed(main_components):
        if main not in components:
            components.insert(0, main)

    joint = _parse_joint(link_elem, id_map)
    rec = {
        "link_name": link_name,
        "parent_link_name": parent_link_name,
        "components": components,
        "main_component": main_components[0] if main_components else None,
        "joint": joint,
    }

    if parent_link_name is None:
        root_rec = rec
    else:
        out_rows.append(rec)
        root_rec = None

    children_block = _resolve_ref(_child(link_elem, "Children"), id_map, "Children")
    for child in _children(children_block, "Link"):
        child_root = _parse_link_tree(child, id_map, link_name, out_rows)
        if parent_link_name is None and child_root is not None:
            _warn("unexpected nested root while parsing SW2URDF payload")

    return root_rec


def _empty_joint():
    return {
        "name": None,
        "type": None,
        "axis_name": None,
        "coordsys_name": None,
        "origin_xyz": None,
        "origin_rpy": None,
        "axis_xyz": None,
        "limit": None,
        "dynamics": None,
        "mimic": None,
        "safety": None,
        "calibration": None,
    }


def parse_sw2urdf_payload(xml_text):
    """Parse the SW2URDF payload XML into a compact link/joint tree.

    Parameters
    ----------
    xml_text : str
        Raw DataContract XML payload from the SW2URDF attribute feature.

    Returns
    -------
    dict | None
        Parsed payload dict on success, else None.
    """
    if not isinstance(xml_text, str) or not xml_text.strip():
        _warn("payload XML text is empty")
        return None

    try:
        root_elem = ET.fromstring(xml_text)
    except ET.ParseError as exc:
        _warn(f"payload XML is malformed: {exc}")
        return None

    if _local_name(root_elem.tag) != "Link":
        _warn(f"unexpected payload root element: {_local_name(root_elem.tag)!r}")
        return None

    id_map = {}
    for elem in root_elem.iter():
        zid = elem.attrib.get(_ZID)
        if zid:
            id_map[zid] = elem

    rows = []
    root_rec = _parse_link_tree(root_elem, id_map, None, rows)
    if not isinstance(root_rec, dict):
        _warn("could not parse SW2URDF root link")
        return None

    root_joint = root_rec.get("joint") or {}
    out = {
        "root": {
            "link_name": root_rec.get("link_name"),
            "components": list(root_rec.get("components") or []),
            "main_component": root_rec.get("main_component"),
            "coordsys_name": root_joint.get("coordsys_name"),
        },
        "links": [],
    }

    if not out["root"]["link_name"]:
        _warn("root link name is missing in payload")
        return None

    for row in rows:
        joint = row.get("joint") or _empty_joint()
        if row.get("parent_link_name") is None:
            _warn(f"non-root payload link {row.get('link_name')!r} has no parent name")
        out["links"].append({
            "link_name": row.get("link_name"),
            "parent_link_name": row.get("parent_link_name"),
            "components": list(row.get("components") or []),
            "main_component": row.get("main_component"),
            "joint": joint,
        })

    return out
