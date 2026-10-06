"""Element-support report for an OpenSim model (Station 1b).

Lists every element of an .osim file that the (ported) MyoConverter pipeline could act on and gives each exactly one
status, predicted from the vendored converter's source code (src/myoconverter/xml, MyoConverter @ cadf380 + our
port fixes):

  converted      converted with no known approximation
  approximated   converted, with a documented approximation (named in `reason`)
  skipped        the converter knowingly skips it and logs a warning
  ignored        the converter never looks at it, with no warning
  unsupported    the converter stops with an error here

Predicted outcome: "fails" at the first `unsupported` element in the converter's traversal order
(converter.convert: ground -> constraints -> bodies/joints depth-first from ground -> forces -> markers), otherwise
"converts". Every rule names the source file/function it comes from (RULES below). The report covers the XML
conversion step (Step 1) only; optimisation steps may fail for other reasons.

Usage:
  python -m lowlimb_converter.support <model.osim> [--geometry DIR] [--json out.json] [--md out.md]
Exit code: 0 if the predicted outcome is "converts", 1 if "fails", 2 on bad input.
"""
import argparse
import json
import os
import sys
from collections import Counter

from lxml import etree

CONVERTED, APPROX, SKIPPED, IGNORED, UNSUPPORTED = "converted", "approximated", "skipped", "ignored", "unsupported"
STATUSES = (CONVERTED, APPROX, SKIPPED, IGNORED, UNSUPPORTED)

# Element types with a parser (xml/parsers.py collects one class per file in each package folder).
JOINT_TYPES = {"CustomJoint", "PinJoint", "SliderJoint", "UniversalJoint", "WeldJoint"}
CONSTRAINT_TYPES = {"CoordinateCouplerConstraint"}
MUSCLE_TYPES = {"Thelen2003Muscle", "Millard2012EquilibriumMuscle", "Schutte1993Muscle_Deprecated"}
STUB_FORCES = {"BushingForce", "FunctionBasedBushingForce", "CoordinateLimitForce", "PointActuator",
               "TorqueActuator", "SpringGeneralizedForce"}   # parse() only logs "not implemented, skipping"
OTHER_FORCES = {"Ligament", "CoordinateActuator"}
PATH_POINT_TYPES = {"PathPoint", "MovingPathPoint", "ConditionalPathPoint"}
WRAP_TYPES = {"WrapCylinder", "WrapSphere", "WrapEllipsoid", "WrapTorus"}
WRAP_QUADRANTS = {"all", "+x", "x", "-x", "+y", "y", "-y", "+z", "z", "-z"}
SPLINES = {"SimmSpline", "NaturalCubicSpline"}
MPP_FUNCTIONS = {"SimmSpline", "NaturalCubicSpline", "PiecewiseLinearFunction"}
HANDLED_TOP = {"Ground", "ground", "BodySet", "JointSet", "ConstraintSet", "ForceSet", "MarkerSet"}
NON_ELEMENT_TOP = {"gravity", "credits", "publications", "length_units", "force_units", "assembly_accuracy",
                   "defaults"}

RULES = {
    "ground_mesh": "bodies/Ground.py _parse_component: attached_geometry/Mesh -> MuJoCo mesh geom",
    "ground_components": "bodies/Ground.py parse: if <components> exists only its children's geometry is parsed",
    "body": "bodies/Body.py parse: body with mass, mass_center, fullinertia (inertia dropped if not positive definite)",
    "body_unreached": "converter._add_bodies_and_joints: only bodies reachable from ground through JointSet are visited",
    "mesh": "bodies/Body.py parse: only attached_geometry/Mesh is converted (vtp->stl via utils.copy_mesh_file)",
    "other_geometry": "bodies/Body.py parse: findall('attached_geometry/Mesh') ignores other geometry types",
    "body_components": "bodies/Body.py parse: <components> of a body are not read",
    "wrap": "wrap_objects/<type>.py via parsers.WrapObjectParser",
    "wrap_ellipsoid": "wrap_objects/WrapEllipsoid.py: MuJoCo has no ellipsoid wrap; replaced by cylinder or sphere",
    "wrap_torus": "wrap_objects/WrapTorus.py: replaced by a small sphere the path must pass through",
    "wrap_quadrant": "wrap_objects/WrapObject.py parse: unknown quadrant -> NotImplementedError",
    "wrap_unknown": "parsers.BaseParser.parse: no parser for this tag -> NotImplementedError",
    "joint": "joints/<type>.py via parsers.JointParser",
    "joint_unknown": "parsers.BaseParser.parse: no parser for this tag -> NotImplementedError",
    "joint_ncoord": "joints/PinJoint|SliderJoint|UniversalJoint.py: wrong number of coordinates -> error",
    "axis_constant": "joints/CustomJoint.py: Constant -> locked joint (or skipped if ~0)",
    "axis_linear": "joints/CustomJoint.py: LinearFunction with coefficients (+-1, 0) -> plain/flipped joint",
    "axis_linear_bad": "joints/CustomJoint.py: LinearFunction with other coefficients -> RuntimeError",
    "axis_spline": "joints/CustomJoint.py: SimmSpline/NaturalCubicSpline -> quartic polynomial + joint equality",
    "axis_unknown": "joints/CustomJoint.py: other function types -> RuntimeError",
    "axis_multi_dep": "joints/CustomJoint.py: spline axis whose coordinate depends on >1 coordinate, or on a "
                      "non-identity LinearFunction coupling -> NotImplementedError",
    "axis_order": "joints/CustomJoint.py parse: axes must be named rotation1..3, translation1..3 in that order -> "
                  "IndexError",
    "designate_bad": "joints/CustomJoint.py _designate_dof: the first axis of each coordinate (processing order "
                     "translation1-3, rotation1-3) must be SimmSpline or LinearFunction -> NotImplementedError",
    "designate_linear_bad": "joints/CustomJoint.py _designate_dof: LinearFunction with slope != 1 and intercept != 0 "
                            "-> RuntimeWarning raised",
    "undesignated": "joints/CustomJoint.py parse: a coordinate never designated as a DoF has its equality removed; "
                    "with no such equality, M_EQUALITY.remove(None) -> TypeError",
    "mesh_ext": "bodies/utils.py copy_mesh_file: mesh file not .vtp or .stl -> NotImplementedError",
    "mesh_missing": "bodies/utils.py copy_mesh_file: mesh file not found in the geometry folder -> FileNotFoundError",
    "coordinate": "joints/utils.py: range->range, clamped->limited, locked->lock equality, default_value->keyframe",
    "coordinate_prescribed": "joints/utils.py: <prescribed>/<prescribed_function> are not read",
    "coordinate_fields": "joints/utils.py: reads <range>, <clamped>, <locked>; missing -> AttributeError",
    "constraint": "constraints/CoordinateCouplerConstraint.py: spline -> quartic fit; LinearFunction -> exact",
    "constraint_bad": "constraints/CoordinateCouplerConstraint.py: >2 independent coords or other function -> error",
    "constraint_unknown": "parsers.BaseParser.parse: no parser for this tag -> NotImplementedError",
    "muscle": "forces/Muscle.py: rigid-tendon MuJoCo muscle; F-L params fitted in Step 3; activation time "
              "constants kept; tendon compliance, fiber damping and F-V parameters not converted",
    "not_applied": "forces/Muscle.py, Ligament.py: appliesForce=false -> returns without creating anything",
    "stub": "forces/<type>.py: parse() logs 'not implemented, skipping'",
    "ligament": "forces/Ligament.py: spatial tendon with springlength=resting_length only; no stiffness, "
                "pcsa_force and force-length curve dropped, so it exerts no force",
    "coord_actuator": "forces/CoordinateActuator.py: motor on the joint; optimal_force read but dropped "
                      "(upstream TODO), so torque scaling differs",
    "force_unknown": "parsers.BaseParser.parse: no parser for this tag -> NotImplementedError",
    "path_point": "path_points/PathPoint.py: site on the parent body",
    "mpp": "path_points/MovingPathPoint.py: child body with slide joints; each axis fitted with a quartic "
           "polynomial of its coordinate (path_points/utils.get_moving_path_point_dependency)",
    "mpp_bad": "path_points/utils.get_moving_path_point_dependency: an axis without an <x>/<y> function "
               "(e.g. Constant) -> AttributeError/RuntimeError",
    "mpp_untested": "path_points/utils.get_moving_path_point_dependency: function type outside "
                    "SimmSpline/NaturalCubicSpline/PiecewiseLinearFunction is logged as critical but still fitted",
    "cpp": "path_points/ConditionalPathPoint.py: modelled as a moving point anchored to the nearest plain "
           "PathPoint on the same body (step change approximated by a polynomial)",
    "cpp_no_anchor": "path_points/ConditionalPathPoint.py: no anchor PathPoint on the same body -> warning, "
                     "treated as a normal fixed PathPoint (conditional behaviour lost)",
    "path_point_unknown": "parsers.BaseParser.parse: no parser for this tag -> NotImplementedError",
    "path_wrap": "path_wraps/PathWrapSet.py: MuJoCo tendon wrap over the referenced wrap object",
    "marker": "markers/Marker.py: site '<name>_marker' on the parent body",
    "marker_unknown": "parsers.BaseParser.parse: no parser for this tag -> NotImplementedError",
    "top_ignored": "converter.convert: this top-level set/component is never read",
    "gravity": "no gravity handling anywhere in src/myoconverter: MuJoCo's default (0, 0, -9.81) is used",
}


def _txt(el, path, default=None):
    f = el.find(path)
    return default if f is None or f.text is None else f.text.strip()


def _vec(s):
    return [float(v) for v in s.split()]


def _objects(parent, set_name):
    s = parent.find(set_name)
    if s is None:
        return []
    objs = s.find("objects")
    return [c for c in (objs if objs is not None else []) if isinstance(c.tag, str)]


class Report:
    def __init__(self, model, geometry=None):
        self.model = model
        self.geometry = geometry
        self.items = []
        self.order = 0

    def add(self, section, kind, name, status, rule, reason=None, path=None, traversed=True):
        assert status in STATUSES
        item = {"section": section, "kind": kind, "name": name, "status": status,
                "reason": reason or RULES[rule], "rule": rule, "path": path or name}
        if traversed:
            item["order"] = self.order
            self.order += 1
        self.items.append(item)
        return item


def _geometry(rep, owner_xml, owner_name, section):
    ag = owner_xml.find("attached_geometry")
    for g in ([c for c in ag if isinstance(c.tag, str)] if ag is not None else []):
        name = g.get("name", "")
        if g.tag == "Mesh":
            mf = _txt(g, "mesh_file", "")
            if mf[-3:] not in ("vtp", "stl"):
                rep.add(section, "Mesh", name, UNSUPPORTED, "mesh_ext", path=f"{owner_name}/{name}",
                        reason=f"mesh file '{mf}' is not .vtp or .stl -> NotImplementedError")
            elif rep.geometry is not None and not os.path.isfile(os.path.join(rep.geometry, mf)):
                rep.add(section, "Mesh", name, UNSUPPORTED, "mesh_missing", path=f"{owner_name}/{name}",
                        reason=f"mesh file '{mf}' not found in {rep.geometry} -> FileNotFoundError")
            else:
                rep.add(section, "Mesh", name, CONVERTED, "mesh", path=f"{owner_name}/{name}")
        else:
            rep.add(section, g.tag, name, IGNORED, "other_geometry", path=f"{owner_name}/{name}", traversed=False)


def _wraps(rep, owner_xml, owner_name, section):
    for w in _objects(owner_xml, "WrapObjectSet"):
        name, path = w.get("name", ""), f"{owner_name}/{w.get('name', '')}"
        if w.tag not in WRAP_TYPES:
            rep.add(section, w.tag, name, UNSUPPORTED, "wrap_unknown", path=path)
            continue
        quadrant = (_txt(w, "quadrant", "all") or "all").lower()
        if quadrant not in WRAP_QUADRANTS:
            rep.add(section, w.tag, name, UNSUPPORTED, "wrap_quadrant", path=path,
                    reason=f"quadrant '{quadrant}' not handled (WrapObject.py -> NotImplementedError)")
        elif w.tag == "WrapEllipsoid":
            rep.add(section, w.tag, name, APPROX, "wrap_ellipsoid", path=path)
        elif w.tag == "WrapTorus":
            rep.add(section, w.tag, name, APPROX, "wrap_torus", path=path)
        else:
            rep.add(section, w.tag, name, CONVERTED, "wrap", path=path)


def _function_of(ta):
    """(scale, function element) for a TransformAxis, looked up the way joints/CustomJoint.py does: unwrap one
    MultiplierFunction (its inner element is MultiplierFunction/function), then check Constant, SimmSpline,
    NaturalCubicSpline, LinearFunction in that order. Any other element is returned as-is (unsupported)."""
    t, scale = ta, 1.0
    if t.find("MultiplierFunction") is not None:
        scale = float(_txt(t, "MultiplierFunction/scale", "1"))
        t = t.find("MultiplierFunction/function")
        if t is None:
            return scale, None
    for tag in ("Constant", "SimmSpline", "NaturalCubicSpline", "LinearFunction"):
        if t.find(tag) is not None:
            return scale, t.find(tag)
    others = [c for c in t if isinstance(c.tag, str) and c.tag not in ("coordinates", "axis")]
    return scale, (others[0] if others else None)


def _coupling_ok(root, coord):
    """CustomJoint.py spline branch: a coordinate may depend on at most one other, through LinearFunction [1, 0]."""
    deps = root.xpath(f"ConstraintSet//dependent_coordinate_name[text()='{coord}']")
    if len(deps) > 1:
        return False
    if len(deps) == 1:
        c = deps[0].getparent()
        enforced = _txt(c, "isEnforced", "true")
        if enforced.lower() in ("false", "0"):
            return True
        ind = _txt(c, "independent_coordinate_names", "")
        if len(ind.split(",")) > 1 or len(ind.split(" ")) > 1:
            return False
        lf = c.find("coupled_coordinates_function/LinearFunction")
        if lf is None or _vec(_txt(lf, "coefficients", "0 0")) != [1.0, 0.0]:
            return False
    return True


def _joint(rep, root, joint, section="JointSet"):
    jname = joint.get("name", "")
    if joint.tag not in JOINT_TYPES:
        rep.add(section, joint.tag, jname, UNSUPPORTED, "joint_unknown")
        return
    cel = joint.find("coordinates")   # joints/*.py: parse_coordinates(xml.find("coordinates")) iterates children
    coords = [c for c in (cel if cel is not None else []) if isinstance(c.tag, str)]
    expected = {"PinJoint": 1, "SliderJoint": 1, "UniversalJoint": 2}.get(joint.tag)
    if expected is not None and len(coords) != expected:
        rep.add(section, joint.tag, jname, UNSUPPORTED, "joint_ncoord",
                reason=f"{joint.tag} has {len(coords)} coordinates, expected {expected}")
        return
    rep.add(section, joint.tag, jname, CONVERTED, "joint")
    for c in coords:
        cpath = f"{jname}/{c.get('name')}"
        if any(c.find(t) is None for t in ("range", "clamped", "locked")):
            rep.add(section, "Coordinate", c.get("name"), UNSUPPORTED, "coordinate_fields", path=cpath)
            continue
        prescribed = (_txt(c, "prescribed", "false") or "false").lower() in ("true", "1")
        if prescribed:
            rep.add(section, "Coordinate", c.get("name"), APPROX, "coordinate_prescribed", path=cpath,
                    reason="prescribed=true is not read: the coordinate becomes a free joint")
        else:
            rep.add(section, "Coordinate", c.get("name"), CONVERTED, "coordinate", path=cpath)
    if joint.tag != "CustomJoint":
        return
    _custom_axes(rep, root, joint, [c.get("name") for c in coords], section)


def _spline_xy(fn):
    return _vec(_txt(fn, "x", "")), _vec(_txt(fn, "y", ""))


def _custom_axes(rep, root, joint, coord_names, section):
    """Mirror joints/CustomJoint.py parse(): axes processed as translation1-3 then rotation1-3; the first axis of each
    coordinate goes through _designate_dof; then the per-axis function branch."""
    jname = joint.get("name", "")
    axes = joint.findall("SpatialTransform/TransformAxis")
    names = ["rotation1", "rotation2", "rotation3", "translation1", "translation2", "translation3"]
    if len(axes) != 6 or [a.get("name") for a in axes] != names:
        rep.add(section, "AxisOrder", jname, UNSUPPORTED, "axis_order", path=f"{jname}#axis-order")
        for ta in axes:
            rep.add(section, "TransformAxis", ta.get("name"), IGNORED, "axis_order", path=f"{jname}/{ta.get('name')}",
                    traversed=False, reason="not processed: the joint fails its axis-order check")
        return
    designated = []
    for idx in (3, 4, 5, 0, 1, 2):
        ta = axes[idx]
        apath = f"{jname}/{ta.get('name')}"
        scale, fn = _function_of(ta)
        coord = _txt(ta, "coordinates")
        status = None
        if coord and coord in coord_names and coord not in designated:
            # _designate_dof(t, ...): SimmSpline (identity -> designated; non-identity -> not designated),
            # LinearFunction (designated unless slope != 1 and intercept != 0), anything else -> error
            if fn is not None and fn.tag == "SimmSpline":
                x, y = _spline_xy(fn)
                if not (len(x) > 2 and not (len(x) == len(y) and all(abs(a - b) <= 1e-8 + 1e-5 * abs(b)
                                                                        for a, b in zip(x, y)))):
                    designated.append(coord)
                    status = (CONVERTED, "axis_linear", None)   # becomes LinearFunction [1, 0]
            elif fn is not None and fn.tag == "LinearFunction":
                k = _vec(_txt(fn, "coefficients", "1 0"))
                if k[0] != 1 and k[1] != 0:
                    status = (UNSUPPORTED, "designate_linear_bad", f"coefficients {k}")
                else:
                    designated.append(coord)
            else:
                status = (UNSUPPORTED, "designate_bad",
                          f"first axis of coordinate {coord} uses {fn.tag if fn is not None else 'no function'}")
        if status is None:
            if fn is None:
                status = (UNSUPPORTED, "axis_unknown", "no function element")
            elif fn.tag == "Constant":
                status = (CONVERTED, "axis_constant", None)
            elif fn.tag in SPLINES:
                status = (UNSUPPORTED, "axis_multi_dep", None) if coord and not _coupling_ok(root, coord) \
                    else (APPROX, "axis_spline", None)
            elif fn.tag == "LinearFunction":
                k = [scale * v for v in _vec(_txt(fn, "coefficients", "1 0"))]
                if abs(k[0]) != 1 or k[1] != 0:
                    status = (UNSUPPORTED, "axis_linear_bad", f"LinearFunction coefficients {k} (only (+-1, 0)) "
                                                              f"-> RuntimeError")
                else:
                    status = (CONVERTED, "axis_linear", None)
            else:
                status = (UNSUPPORTED, "axis_unknown", f"function type {fn.tag} not handled -> RuntimeError")
        st, rule, why = status
        rep.add(section, "TransformAxis", ta.get("name"), st, rule, path=apath,
                reason=(f"{RULES[rule]} ({why})" if why else None))
        if st == UNSUPPORTED:
            # conversion stops inside this joint; remaining axes are never processed
            done = {apath}
            for j in (3, 4, 5, 0, 1, 2):
                p2 = f"{jname}/{axes[j].get('name')}"
                if p2 not in done and not any(i["path"] == p2 and i["kind"] == "TransformAxis" for i in rep.items):
                    rep.add(section, "TransformAxis", axes[j].get("name"), IGNORED, "axis_unknown", path=p2,
                            traversed=False, reason="not processed: conversion stops at an earlier axis of this joint")
            return
    for c in coord_names:
        if c not in designated:
            has_eq = bool(root.xpath(f"ConstraintSet//CoordinateCouplerConstraint[dependent_coordinate_name='{c}']"))
            if not has_eq:
                rep.add(section, "Designation", c, UNSUPPORTED, "undesignated",
                        path=f"{jname}/{c}#designation")


def _body_tree(rep, root):
    """Depth-first traversal identical to converter._add_bodies_and_joints."""
    jointset = root.find("JointSet")
    visited_bodies, visited_joints = set(), set()

    def find_body(name):
        for b in _objects(root, "BodySet"):
            if b.get("name") == name:
                return b
        return None

    def visit(parent_name, depth=0):
        if jointset is None or depth > 500:
            return
        for s in jointset.xpath(f".//socket_parent[text()='{parent_name}']"):
            frames = s.getparent().getparent()
            joint = frames.getparent()
            pf = frames.find(f".//*[@name='{_txt(joint, 'socket_parent_frame')}']")
            cf = frames.find(f".//*[@name='{_txt(joint, 'socket_child_frame')}']")
            if pf is None or cf is None:
                rep.add("JointSet", joint.tag, joint.get("name", ""), UNSUPPORTED, "joint_unknown",
                        reason="socket frames not found")
                continue
            if _txt(cf, "socket_parent") == parent_name:
                continue
            child_name = _txt(cf, "socket_parent").split("/")[-1]
            body = find_body(child_name)
            if body is None:
                rep.add("BodySet", "Body", child_name, UNSUPPORTED, "body",
                        reason="child body referenced by a joint not found in BodySet")
                continue
            visited_bodies.add(child_name)
            visited_joints.add(joint.get("name"))
            rep.add("BodySet", body.tag, child_name, CONVERTED, "body")
            _geometry(rep, body, child_name, "BodySet")
            _wraps(rep, body, child_name, "BodySet")
            comps = body.find("components")
            for comp in ([c for c in comps if isinstance(c.tag, str)] if comps is not None else []):
                rep.add("BodySet", comp.tag, comp.get("name", ""), IGNORED, "body_components",
                        path=f"{child_name}/{comp.get('name', '')}", traversed=False)
            _joint(rep, root, joint)
            visit(_txt(cf, "socket_parent"), depth + 1)

    ground = root.find("Ground") if root.find("Ground") is not None else root.find("ground")
    visit(f"/{ground.get('name', 'ground')}")
    why = "belongs to a body/joint not reached from ground"
    for b in _objects(root, "BodySet"):
        if b.get("name") not in visited_bodies:
            bn = b.get("name")
            rep.add("BodySet", b.tag, bn, IGNORED, "body_unreached", traversed=False)
            ag = b.find("attached_geometry")
            for g in ([c for c in ag if isinstance(c.tag, str)] if ag is not None else []):
                rep.add("BodySet", g.tag, g.get("name", ""), IGNORED, "body_unreached", reason=why,
                        path=f"{bn}/{g.get('name', '')}", traversed=False)
            for w in _objects(b, "WrapObjectSet"):
                rep.add("BodySet", w.tag, w.get("name", ""), IGNORED, "body_unreached", reason=why,
                        path=f"{bn}/{w.get('name', '')}", traversed=False)
            comps = b.find("components")
            for comp in ([c for c in comps if isinstance(c.tag, str)] if comps is not None else []):
                rep.add("BodySet", comp.tag, comp.get("name", ""), IGNORED, "body_components",
                        path=f"{bn}/{comp.get('name', '')}", traversed=False)
    for j in _objects(root, "JointSet"):
        if j.get("name") not in visited_joints:
            jn = j.get("name")
            rep.add("JointSet", j.tag, jn, IGNORED, "body_unreached", traversed=False,
                    reason="joint not reached from ground (its child body is not visited)")
            cel = j.find("coordinates")
            for c in [c for c in (cel if cel is not None else []) if isinstance(c.tag, str)]:
                rep.add("JointSet", "Coordinate", c.get("name"), IGNORED, "body_unreached", reason=why,
                        path=f"{jn}/{c.get('name')}", traversed=False)
            for ta in j.findall("SpatialTransform/TransformAxis"):
                rep.add("JointSet", "TransformAxis", ta.get("name"), IGNORED, "body_unreached", reason=why,
                        path=f"{jn}/{ta.get('name')}", traversed=False)


def _ignore_path(rep, force, why):
    """List path points and path wraps of a force that is not converted (S1 completeness)."""
    fname = force.get("name")
    pps = force.find("GeometryPath/PathPointSet/objects")
    for pp in [c for c in (pps if pps is not None else []) if isinstance(c.tag, str)]:
        rep.add("ForceSet", pp.tag, pp.get("name"), IGNORED, "not_applied", reason=why,
                path=f"{fname}/{pp.get('name')}", traversed=False)
    gp = force.find("GeometryPath")
    for pw in (_objects(gp, "PathWrapSet") if gp is not None else []):
        rep.add("ForceSet", pw.tag, pw.get("name", ""), IGNORED, "not_applied", reason=why,
                path=f"{fname}/{pw.get('name', '')}->{_txt(pw, 'wrap_object')}", traversed=False)


def _mpp_status(pp):
    for axis in ("x_location", "y_location", "z_location"):
        a = pp.find(axis)
        if a is None or a.find(".//x") is None or a.find(".//y") is None:
            return UNSUPPORTED, "mpp_bad", f"{axis} has no function with <x>/<y> samples"
    first = pp.find(".//x").getparent().tag
    if first not in MPP_FUNCTIONS:
        return APPROX, "mpp_untested", None
    return APPROX, "mpp", None


def _path(rep, force):
    fname = force.get("name")
    pps = force.find("GeometryPath/PathPointSet/objects")
    children = [c for c in (pps if pps is not None else []) if isinstance(c.tag, str)]
    for i, pp in enumerate(children):
        path = f"{fname}/{pp.get('name')}"
        if pp.tag == "PathPoint":
            rep.add("ForceSet", "PathPoint", pp.get("name"), CONVERTED, "path_point", path=path)
        elif pp.tag == "MovingPathPoint":
            st, rule, reason = _mpp_status(pp)
            rep.add("ForceSet", "MovingPathPoint", pp.get("name"), st, rule, reason=reason, path=path)
        elif pp.tag == "ConditionalPathPoint":
            frame = _txt(pp, "socket_parent_frame")
            anchors = [c for c in children if c.tag == "PathPoint" and _txt(c, "socket_parent_frame") == frame]
            if anchors:
                rep.add("ForceSet", "ConditionalPathPoint", pp.get("name"), APPROX, "cpp", path=path)
            else:
                rep.add("ForceSet", "ConditionalPathPoint", pp.get("name"), APPROX, "cpp_no_anchor", path=path)
        else:
            rep.add("ForceSet", pp.tag, pp.get("name"), UNSUPPORTED, "path_point_unknown", path=path)
    for pw in _objects(force.find("GeometryPath") if force.find("GeometryPath") is not None else force,
                       "PathWrapSet"):
        rep.add("ForceSet", pw.tag, pw.get("name", ""), CONVERTED, "path_wrap",
                path=f"{fname}/{pw.get('name', '')}->{_txt(pw, 'wrap_object')}")


def _forces(rep, root):
    for f in _objects(root, "ForceSet"):
        name = f.get("name")
        applies = (_txt(f, "appliesForce", "true") or "true").lower() not in ("false", "0")
        if f.tag in MUSCLE_TYPES or f.tag == "Ligament":
            if not applies:
                rep.add("ForceSet", f.tag, name, IGNORED, "not_applied")
                _ignore_path(rep, f, "its force has appliesForce=false and is not converted")
                continue
            rep.add("ForceSet", f.tag, name, APPROX, "muscle" if f.tag in MUSCLE_TYPES else "ligament")
            _path(rep, f)
        elif f.tag == "CoordinateActuator":
            rep.add("ForceSet", f.tag, name, APPROX, "coord_actuator")
        elif f.tag in STUB_FORCES:
            rep.add("ForceSet", f.tag, name, SKIPPED, "stub")
            _ignore_path(rep, f, "its force is skipped (stub parser)")
        else:
            rep.add("ForceSet", f.tag, name, UNSUPPORTED, "force_unknown")
            _ignore_path(rep, f, "its force type has no parser")


def _constraints(rep, root):
    for c in _objects(root, "ConstraintSet"):
        name = c.get("name")
        if c.tag not in CONSTRAINT_TYPES:
            rep.add("ConstraintSet", c.tag, name, UNSUPPORTED, "constraint_unknown")
            continue
        ind = _txt(c, "independent_coordinate_names", "")
        ccf = c.find("coupled_coordinates_function")
        fn = [x for x in (ccf if ccf is not None else []) if isinstance(x.tag, str)]
        if len(ind.split(" ")) > 2 or len(ind.split(",")) > 2 or not fn or fn[0].tag not in SPLINES | {"LinearFunction"}:
            rep.add("ConstraintSet", c.tag, name, UNSUPPORTED, "constraint_bad")
        elif fn[0].tag in SPLINES:
            rep.add("ConstraintSet", c.tag, name, APPROX, "constraint")
        else:
            rep.add("ConstraintSet", c.tag, name, CONVERTED, "constraint")


def _ground(rep, root):
    g = root.find("Ground") if root.find("Ground") is not None else root.find("ground")
    gname = g.get("name", "ground")
    comps = g.find("components")
    if comps is not None:
        for comp in [c for c in comps if isinstance(c.tag, str)]:
            _geometry(rep, comp, f"{gname}/{comp.get('name', '')}", "Ground")
        ag = g.find("attached_geometry")
        for x in ([c for c in ag if isinstance(c.tag, str)] if ag is not None else []):
            rep.add("Ground", x.tag, x.get("name", ""), IGNORED, "ground_components",
                    path=f"{gname}/{x.get('name', '')}", traversed=False)
    else:
        _geometry(rep, g, gname, "Ground")
    _wraps(rep, g, gname, "Ground")


def build_report(osim_path, geometry=None):
    tree = etree.parse(osim_path)
    root = tree.getroot().find("Model")
    if root is None:
        raise ValueError("no <Model> element: not an OpenSim 4 model file")
    rep = Report(root.get("name"), geometry)
    _ground(rep, root)
    _constraints(rep, root)
    _body_tree(rep, root)
    _forces(rep, root)
    for m in _objects(root, "MarkerSet"):
        if m.tag == "Marker":
            rep.add("MarkerSet", "Marker", m.get("name"), CONVERTED, "marker")
        else:
            rep.add("MarkerSet", m.tag, m.get("name"), UNSUPPORTED, "marker_unknown")
    for child in [c for c in root if isinstance(c.tag, str)]:
        if child.tag in HANDLED_TOP or child.tag in NON_ELEMENT_TOP:
            continue
        objs = child.find("objects")
        members = [c for c in (objs if objs is not None else child) if isinstance(c.tag, str)]
        if not members:
            rep.add("Model", child.tag, child.tag, IGNORED, "top_ignored", traversed=False,
                    reason=f"{RULES['top_ignored']} (empty)")
        for m in members:
            rep.add("Model", m.tag, m.get("name", m.tag), IGNORED, "top_ignored",
                    path=f"{child.tag}/{m.get('name', m.tag)}", traversed=False)
    gravity = _txt(root, "gravity", "0 -9.80665 0")
    rep.add("Model", "gravity", "gravity", APPROX, "gravity", traversed=False,
            reason=f"OpenSim gravity ({gravity}) not converted; MuJoCo default (0 0 -9.81) used")

    counts = Counter(i["status"] for i in rep.items)
    unsupported = sorted((i for i in rep.items if i["status"] == UNSUPPORTED), key=lambda i: i.get("order", 1e9))
    return {
        "model": rep.model, "file": os.path.basename(osim_path), "geometry_checked": geometry is not None,
        "outcome": "fails" if unsupported else "converts",
        "first_failure": unsupported[0] if unsupported else None,
        "counts": {s: counts.get(s, 0) for s in STATUSES},
        "items": rep.items,
    }


def to_markdown(r):
    lines = [f"# Support report: {r['model']}", "",
             f"Predicted outcome of the XML conversion step: **{r['outcome']}**"
             + (f" (first failure: {r['first_failure']['kind']} `{r['first_failure']['path']}`)"
                if r["first_failure"] else ""), "",
             "| Status | Count |", "| --- | --- |"] + [f"| {s} | {n} |" for s, n in r["counts"].items()]
    lines += ["", "| Status | Section | Type | Element | Reason |", "| --- | --- | --- | --- | --- |"]
    for s in (UNSUPPORTED, SKIPPED, IGNORED, APPROX):
        groups = Counter((i["section"], i["kind"], i["reason"]) for i in r["items"] if i["status"] == s)
        for (sec, kind, reason), n in sorted(groups.items()):
            examples = [i["path"] for i in r["items"] if i["status"] == s and i["section"] == sec
                        and i["kind"] == kind and i["reason"] == reason]
            shown = ", ".join(f"`{e}`" for e in examples[:3]) + (f" (+{n - 3} more)" if n > 3 else "")
            lines.append(f"| {s} | {sec} | {kind} | {shown} | {reason} |")
    conv = Counter((i["section"], i["kind"]) for i in r["items"] if i["status"] == CONVERTED)
    lines.append("| converted | — | — | " + ", ".join(f"{k} ×{n}" for (_, k), n in sorted(conv.items())) + " | — |")
    return "\n".join(lines) + "\n"


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("osim")
    ap.add_argument("--geometry", help="folder the converter will read meshes from; checks every mesh file exists")
    ap.add_argument("--json")
    ap.add_argument("--md")
    a = ap.parse_args(argv)
    try:
        r = build_report(a.osim, a.geometry)
    except (OSError, etree.XMLSyntaxError, ValueError) as e:
        print(f"error: {e}", file=sys.stderr)
        return 2
    if a.json:
        with open(a.json, "w") as f:
            json.dump(r, f, indent=1)
    if a.md:
        with open(a.md, "w") as f:
            f.write(to_markdown(r))
    print(f"{r['model']}: {r['outcome']} | " + ", ".join(f"{k} {v}" for k, v in r["counts"].items())
          + (f" | first failure: {r['first_failure']['kind']} {r['first_failure']['path']}"
             f" ({r['first_failure']['reason']})" if r["first_failure"] else ""))
    return 0 if r["outcome"] == "converts" else 1


if __name__ == "__main__":
    sys.exit(main())
