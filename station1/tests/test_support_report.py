"""Station 1b gates S1 (complete) and S2 (matches the converter) for lowlimb_converter.support.

S1 recounts elements with whole-file XPath queries written independently of the report's traversal.
S2 compares the report with a fresh XML conversion by the current converter (cvt1 XML + conversion log), made in
scratch_dir. (Until Station 3 it compared with the committed Station 1a outputs; the report now describes the
converter as changed since, see EXPERIMENTS.md.)
Usage: python station1/tests/test_support_report.py <upstream_clone_dir> <scratch_dir> [extra .osim files for S1 only ...]
Exit code 0 only if every check passes.
"""
import os, sys, collections
from lxml import etree
HERE = os.path.dirname(os.path.abspath(__file__)); REPO = os.path.join(HERE, "..", "..")
sys.path.insert(0, REPO)
from lowlimb_converter.support import build_report, read_osim

fails = []
def check(cond, msg):
    print(("  ok   " if cond else "  FAIL ") + msg)
    if not cond: fails.append(msg)

def n(root, xp): return len(root.xpath(xp))

def s1(osim):
    r = build_report(osim); m = read_osim(osim).getroot().find("Model")   # read_osim: "::" names (change 6.1)
    items = r["items"]
    print(f"S1 {os.path.basename(osim)}: {len(items)} items, outcome {r['outcome']}")
    check(all(i["status"] in ("converted","approximated","skipped","ignored","unsupported") and i["reason"] for i in items),
          "every item has one valid status and a reason")
    keys = [(i["section"], i["kind"], i["path"]) for i in items]
    dup = [k for k, c in collections.Counter(keys).items() if c > 1]
    check(not dup, f"no element listed twice (duplicates: {dup[:3]})")
    by = collections.Counter()
    for i in items:
        sec, kind = i["section"], i["kind"]
        if sec == "BodySet" and kind == "Body": by["bodies"] += 1
        elif sec == "JointSet" and kind == "Coordinate": by["coordinates"] += 1
        elif sec == "JointSet" and kind == "TransformAxis": by["axes"] += 1
        elif sec == "JointSet" and kind in ("Designation", "AxisOrder"): by["joint_checks"] += 1
        elif sec == "JointSet": by["joints"] += 1
        elif sec == "ConstraintSet": by["constraints"] += 1
        elif sec == "MarkerSet": by["markers"] += 1
        elif sec == "ForceSet" and "->" in i["path"]: by["path_wraps"] += 1
        elif sec == "ForceSet" and "/" in i["path"]: by["path_points"] += 1
        elif sec == "ForceSet": by["forces"] += 1
        elif sec in ("BodySet", "Ground") and kind.startswith("Wrap"): by["wrap_objects"] += 1
        elif sec in ("BodySet", "Ground") and i["rule"] == "body_components": by["body_components"] += 1
        elif sec in ("BodySet", "Ground"): by["geometry"] += 1
        elif sec == "Model" and kind == "gravity": by["gravity"] += 1
        elif sec == "Model" and kind == "defaults": by["defaults"] += 1
        elif sec == "Model": by["top_other"] += 1
        else: by["UNCLASSIFIED"] += 1
    other_sets = [c for c in m if isinstance(c.tag, str) and c.tag not in
                  {"Ground","ground","BodySet","JointSet","ConstraintSet","ForceSet","MarkerSet","gravity","credits",
                   "publications","length_units","force_units","assembly_accuracy","defaults"}]
    top_other = sum(max(1, len([x for x in (c.find("objects") if c.find("objects") is not None else c)
                                if isinstance(x.tag, str)])) for c in other_sets)
    expect = {
        "bodies": n(m, "BodySet/objects/*"),
        "joints": n(m, "JointSet/objects/*"),
        "coordinates": n(m, "JointSet//coordinates/Coordinate"),
        "axes": n(m, "JointSet//TransformAxis"),
        "constraints": n(m, "ConstraintSet/objects/*"),
        "forces": n(m, "ForceSet/objects/*"),
        "path_points": n(m, "ForceSet//PathPointSet/objects/*"),
        "path_wraps": n(m, "ForceSet//PathWrapSet/objects/*"),
        "markers": n(m, "MarkerSet/objects/*"),
        "wrap_objects": n(m, ".//WrapObjectSet/objects/*"),
        "geometry": n(m, "BodySet/objects/*/attached_geometry/*") + n(m, "Ground/attached_geometry/*")
                    + n(m, "Ground/components/*/attached_geometry/*"),
        "body_components": n(m, "BodySet/objects/*/components/*"),
        "top_other": top_other,
        "gravity": 1,
        "defaults": 1 if m.find("defaults") is not None else 0,   # change 1c.3
    }
    check(by["UNCLASSIFIED"] == 0, "0 unclassified items")
    for k, v in expect.items():
        check(by[k] == v, f"{k}: report {by[k]} == independent XPath count {v}")
    deep_geom = n(m, "BodySet/objects/*/components//attached_geometry/*")
    check(deep_geom == 0 or by["body_components"] > 0,
          f"geometry nested in body components ({deep_geom}) is covered by its ignored component")
    return r

def s2(osim, cvt1, log):
    r = build_report(osim)
    x = etree.parse(cvt1).getroot(); logtxt = open(log).read()
    print(f"S2 {os.path.basename(osim)} vs {os.path.basename(cvt1)}")
    names = lambda tag: {e.get("name") for e in x.iter(tag) if e.get("name")}
    bodies, joints, sites, geoms = names("body"), names("joint"), names("site"), names("geom")
    acts = {e.get("name") for e in x.find("actuator")}; tendons = names("spatial")
    eq_j1 = {e.get("joint1") for e in x.find("equality") if e.tag == "joint"}
    check(r["outcome"] == "converts", "predicted outcome is 'converts' (Station 1a converted it)")
    bad = []
    conv = ("converted", "approximated")
    for i in r["items"]:
        st, k, nm, p = i["status"], i["kind"], i["name"], i["path"]
        force = p.split("/")[0]
        if k == "Body": ok = (nm in bodies) == (st in conv)
        elif k == "Coordinate": ok = (nm in joints) == (st in conv)
        elif k in ("Thelen2003Muscle", "Millard2012EquilibriumMuscle", "Schutte1993Muscle_Deprecated"):
            ok = ((nm in acts) and (f"{nm}_tendon" in tendons)) == (st in conv)
        elif k == "Mesh": ok = (nm in geoms) == (st in conv)
        elif k == "Marker": ok = (f"{nm}_marker" in sites) == (st in conv)
        elif k == "PathPoint" or (k == "ConditionalPathPoint" and i["rule"] == "cpp_no_anchor"):
            ok = (f"{force}_{nm}" in sites) and (f"{force}_{nm}" not in bodies)
        elif k in ("MovingPathPoint", "ConditionalPathPoint"):
            ok = (f"{force}_{nm}" in bodies) and (f"{force}_{nm}" in sites)
        elif k == "TransformAxis" and i["rule"] == "axis_spline":
            ok = f"{p.replace('/', '_')}" in eq_j1
        elif k.startswith("Wrap") and st in conv: ok = any(g.startswith(f"{nm}_") for g in geoms)
        elif st == "skipped": ok = (nm not in acts) and (f"skipping {nm}" in logtxt)
        else: continue
        if not ok: bad.append(f"{st} {k} {p}")
    check(not bad, f"every checkable item matches cvt1 ({len(bad)} mismatches: {bad[:5]})")
    cpp_warn = [i for i in r["items"] if i["rule"] == "cpp_no_anchor"]
    check(all(f"ConditionalPathPoint {i['name']}" in logtxt for i in cpp_warn),
          f"all {len(cpp_warn)} no-anchor ConditionalPathPoint warnings are in the log")
    check(logtxt.count("Suitable 'anchor' PathPoint was not found") == len(cpp_warn),
          "no other no-anchor warnings in the log")
    fallback = [i for i in r["items"] if i["rule"] in ("cpp_upstream", "cpp_no_anchor")]
    fitted = [i for i in r["items"] if i["rule"] == "cpp"]
    check(logtxt.count("could not be evaluated with OpenSim, using the upstream method") == len(fallback),
          f"{len(fallback)} ConditionalPathPoints predicted to fall back to upstream's model, as in the log")
    check(all(f"ConditionalPathPoint {i['name']} of {i['path'].split('/')[0]}: using" in logtxt for i in fitted),
          f"all {len(fitted)} ConditionalPathPoints predicted to be fitted (change 3.1) are fitted in the log")
    opt = x.find("option")
    g_os = [float(v) for v in (etree.parse(osim).getroot().findtext(".//Model/gravity") or "0 -9.80665 0").split()]
    g_mj = [float(v) for v in opt.get("gravity").split()] if opt is not None and opt.get("gravity") else None
    check(g_mj is not None and abs(g_mj[0] - g_os[0]) < 1e-12 and abs(g_mj[1] + g_os[2]) < 1e-12
          and abs(g_mj[2] - g_os[1]) < 1e-12, f"cvt1 gravity {g_mj} is OpenSim's {g_os} rotated into MuJoCo's world")

up = sys.argv[1]
models = {"gait10dof18musc": f"{up}/models/osim/Gait10dof18musc/gait10dof18musc.osim",
          "gait2354": f"{up}/models/osim/Gait2354Simbody/gait2354.osim"}
scratch = os.path.abspath(sys.argv[2])
for name, osim in models.items():
    s1(osim)
    d = os.path.join(scratch, name)
    if os.path.exists(d):
        sys.exit(f"{d} already exists; use an empty scratch folder")
    import subprocess
    rc = subprocess.run([sys.executable, os.path.join(REPO, "station2", "convert_xml_step.py"), osim,
                         os.path.join(os.path.dirname(osim), "Geometry"), d], capture_output=True).returncode
    check(rc == 0, f"current converter's XML step runs on {name}")
    s2(osim, os.path.join(d, f"{name}_cvt1.xml"), os.path.join(d, f"{name}_conversion.log"))
for extra in sys.argv[3:]:
    s1(extra)
print("ALL CHECKS PASS" if not fails else f"{len(fails)} CHECK(S) FAILED")
sys.exit(1 if fails else 0)
