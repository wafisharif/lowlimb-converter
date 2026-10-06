"""Station 1b gate S3: the support report's predicted outcome matches the real XML conversion step.

For each model, runs the ported pipeline with convert_steps=[1], conversion only (no validation, no PDF), in a
subprocess, and compares:
  predicted 'converts' -> the step must finish and write *_cvt1.xml
  predicted 'fails' at element E -> the step must raise, and the converter's stack frames at the moment of the
  error must be processing exactly E: for 'joint/axis' the CustomJoint being parsed and the axis transform_axes[idx];
  for 'force/point' the force and path point; for 'body/mesh' the body and mesh; otherwise the element 'xml'.
(Matching revised 2026-10-06 after the first S3 run, to be stricter: the first version required the axis name to
appear in the error text, which some upstream errors never contain. See EXPERIMENTS.md.)
Usage: python station1/tests/test_support_predictions.py <out_dir> <model.osim>:<geometry_dir> [...]
"""
import os, re, sys, json, subprocess
HERE = os.path.dirname(os.path.abspath(__file__)); REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, REPO)
from lowlimb_converter.support import build_report

RUNNER = r'''
import sys, json, traceback
sys.path.insert(0, sys.argv[4])
from myoconverter.O2MPipeline import O2MPipeline
try:
    O2MPipeline(sys.argv[1], sys.argv[2], sys.argv[3], convert_steps=[1], muscle_list=None, osim_data_overwrite=True,
                conversion=True, validation=False, speedy=False, generate_pdf=False, add_ground_geom=True,
                treat_as_normal_path_point=False)
except BaseException as e:
    frames = []
    tb = e.__traceback__
    while tb is not None:
        f = tb.tb_frame
        if "/myoconverter/xml/" in f.f_code.co_filename:
            L = f.f_locals
            info = {"func": f.f_code.co_name, "file": f.f_code.co_filename.split("/myoconverter/")[-1]}
            for k in ("xml", "path_point", "mesh", "joint"):
                el = L.get(k)
                if hasattr(el, "attrib") and "name" in el.attrib:
                    info[k] = el.attrib["name"]
            if isinstance(L.get("force_name"), str):
                info["force_name"] = L["force_name"]
            if "transform_axes" in L and "idx" in L:
                info["axis"] = L["transform_axes"][L["idx"]].attrib.get("name")
            frames.append(info)
        tb = tb.tb_next
    traceback.print_exc()
    print("CRASH_FRAMES " + json.dumps(frames))
    sys.exit(1)
'''
out_root = sys.argv[1]; fails = []
for case, spec in enumerate(sys.argv[2:]):
    osim, geom = spec.split(":")
    r = build_report(osim, geom)
    name = os.path.splitext(os.path.basename(osim))[0]
    out = os.path.join(out_root, f"{case}_{name}")   # one folder per case: same model may be tested twice
    if os.path.exists(out):
        raise SystemExit(f"output folder {out} already exists; use an empty out_dir")
    os.makedirs(out)
    p = subprocess.run([sys.executable, "-c", RUNNER, osim, geom, out, os.path.join(REPO, "src")],
                       capture_output=True, text=True, timeout=3600)
    text = p.stdout + p.stderr
    open(os.path.join(out, "step1_output.txt"), "w").write(text)
    produced = any(f.endswith("_cvt1.xml") for f in os.listdir(out))
    crashed = p.returncode != 0 or "Traceback" in text or "An error has been caught" in text
    parsed = re.findall(r"\[(\w+Parser): (\w+)\] (\S+)", text)
    last = parsed[-1] if parsed else None
    pred = r["outcome"]; ff = r["first_failure"]
    print(f"{name}: predicted {pred}" + (f" at {ff['kind']} {ff['path']}" if ff else "")
          + f" | actual: {'crashed' if crashed else 'finished'}, cvt1 written: {produced}, "
          + f"last parsed: {last}, exit {p.returncode}")
    frames = json.loads(text.split("CRASH_FRAMES ", 1)[1].splitlines()[0]) if "CRASH_FRAMES " in text else []
    if frames:
        print(f"  crash frames: {frames}")
    if pred == "converts":
        ok = (not crashed) and produced
    else:
        top, _, child = ff["path"].split("#")[0].partition("/")
        def hit(fr):
            if ff["kind"] == "TransformAxis":
                return fr.get("xml") == top and fr.get("axis") == child
            if ff["kind"] == "Mesh":
                return fr.get("xml") == top and fr.get("mesh") == child
            if child:  # path point of a force
                return fr.get("force_name") == top and child in (fr.get("xml"), fr.get("path_point"))
            return top in (fr.get("xml"), fr.get("joint"))
        ok = crashed and not produced and any(hit(fr) for fr in frames)
    print(("  ok   " if ok else "  FAIL ") + "prediction matches the conversion step")
    if not ok: fails.append(name)
print("ALL PREDICTIONS MATCH" if not fails else f"MISMATCH: {fails}")
sys.exit(1 if fails else 0)
