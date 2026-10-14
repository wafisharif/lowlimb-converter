"""Station 6, gates L0-L1: which of OpenSim's own models the XML step converts, and does the support report know?

For every .osim file in opensim-models (sorted paths), with the geometry folder built as station6/GATES.md says
(repo-wide Geometry/, overlaid with <model folder>/Geometry/, overlaid with the mesh files in the model folder itself):
- the support report's prediction (osim2mujoco.support for our converter; for the MyoConverter column, the report
  as of commit 4313be5, Station 1b, which describes the Station 1a port);
- the real XML step (convert_steps=[1]) in a subprocess with that column's converter, its outcome, and where it stopped
  (the converter's stack frames, matched to the predicted element exactly as Station 1b's gate S3 does).
Writes <out_dir>/<column>.json and prints one line per file.

Usage: python station6/coverage.py <opensim_models_dir> <work_dir> <column: ours|myoconverter> <out_json>
  ours: this repo's src/ and support report; myoconverter: src/ of tag station1a (1670ae3) and the support report of
  4313be5, both taken from this repo's git history.
"""
import glob
import hashlib
import importlib.util
import json
import os
import shutil
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
PORT_COMMIT, PORT_SUPPORT_COMMIT = "1670ae3", "4313be5"
MESH_EXT = (".vtp", ".stl", ".obj")

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


def geometry_folder(osm, osim, work):
    """repo-wide Geometry/, then <model folder>/Geometry/, then mesh files in the model folder (later wins)."""
    folder = os.path.dirname(osim)
    out = os.path.join(work, "geometry", hashlib.sha256(folder.encode()).hexdigest()[:12])
    if not os.path.isdir(out):
        os.makedirs(out)
        for src in (os.path.join(osm, "Geometry"), os.path.join(folder, "Geometry")):
            if os.path.isdir(src):
                for f in os.listdir(src):
                    if os.path.isfile(os.path.join(src, f)):
                        shutil.copyfile(os.path.join(src, f), os.path.join(out, f))
        for f in os.listdir(folder):
            if f.lower().endswith(MESH_EXT) and os.path.isfile(os.path.join(folder, f)):
                shutil.copyfile(os.path.join(folder, f), os.path.join(out, f))
    return out


def load_support(column, work):
    if column == "ours":
        sys.path.insert(0, REPO)
        from osim2mujoco import support
        return support, os.path.join(REPO, "src")
    port = os.path.join(work, "port")
    if not os.path.isdir(os.path.join(port, "src")):
        os.makedirs(port, exist_ok=True)
        subprocess.run(f"git -C {REPO} archive {PORT_COMMIT} src | tar -x -C {port}", shell=True, check=True)
        with open(os.path.join(port, "support_port.py"), "wb") as f:
            f.write(subprocess.run(["git", "-C", REPO, "show", f"{PORT_SUPPORT_COMMIT}:lowlimb_converter/support.py"],
                                   capture_output=True, check=True).stdout)
    spec = importlib.util.spec_from_file_location("support_port", os.path.join(port, "support_port.py"))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod, os.path.join(port, "src")


def matches(pred, crashed, produced, frames, text):
    """Station 1b gate S3's rule; a report that can't read the file must match a converter that can't either."""
    if pred["outcome"] == "converts":
        return (not crashed) and produced
    if pred["outcome"] == "error":
        return crashed and not produced and "XMLSyntaxError" in text     # neither could read the file
    ff = pred["first_failure"]
    top, _, child = ff["path"].split("#")[0].partition("/")

    def hit(fr):
        if ff["kind"] == "TransformAxis":
            return fr.get("xml") == top and fr.get("axis") == child
        if ff["kind"] == "Mesh":
            return fr.get("mesh") == ff["name"]
        if child:
            return fr.get("force_name") == top and child in (fr.get("xml"), fr.get("path_point"))
        return top in (fr.get("xml"), fr.get("joint"))
    return crashed and not produced and any(hit(fr) for fr in frames)


def main():
    osm, work, column, out_json = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2]), sys.argv[3], sys.argv[4]
    os.makedirs(work, exist_ok=True)
    support, src = load_support(column, work)
    files = sorted(os.path.relpath(p, osm) for p in glob.glob(os.path.join(osm, "**", "*.osim"), recursive=True))
    results = []
    for k, rel in enumerate(files):
        osim = os.path.join(osm, rel)
        geom = geometry_folder(osm, osim, work)
        try:
            r = support.build_report(osim, geom)
            pred = {"outcome": r["outcome"], "first_failure": r["first_failure"]}
        except Exception as e:                       # e.g. the file can't be read
            pred = {"outcome": "error", "first_failure": None, "error": f"{type(e).__name__}: {e}"[:300]}
        out = os.path.join(work, column, f"{k:02d}")
        if os.path.exists(out):
            shutil.rmtree(out)
        os.makedirs(out)
        p = subprocess.run([sys.executable, "-c", RUNNER, osim, geom, out, src], capture_output=True, text=True,
                           timeout=7200)
        text = p.stdout + p.stderr
        with open(os.path.join(out, "step1_output.txt"), "w") as f:
            f.write(text)
        produced = any(f.endswith("_cvt1.xml") for f in os.listdir(out))
        crashed = p.returncode != 0 or "Traceback" in text
        frames = json.loads(text.split("CRASH_FRAMES ", 1)[1].splitlines()[0]) if "CRASH_FRAMES " in text else []
        error_line = next((ln for ln in reversed(text.strip().splitlines()) if "Error" in ln), "") if crashed else ""
        ok = matches(pred, crashed, produced, frames, text)
        with open(osim, "rb") as f:
            sha = hashlib.sha256(f.read()).hexdigest()
        cvt1 = next((os.path.join(out, f) for f in os.listdir(out) if f.endswith("_cvt1.xml")), None)
        results.append({"file": rel, "sha256": sha, "predicted": pred["outcome"],
                        "predicted_first_failure": (f"{pred['first_failure']['kind']} {pred['first_failure']['path']}"
                                                    if pred.get("first_failure") else pred.get("error")),
                        "converted": produced and not crashed, "crashed": crashed, "error": error_line[:300],
                        "prediction_matches": ok, "cvt1": cvt1, "geometry": geom})
        print(f"{rel}: predicted {pred['outcome']}"
              + (f" ({results[-1]['predicted_first_failure']})" if pred["outcome"] != "converts" else "")
              + f" | {'converted' if results[-1]['converted'] else 'failed: ' + error_line[:120]}"
              + f" | {'ok' if ok else 'MISMATCH'}", flush=True)
    summary = {"column": column, "files": len(results), "converted": sum(r["converted"] for r in results),
               "predictions_match": sum(r["prediction_matches"] for r in results), "results": results}
    with open(out_json, "w") as f:
        json.dump(summary, f, indent=1)
    print(f"{column}: {summary['converted']}/{summary['files']} converted; predictions match "
          f"{summary['predictions_match']}/{summary['files']}")
    return 0 if summary["predictions_match"] == summary["files"] else 1


if __name__ == "__main__":
    sys.exit(main())
