"""Station 6, gates L1-L7 (station6/GATES.md) on the files the coverage run (station6/coverage.py) converted.

Writes station6/results/<label>/{gates.json, gates.md} and one JSON per check. Exit 0 only if every gate passes.

Usage: python station6/gates.py <opensim_models_dir> <coverage_ours.json> <results_dir> <work_dir> [--no-mc]
  --no-mc: skip the MyoConverter column of L3 (needs coverage_myoconverter.json next to coverage_ours.json)
"""
import json
import os
import subprocess
import sys
import traceback

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, REPO)
from lowlimb_converter.checks import (contact_dynamics, contact_geometry, limit_forces, mass_properties,  # noqa: E402
                                      mujoco_compare, osim_reference, path_forces)
from lowlimb_converter.support import read_osim  # noqa: E402

OUT_OF_SCOPE = {"Models/BouncingBlock/bouncing_block.osim", "Models/BouncingBlock/bouncing_block_weak_spring.osim"}
# L2's gated files (the regression on the four models of Stations 2-5 is station2/run_station2.sh)
L2_GATED = {"Models/WalkerModel/WalkerModel.osim", "Models/WalkerModel/WalkerModelTerrain.osim",
            "Models/SoccerKick/SoccerKickingModel.osim"}
L7_MODEL = "Models/WalkerModel/WalkerModel.osim"


def has(osim, tags):
    root = read_osim(osim).getroot().find("Model")
    return any(root.find(f"ForceSet/objects/{t}") is not None for t in tags)


def safe(fn, *args):
    try:
        return fn(*args)
    except Exception as e:      # recorded as a failure of that check
        return {"error": f"{type(e).__name__}: {e}", "traceback": traceback.format_exc()[-2000:]}


def main():
    osm, cov_path, res, work = (os.path.abspath(p) for p in sys.argv[1:5])
    no_mc = "--no-mc" in sys.argv[5:]
    os.makedirs(work, exist_ok=True)
    cov = json.load(open(cov_path))
    converted = {r["file"]: r["cvt1"] for r in cov["results"] if r["converted"]}
    out = {"L1": {}, "L2": {}, "L3": {}, "L4": {}, "L5": {}, "L6": {}, "L7": {}}

    # L1
    expected = {r["file"] for r in cov["results"]} - OUT_OF_SCOPE
    missing = sorted(expected - set(converted))
    out["L1"] = {"converted": len(converted), "files": len(cov["results"]), "expected": len(expected),
                 "not_converted": missing, "gate": "PASS" if not missing else "FAIL"}
    print(f"L1 {out['L1']['gate']}: {len(converted)}/{len(cov['results'])} converted" + (f", missing {missing}"
                                                                                       if missing else ""))

    # L2: Station 2 on every converted file; gated for L2_GATED, plus the synthetic joint models (test script)
    l2 = {}
    for f, xml in sorted(converted.items()):
        osim = os.path.join(osm, f)
        r = safe(lambda: mujoco_compare.compare(xml, osim_reference.reference(osim), keep_arrays=False))
        entry = {"gated": f in L2_GATED}
        if "error" in r:
            entry.update({"result": "ERROR", "error": r["error"]})
        else:
            ok = r["K0"] == "PASS" and r["K1"] == "PASS" and r["K2"] == "PASS"
            entry.update({"result": "PASS" if ok else "FAIL", "K0": r["K0"],
                          "worst_mm": r["pos_worst_mm"]["value"] if r["K0"] == "PASS" else None,
                          "worst_deg": r["rot_worst_deg"]["value"] if r["K0"] == "PASS" else None,
                          "problems": r.get("K0_problems", [])[:5]})
        l2[f] = entry
        print(f"L2 {f}: {entry['result']}" + (f" ({entry.get('worst_mm', 0):.4f} mm, {entry.get('worst_deg', 0):.4f}"
                                              f" deg)" if entry.get("worst_mm") is not None else
                                              f" {entry.get('error', entry.get('problems'))}"), flush=True)
    syn = subprocess.run([sys.executable, os.path.join(HERE, "tests", "test_joint_types.py"),
                          os.path.join(REPO, "src"), os.path.join(work, "joint_types")], capture_output=True, text=True)
    with open(os.path.join(res, "joint_types.txt"), "w") as fh:
        fh.write(syn.stdout + syn.stderr)
    gated_l2 = [v["result"] == "PASS" for f, v in l2.items() if v["gated"]]
    out["L2"] = {"files": l2, "synthetic_joint_models": "PASS" if syn.returncode == 0 else "FAIL",
                 "gate": "PASS" if all(gated_l2) and len(gated_l2) == len(L2_GATED) and syn.returncode == 0
                 else "FAIL"}
    print(f"L2 {out['L2']['gate']} (synthetic joint models {out['L2']['synthetic_joint_models']})")

    # L3: every converted file (and the MyoConverter column)
    mc_cvt1 = {}
    mc_path = cov_path.replace("coverage_ours.json", "coverage_myoconverter.json")
    if not no_mc and os.path.isfile(mc_path):
        mc_cvt1 = {r["file"]: r["cvt1"] for r in json.load(open(mc_path))["results"] if r["converted"]}
    l3 = {}
    for f, xml in sorted(converted.items()):
        osim = os.path.join(osm, f)
        r = safe(mass_properties.evaluate, osim, xml)
        entry = {"ours": r.get("L3", "ERROR"), "failing": r.get("failing"), "invalid": r.get("invalid_opensim_inertia"),
                 "error": r.get("error")}
        if f in mc_cvt1:
            rm = safe(mass_properties.evaluate, osim, mc_cvt1[f])
            entry.update({"myoconverter": rm.get("L3", "ERROR"), "myoconverter_failing": rm.get("failing")})
        l3[f] = entry
    out["L3"] = {"files": l3, "gate": "PASS" if all(v["ours"] == "PASS" for v in l3.values()) else "FAIL",
                 "myoconverter_failing_files": sorted(f for f, v in l3.items() if v.get("myoconverter") == "FAIL")}
    print(f"L3 {out['L3']['gate']}: {sum(v['ours'] == 'PASS' for v in l3.values())}/{len(l3)} files; MyoConverter "
          f"fails {len(out['L3']['myoconverter_failing_files'])}")

    # L4, L5, L6: every converted file with the element
    for gate, tags, mod, key in (("L4", ("CoordinateLimitForce",), limit_forces, "L4"),
                                 ("L5", ("PathActuator", "PathSpring"), path_forces, "L5"),
                                 ("L6", ("HuntCrossleyForce",), contact_geometry, "L6")):
        files = {}
        for f, xml in sorted(converted.items()):
            osim = os.path.join(osm, f)
            if not has(osim, tags):
                continue
            r = safe(mod.evaluate, osim, xml)
            with open(os.path.join(res, f"{gate}_{f.replace('/', '__')}.json"), "w") as fh:
                json.dump(r, fh, indent=1)
            files[f] = r.get(key, "ERROR") if "error" not in r else "ERROR"
            print(f"{gate} {f}: {files[f]}", flush=True)
        out[gate] = {"files": files, "gate": "PASS" if files and all(v in ("PASS",) for v in files.values())
                     else "FAIL"}
        print(f"{gate} {out[gate]['gate']}")

    # L7: WalkerModel held-out drops; reported: calibration drops, other contact models' calibration
    r = safe(contact_dynamics.evaluate, os.path.join(osm, L7_MODEL), converted.get(L7_MODEL),
             ("calibration", "held-out"))
    with open(os.path.join(res, "L7_WalkerModel.json"), "w") as fh:
        json.dump(r, fh, indent=1)
    free = safe(contact_dynamics.free_run, os.path.join(osm, L7_MODEL), converted.get(L7_MODEL))
    with open(os.path.join(res, "L7_WalkerModel_unlocked.json"), "w") as fh:
        json.dump(free, fh, indent=1)
    cal = {}
    for f, xml in sorted(converted.items()):
        p = os.path.join(os.path.dirname(xml), "contact_calibration.json")
        if os.path.isfile(p):
            c = json.load(open(p))
            cal[f] = {"failed": c["failed"]} if "failed" in c else \
                {"root_position_rms_mm": c["root_position_rms_mm"], "start_rms_mm": c["start_rms_mm"],
                 "groups": {g: {k: v[k] for k in ("timeconst", "dampratio", "friction")}
                            for g, v in c["groups"].items()}}
    out["L7"] = {"C1": r.get("C1", "ERROR"), "C2": r.get("C2", "ERROR"), "error": r.get("error"),
                 "runs": {n: {k: v[k] for k in ("group", "position_rms_mm", "rotation_rms_deg",
                                                "mujoco_default_position_rms_mm", "mujoco_default_rotation_rms_deg")}
                          for n, v in r.get("runs", {}).items()},
                 "unlocked_walker": {k: v for k, v in free.items() if k != "traceback"},
                 "calibrations": cal,
                 "gate": "PASS" if r.get("C1") == "PASS" and r.get("C2") == "PASS" else "FAIL"}
    print(f"L7 {out['L7']['gate']}: C1 {out['L7']['C1']} C2 {out['L7']['C2']}")

    with open(os.path.join(res, "gates.json"), "w") as fh:
        json.dump(out, fh, indent=1)
    gates = {g: out[g]["gate"] for g in ("L1", "L2", "L3", "L4", "L5", "L6", "L7")}
    print("GATES " + " ".join(f"{g} {v}" for g, v in gates.items()))
    return 0 if all(v == "PASS" for v in gates.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
