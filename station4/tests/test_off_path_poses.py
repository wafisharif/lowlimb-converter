"""Where the hip flexors miss F1/F2 after change 5.4: only in poses where OpenSim's own path jumps.

Usage: python station4/tests/test_off_path_poses.py <model> <model.osim> <forces_reference.npz> <ours_cvt3.xml>
           <out.json> [python_mujoco237]
  model: gait10dof18musc (iliopsoas) or gait2354 (iliacus, psoas)
  forces_reference.npz: written by station4/osim_forces.py for that model (the Station 2 poses)
  ours_cvt3.xml: the converted model being gated (e.g. <work>/<label>/<model>/<model>_cvt3.xml)
  python_mujoco237: interpreter with mujoco==2.3.7, to compute MyoConverter's column from its own output (baseline/,
    as in Station 4); without it, MyoConverter's numbers are read from the committed
    station4/<model>/off_path_split.json (computed that way once)

Checks, for each hip flexor on both sides (each has a conditional path point active for hip flexion -90..45 deg):
- OpenSim's path jumps: with every other coordinate at its default, the muscle's length changes by more than 10 mm
  between hip flexion -89.5 and -90.5 degrees (the point switches off), more than 10 times its largest change over
  any other 1-degree step of the hip's range.
- Ours is more than 10 mm from OpenSim's length in every Station 2 pose with hip flexion beyond -90 degrees (there it
  keeps the path with the point on; change 5.4 leaves such poses out of the force refit).
- Static force, over the poses with hip flexion at -90 degrees or more: ours is within MyoConverter's RMS error
  + 0.5% of Fmax (the F1 / F2 tolerance) at full activation and passive.
Reported, not checked: ours' path error over the poses at -90 degrees or more. The numbers (both columns; all poses,
hip >= -90, hip < -90) are written to out.json. Exit 0 if every check passes.
"""
import json
import os
import subprocess
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, REPO)
from osim2mujoco.checks import mujoco_forces as mf   # noqa: E402

EDGE = np.radians(-90.0)
TOL_PCT = 0.5
MUSCLES = {"gait10dof18musc": ("iliopsoas_l", "iliopsoas_r"),
           "gait2354": ("iliacus_l", "iliacus_r", "psoas_l", "psoas_r")}
BASELINE = r'''
import sys, numpy as np
sys.path.insert(0, sys.argv[1])
from osim2mujoco.checks import mujoco_forces as mf
r = mf.evaluate(sys.argv[2], dict(np.load(sys.argv[3])), keep_arrays=True)
np.savez(sys.argv[4], force=r["_force"], length=r["_length"])
'''
fails = 0


def check(ok, msg):
    global fails
    print(("  ok   " if ok else "  FAIL ") + msg)
    fails += 0 if ok else 1


def main():
    name, osim_path, ref_path, ours_xml, out_path = sys.argv[1:6]
    py237 = sys.argv[6] if len(sys.argv) > 6 else None
    ref = dict(np.load(ref_path))
    names = [str(x) for x in ref["coord_names"]]
    muscles = [str(x) for x in ref["muscles"]]
    acts = [float(x) for x in ref["activations"]]
    ours = mf.evaluate(ours_xml, ref, keep_arrays=True)
    base, committed = None, None
    if py237:
        tmp = out_path + ".baseline.npz"
        baseline_xml = os.path.join(REPO, "baseline", name, f"{name}_cvt3.xml")
        subprocess.run([py237, "-c", BASELINE, REPO, baseline_xml, ref_path, tmp], check=True)
        base = dict(np.load(tmp))
        os.remove(tmp)
    else:
        committed = json.load(open(os.path.join(REPO, "station4", name, "off_path_split.json")))

    import opensim
    model = opensim.Model(osim_path)
    state = model.initSystem()
    result = {}
    for mn in MUSCLES[name]:
        side = mn[-1]
        hip = model.getCoordinateSet().get(f"hip_flexion_{side}")
        muscle = model.getMuscles().get(mn)
        print(mn)

        def osim_length(q):
            hip.setValue(state, float(q), False)
            model.realizePosition(state)
            return muscle.getLength(state)
        jump = abs(osim_length(EDGE + np.radians(0.5)) - osim_length(EDGE - np.radians(0.5)))
        sweep = np.radians(np.arange(-120.0, 120.0 + 1e-9, 1.0))
        lengths = np.array([osim_length(q) for q in sweep])
        steps = np.abs(np.diff(lengths))
        across = (sweep[:-1] < EDGE) & (sweep[1:] >= EDGE)
        hip.setValue(state, hip.getDefaultValue(), False)
        check(jump > 0.010 and jump > 10 * steps[~across].max(),
              f"OpenSim's path jumps {1000 * jump:.1f} mm at -90 deg (elsewhere at most "
              f"{1000 * steps[~across].max():.2f} mm per degree)")

        i = muscles.index(mn)
        q = ref["coords"][:, names.index(f"hip_flexion_{side}")]
        inside, beyond = q >= EDGE, q < EDGE
        path_err = np.abs(ours["_length"][:, i] - ref["length"][:, i])
        check(path_err[beyond].min() > 0.010, f"ours {1000 * path_err[beyond].min():.1f}-"
              f"{1000 * path_err[beyond].max():.1f} mm off in all {int(beyond.sum())} poses beyond -90 deg")
        within = float(np.mean(path_err[inside] <= 0.001))
        print(f"  (reported) hip >= -90 deg: ours within 1 mm of OpenSim's length in {100 * within:.1f}% of "
              f"{int(inside.sum())} poses, median {1000 * np.median(path_err[inside]):.2f} mm, "
              f"max {1000 * path_err[inside].max():.1f} mm")
        result[mn] = {"opensim_jump_mm": 1000 * jump, "poses_inside": int(inside.sum()),
                      "poses_beyond": int(beyond.sum()), "path_error_inside_max_mm": 1000 * path_err[inside].max(),
                      "path_error_inside_median_mm": 1000 * float(np.median(path_err[inside])),
                      "path_within_1mm_inside_frac": within}
        for a, key in ((1.0, "a1"), (0.0, "a0")):
            k = acts.index(a)
            target = ref["force"][k, :, i]
            valid = np.isfinite(target)
            rms = {}
            columns = [("ours", ours["_force"])] + ([("myoconverter", base["force"])] if base is not None else [])
            for col, arr in columns:
                e = (arr[k, :, i] - target) / ref["fmax"][i] * 100.0
                rms[col] = {lab: float(np.sqrt(np.mean(e[valid & m] ** 2)))
                            for lab, m in (("all", np.ones_like(valid)), ("hip_ge_-90", inside),
                                           ("hip_lt_-90", beyond))}
            if base is None:
                rms["myoconverter"] = committed[mn][key]["myoconverter"]
            result[mn][key] = rms
            o, b = rms["ours"]["hip_ge_-90"], rms["myoconverter"]["hip_ge_-90"]
            check(o <= b + TOL_PCT, f"{key}: hip >= -90 deg ours {o:.2f}% vs MyoConverter {b:.2f}% of Fmax "
                  f"(all poses {rms['ours']['all']:.2f}% vs {rms['myoconverter']['all']:.2f}%; beyond -90 deg "
                  f"{rms['ours']['hip_lt_-90']:.2f}% vs {rms['myoconverter']['hip_lt_-90']:.2f}%)")
    result["mujoco_version"] = mujoco.__version__
    result["myoconverter_column"] = "computed (MuJoCo 2.3.7)" if base is not None else "committed file"
    with open(out_path, "w") as f:
        json.dump(result, f, indent=1)
    print("ALL CHECKS PASS" if fails == 0 else f"{fails} CHECK(S) FAILED")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
