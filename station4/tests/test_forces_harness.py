"""Station 4: prove the muscle-force check can fail, and cross-check its forces a second way.

Uses gait10dof18musc: the Station 1a cvt3 file committed in station1/ (a fixed fixture) and its OpenSim model.

- The harness's MuJoCo force equals MuJoCo's own muscle functions (mju_muscleGain/mju_muscleBias) evaluated at the
  actuator length, and the converter's vectorised copy of them (force_refit.static_force) agrees with both.
- The activation actually held by OpenSim is the one given to MuJoCo.
- Scaling one muscle's peak force by 10% changes only that muscle's errors.
- Renaming a muscle fails F0.

Usage: python station4/tests/test_forces_harness.py <gait10dof18musc.osim> <scratch_dir>
Exit 0 if every check passes.
"""
import os
import re
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "src"))
from osim2mujoco.checks import mujoco_compare as mc   # noqa: E402
from osim2mujoco.checks import mujoco_forces as mf    # noqa: E402
from osim2mujoco.checks import osim_forces as of      # noqa: E402
from myoconverter.optimization.utils.force_refit import static_force  # noqa: E402

XML = os.path.join(REPO, "station1", "gait10dof18musc", "gait10dof18musc_cvt3.xml")
fails = 0


def check(ok, msg):
    global fails
    print(("  ok   " if ok else "  FAIL ") + msg)
    fails += 0 if ok else 1


def main():
    osim_path, scratch = sys.argv[1], sys.argv[2]
    os.makedirs(scratch, exist_ok=True)
    ref = of.reference(osim_path)
    base = mf.evaluate(XML, ref, keep_arrays=True)
    check(base["F0"] == "PASS", f"unmutated model passes F0 ({base['F0_problems'][:3]})")

    print("Activation copied from OpenSim")
    held = np.asarray(ref["held_activation"])
    check(np.allclose(held[0], 0.01) and np.allclose(held[3], 1.0),
          f"OpenSim holds a = 0.01 (its minimum) when asked for 0, and 1 for 1 (min {held[0].min()}, max {held[0].max()})")

    print("Harness force vs MuJoCo's own muscle functions and the vectorised copy")
    m, _ = mc.load_without_meshes(XML)
    muscles = [str(x) for x in ref["muscles"]]
    worst_mj, worst_np = 0.0, 0.0
    for k in range(len(ref["activations"])):
        for p in range(0, len(ref["labels"]), 7):
            for i, mn in enumerate(muscles):
                a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, mn)
                ln, act = base["_length"][p, i], held[k, p, i]
                prm = np.array(m.actuator_gainprm[a, :9])
                g = mujoco.mju_muscleGain(ln, 0.0, m.actuator_lengthrange[a], m.actuator_acc0[a], prm)
                b = mujoco.mju_muscleBias(ln, m.actuator_lengthrange[a], m.actuator_acc0[a], prm)
                direct = -(act * g + b)
                worst_mj = max(worst_mj, abs(direct - base["_force"][k, p, i]) / ref["fmax"][i])
                mine = static_force(np.array([ln]), np.array([act]), m.actuator_lengthrange[a], prm)[0]
                worst_np = max(worst_np, abs(mine - direct) / ref["fmax"][i])
    check(worst_mj < 1e-9, f"harness = mju_muscleGain/Bias (largest difference {worst_mj:.1e} Fmax)")
    check(worst_np < 1e-9, f"force_refit.static_force = mju_muscleGain/Bias (largest difference {worst_np:.1e} Fmax)")

    print("Mutation: soleus_r peak force +10%")
    text = open(XML).read()
    pat = re.compile(r'(<general name="soleus_r"[^>]*gainprm=")([^"]*)(")')
    mt = pat.search(text)
    vals = mt.group(2).split()
    vals[2] = repr(float(vals[2]) * 1.1)
    mutated = text[:mt.start(2)] + " ".join(vals) + text[mt.end(2):]
    path = os.path.join(scratch, "mut_force.xml")
    open(path, "w").write(mutated)
    r = mf.evaluate(path, ref)
    changed = [mn for mn in r["per_muscle"] if r["per_muscle"][mn] != base["per_muscle"][mn]]
    check(changed == ["soleus_r"], f"only soleus_r changes ({changed})")
    d1 = r["per_muscle"]["soleus_r"]["a1"]["mean_signed_pct"] - base["per_muscle"]["soleus_r"]["a1"]["mean_signed_pct"]
    check(d1 > 0, f"soleus_r's mean signed error at a = 1 goes up ({d1:+.2f}% Fmax)")

    print("Mutation: muscle renamed")
    path = os.path.join(scratch, "mut_name.xml")
    open(path, "w").write(text.replace('<general name="soleus_r"', '<general name="soleus_rX"'))
    r = mf.evaluate(path, ref)
    check(r["F0"] == "FAIL" and any("soleus_r" in q for q in r["F0_problems"]), "F0 fails and names soleus_r")

    print("ALL CHECKS PASS" if fails == 0 else f"{fails} CHECK(S) FAILED")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
