"""Station 4, change 4.4: the static-force refit keeps both ends of the activation range close to their own best fit.

On Rajagopal 2016's XML-step model (cvt1; Step 1 only, a few minutes), refits three muscles with long, compliant
tendons (soleus_r, ehl_r, tibpost_r) with the converter in src_dir and with an older one (old_src_dir, before 4.4):
- with the change: at full activation and at the passive end, the training RMS error is within LEVEL_SLACK of the
  fit made to that level alone, or that level's weight reached MAX_LEVEL_WEIGHT (both ends can't always be had at
  once: a rigid tendon can't follow how an elastic one shifts the force-length curve with activation), or the
  weighted fit failed the largest-error check and the equal-weight fit was kept;
- before it: at least one muscle's full-activation error is more than LEVEL_SLACK above that level's own best fit
  (the equal-weight fit traded it away; Station 4's Rajagopal run1 failed F1 on 11 such muscles);
- the refit is deterministic: run twice, the same parameters to the last bit;
- a muscle that meets both guarantees with weight 1 gets exactly the equal-weight fit's parameters (gait10dof18musc's
  bifemsh_r, refitted on its own cvt1, by both converters).

Usage: python station4/tests/test_level_guard.py <src_dir> <old_src_dir> <opensim_models_dir> <rajagopal_geometry_dir>
         <gait10dof18musc_dir> <scratch_dir>
  gait10dof18musc_dir: MyoConverter's models/osim/Gait10dof18musc (model and Geometry)
"""
import json
import os
import shutil
import subprocess
import sys

RUNNER = r'''
import sys
sys.path.insert(0, sys.argv[4])
from myoconverter.O2MPipeline import O2MPipeline
O2MPipeline(sys.argv[1], sys.argv[2], sys.argv[3], convert_steps=[1], muscle_list=None, osim_data_overwrite=True,
            conversion=True, validation=False, speedy=False, generate_pdf=False, add_ground_geom=True,
            treat_as_normal_path_point=False)
'''
REFIT = r'''
import json, sys
sys.path.insert(0, sys.argv[1])
import myoconverter.xml.config  # noqa: F401
import mujoco, numpy as np, opensim
from myoconverter.optimization.utils import force_refit as fr
m = mujoco.MjModel.from_xml_path(sys.argv[2])
om = opensim.Model(sys.argv[3])
out = {}
for mus in sys.argv[4].split(","):
    aid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, mus)
    m.actuator_gainprm[aid, 2] = 1e-3      # make sure the refit is kept (compare the refits, not Step 3)
    m.actuator_biasprm[aid, 2] = 1e-3
    r = fr.refit_muscle(m, om, mus)
    # the training data again, to score each activation level (the same function the refit uses)
    poses, force, held, rel, _ = fr._opensim_samples(om, mus, 400, 2)
    d = mujoco.MjData(m)
    plan = fr._joint_plan(m)
    L = np.zeros(len(poses))
    for p, pose in enumerate(poses):
        d.qpos[:] = fr._solve_qpos(m, plan, pose)
        mujoco.mj_fwdPosition(m, d)
        L[p] = d.actuator_length[aid]
    fmax = om.getMuscles().get(mus).getMaxIsometricForce()
    prm = np.array(m.actuator_gainprm[aid, :9])
    lr = np.array(m.actuator_lengthrange[aid])
    levels = {}
    for k, a in enumerate(fr.ACTIVATIONS):
        ok = np.isfinite(force[k]) & np.isfinite(held[k])
        e = fr.static_force(L[ok], held[k][ok], lr, prm) / fmax - force[k][ok] / fmax
        levels[str(a)] = float(np.sqrt(np.mean(e ** 2)))
    out[mus] = {"kept": r["kept"], "levels": levels, "prm": prm.tolist(), "lr": lr.tolist(),
                "weights": {str(k): v for k, v in r.get("level_weights", {}).items()},
                "alone": {str(k): v / 100 for k, v in r.get("level_alone_rms_pct", {}).items()},
                "slack": getattr(fr, "LEVEL_SLACK", None), "cap": getattr(fr, "MAX_LEVEL_WEIGHT", None),
                "dropped": bool(r.get("weighting_dropped_by_max_error", False))}
print("RESULT " + json.dumps(out))
'''
fails = 0


def check(ok, msg):
    global fails
    print(("  ok   " if ok else "  FAIL ") + msg)
    fails += 0 if ok else 1


def convert(src, osim, geometry, out):
    subprocess.run([sys.executable, "-c", RUNNER, osim, geometry, out, src], capture_output=True, text=True)
    name = os.path.splitext(os.path.basename(osim))[0]
    xml = os.path.join(out, f"{name}_cvt1.xml")
    return xml if os.path.isfile(xml) else None


def refit(src, xml, osim, muscles):
    r = subprocess.run([sys.executable, "-c", REFIT, src, xml, osim, ",".join(muscles)], capture_output=True,
                       text=True)
    line = [ln for ln in r.stdout.splitlines() if ln.startswith("RESULT ")]
    if not line:
        print(r.stdout[-2000:] + r.stderr[-3000:])
        return None
    return json.loads(line[0][7:])


def main():
    src, old, osm, rgeom, g10, scratch = (os.path.abspath(p) for p in sys.argv[1:7])
    if os.path.exists(scratch):
        shutil.rmtree(scratch)
    os.makedirs(scratch)
    raj = os.path.join(osm, "Models", "Rajagopal", "Rajagopal2016.osim")
    xml = convert(src, raj, rgeom, os.path.join(scratch, "raj"))
    check(xml is not None, "Rajagopal 2016's XML step converts")
    if xml is None:
        return 1
    muscles = ["soleus_r", "ehl_r", "tibpost_r"]
    new = refit(src, xml, raj, muscles)
    again = refit(src, xml, raj, muscles)
    before = refit(old, xml, raj, muscles)
    check(new is not None and again is not None and before is not None, "the refits ran")
    if None in (new, again, before):
        return 1
    print("with change 4.4")
    for mus, r in new.items():
        lv, alone, slack, cap = r["levels"], r["alone"], r["slack"], r["cap"]
        for a in ("0.0", "1.0"):
            met = lv[a] <= alone[a] + slack + 1e-12
            check(met or r["weights"][a] >= cap or r["dropped"],
                  f"{mus} a={a}: {100 * lv[a]:.2f}% Fmax vs alone {100 * alone[a]:.2f}% (+{100 * slack:.2f}%), "
                  f"weight {r['weights'][a]}" + ("" if met else " (equal-weight fit kept: the weighted one failed "
                                                  "the largest-error check)" if r["dropped"] else " (at the cap)"))
    check(all(new[m]["prm"] == again[m]["prm"] and new[m]["lr"] == again[m]["lr"] for m in muscles),
          "deterministic: the same parameters twice")
    print("before change 4.4")
    worse = {m: before[m]["levels"]["1.0"] - new[m]["alone"]["1.0"] for m in muscles}
    for m, w in worse.items():
        print(f"       {m}: full activation {100 * before[m]['levels']['1.0']:.2f}% Fmax, {100 * w:.2f}% above "
              f"the full-activation-only fit")
    check(any(w > new[muscles[0]]["slack"] for w in worse.values()),
          "the equal-weight fit gave up more than LEVEL_SLACK at full activation for at least one muscle")

    print("a muscle that meets both guarantees keeps the equal-weight fit (gait10dof18musc)")
    g_osim = os.path.join(g10, "gait10dof18musc.osim")
    g_xml = convert(src, g_osim, os.path.join(g10, "Geometry"), os.path.join(scratch, "gait10"))
    check(g_xml is not None, "gait10dof18musc's XML step converts")
    if g_xml:
        gn = refit(src, g_xml, g_osim, ["bifemsh_r"])
        go = refit(old, g_xml, g_osim, ["bifemsh_r"])
        if gn and go:
            w = gn["bifemsh_r"]["weights"]
            same = gn["bifemsh_r"]["prm"] == go["bifemsh_r"]["prm"] and gn["bifemsh_r"]["lr"] == go["bifemsh_r"]["lr"]
            check(all(v == 1 for v in w.values()) and same,
                  f"bifemsh_r: weights {w}, parameters identical to the converter before 4.4: {same}")
        else:
            check(False, "gait10 refits ran")
    print("ALL CHECKS PASS" if fails == 0 else f"{fails} CHECK(S) FAILED")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
