"""Station 4, MuJoCo side: static muscle forces in every pose and activation level, compared with OpenSim.

Needs only MuJoCo and NumPy (works with MuJoCo 2.3.7 / NumPy 1.x and MuJoCo 3.x / NumPy 2.x); reads the .npz
written by osim_forces.py. Checks gate F0 and writes per-muscle errors (F1-F2 compare two such files:
station4/gate_f.py). Method and gates: station4/GATES.md. Usage:
    python -m lowlimb_converter.checks.mujoco_forces model.xml forces_reference.npz out.json
    or: python station4/mujoco_forces.py ...
Exit 0 if F0 passes.

Moved here from station4/mujoco_forces.py on 2026-10-11 so the installable tool can use it; that file is now a
wrapper with the same command line.
"""
import argparse
import json
import os
import sys

import mujoco
import numpy as np

from . import mujoco_compare as mc  # same model loading and pose solving as Stations 2-3

RESIDUAL = 1e-9
EQ_TOL = 1e-6        # |tendon force - fiber force along tendon| / Fmax
MIN_FINITE = 0.99


def act_index(m, a):
    """Index of actuator a's activation state in d.act (MuJoCo 2.3.7 and 3.x)."""
    if hasattr(m, "actuator_actadr"):
        return int(m.actuator_actadr[a])
    return a - (m.nu - m.na)            # older layout: stateful actuators come last


def evaluate(xml_path, ref, keep_arrays=False):
    m, _ = mc.load_without_meshes(xml_path)
    d = mujoco.MjData(m)
    names = [str(x) for x in ref["coord_names"]]
    swept = [str(x) for x in ref["swept"]]
    labels = [str(x) for x in ref["labels"]]
    muscles = [str(x) for x in ref["muscles"]]
    acts = [float(x) for x in ref["activations"]]
    fmax = np.asarray(ref["fmax"], dtype=float)
    f0 = []

    aid = []
    for mn in muscles:
        a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, mn)
        if a < 0:
            f0.append(f"muscle {mn}: no MuJoCo actuator")
        elif int(m.actuator_dyntype[a]) != int(mujoco.mjtDyn.mjDYN_MUSCLE):
            f0.append(f"muscle {mn}: actuator is not a MuJoCo muscle")
        aid.append(a)
    from_coord, driven, problems = mc.joint_plan(m, names)
    f0 += problems
    if len(labels) != 1000 + 41 * len(swept) or labels.count("random") != 1000:
        f0.append(f"pose count {len(labels)} != 1000 + 41 x {len(swept)}")
    if float(np.max(ref["qerr"])) > RESIDUAL:
        f0.append(f"OpenSim QErr {float(np.max(ref['qerr'])):.2e} > {RESIDUAL}")
    os_force = np.asarray(ref["force"], dtype=float)
    os_fat = np.asarray(ref["fiber_along_tendon"], dtype=float)
    held = np.asarray(ref["held_activation"], dtype=float)
    # GATES amendment: samples where OpenSim clamps the fiber at its minimum length have no equilibrium; they are
    # left out like non-finite ones, and F0's 99% criteria apply to the rest
    # (corrected the same day: only clamped samples whose tendon and fiber forces disagree, i.e. no equilibrium)
    clamped_any = np.asarray(ref["clamped"], dtype=bool) if "clamped" in ref else np.zeros(os_force.shape, dtype=bool)
    clamped = clamped_any & ~(np.abs(os_force - os_fat) <= EQ_TOL * fmax[None, None, :])
    usable = ~clamped
    n_usable = np.maximum(usable.sum(axis=1), 1)
    finite_frac = (np.isfinite(os_force) & usable).sum(axis=1) / n_usable              # (A, M)
    eq_ok = (np.abs(os_force - os_fat) <= EQ_TOL * fmax[None, None, :]) & usable
    eq_frac = eq_ok.sum(axis=1) / n_usable
    for k, a in enumerate(acts):
        for i, mn in enumerate(muscles):
            if finite_frac[k, i] < MIN_FINITE:
                f0.append(f"{mn} a={a}: OpenSim force finite on only {100 * finite_frac[k, i]:.1f}% of poses")
            if eq_frac[k, i] < MIN_FINITE:
                f0.append(f"{mn} a={a}: tendon/fiber equilibrium residual within tolerance on only "
                          f"{100 * eq_frac[k, i]:.1f}% of poses")
    if f0:
        return {"model": os.path.basename(xml_path), "F0": "FAIL", "F0_problems": f0}

    P, M, A = len(labels), len(muscles), len(acts)
    mj = np.full((A, P, M), np.nan)
    length = np.zeros((P, M))
    max_res = 0.0
    adr = [act_index(m, a) for a in aid]
    for p in range(P):
        values = {n: ref["coords"][p, names.index(n)] for n in names}
        d.qpos[:] = mc.solve_qpos(m, from_coord, driven, values)
        d.qvel[:] = 0
        for k in range(A):
            for i in range(M):
                h = held[k, p, i]
                d.act[adr[i]] = h if np.isfinite(h) else acts[k]
            mujoco.mj_forward(m, d)
            if k == 0:
                res, _ = mc.equality_residual(m, d)
                max_res = max(max_res, res)
                length[p] = d.actuator_length[aid]
            mj[k, p] = -np.asarray(d.actuator_force)[aid]
    if max_res > RESIDUAL:
        f0.append(f"MuJoCo equality residual {max_res:.2e} > {RESIDUAL}")
    if not np.all(np.isfinite(mj)):
        f0.append("non-finite MuJoCo force")

    valid = np.isfinite(os_force) & usable
    err = np.where(valid, np.abs(mj - os_force) / fmax[None, None, :] * 100.0, np.nan)   # % of Fmax
    signed = np.where(valid, (mj - os_force) / fmax[None, None, :] * 100.0, np.nan)
    per_muscle = {}
    for i, mn in enumerate(muscles):
        row = {"fmax_N": float(fmax[i]), "excluded_samples": int((~valid[:, :, i]).sum()),
               "clamped_samples": {f"a{a:g}": int(clamped[k, :, i].sum()) for k, a in enumerate(acts)}}
        for k, a in enumerate(acts):
            e = err[k, :, i][valid[k, :, i]]
            row[f"a{a:g}"] = {"rms_pct": float(np.sqrt(np.mean(e ** 2))), "max_pct": float(e.max()),
                              "mean_signed_pct": float(np.nanmean(signed[k, :, i])),
                              "worst_pose": labels[int(np.nanargmax(err[k, :, i]))]}
        per_muscle[mn] = row
    summary = {}
    for a in acts:
        key = f"a{a:g}"
        r = [v[key]["rms_pct"] for v in per_muscle.values()]
        worst = max(per_muscle, key=lambda mn: per_muscle[mn][key]["rms_pct"])
        summary[key] = {"rms_pct_mean": float(np.mean(r)), "rms_pct_median": float(np.median(r)),
                        "worst_muscle": worst, "worst_rms_pct": per_muscle[worst][key]["rms_pct"],
                        "muscles_within_5pct": int(sum(x <= 5 for x in r))}
    out = {"model": os.path.basename(xml_path), "mujoco_version": mujoco.__version__, "numpy_version": np.__version__,
           "opensim_version": str(ref["opensim_version"]), "poses": P, "muscles": M, "activations": acts,
           "max_equality_residual": max_res, "excluded_samples": int((~valid).sum()),
           "clamped_samples": int(clamped.sum()), "clamped_in_reference": "clamped" in ref,
           "F0": "PASS" if not f0 else "FAIL", "F0_problems": f0, "summary": summary, "per_muscle": per_muscle}
    if keep_arrays:
        out["_force"], out["_length"] = mj, length
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("xml")
    ap.add_argument("ref")
    ap.add_argument("out")
    a = ap.parse_args(argv)
    r = evaluate(a.xml, dict(np.load(a.ref)))
    with open(a.out, "w") as f:
        json.dump(r, f, indent=1)
    if r["F0"] != "PASS":
        print(f"{r['model']}: F0 FAIL: " + "; ".join(r["F0_problems"][:10]))
        return 1
    s = r["summary"]
    print(f"{r['model']}: F0 PASS | RMS force error (% Fmax), mean over muscles: "
          + ", ".join(f"a={k[1:]} {v['rms_pct_mean']:.2f} (worst {v['worst_rms_pct']:.1f} {v['worst_muscle']})"
                      for k, v in s.items()))
    return 0


if __name__ == "__main__":
    sys.exit(main())
