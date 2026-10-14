"""Station 3, MuJoCo side: put the converted model in each pose and compare muscle lengths and moment arms.

Needs only MuJoCo and NumPy (works with MuJoCo 2.3.7 / NumPy 1.x and MuJoCo 3.x / NumPy 2.x); reads the .npz
written by osim_paths.py. Checks gate M0 and writes per-pair and per-muscle errors (M1 compares two such files:
station3/gate_m1.py). Method and gates: station3/GATES.md. Usage:
    python -m osim2mujoco.checks.mujoco_paths model.xml paths_reference.npz out.json
    or: python station3/mujoco_paths.py ...
Exit 0 if M0 passes.

Moved here from station3/mujoco_paths.py on 2026-10-11 so the installable tool can use it; that file is now a
wrapper with the same command line.
"""
import argparse
import json
import os
import sys

import mujoco
import numpy as np

from . import mujoco_compare as mc  # same model loading and pose solving as Station 2

H = 1e-6
SCORE_MM = 1.0       # a pair is scored if OpenSim's peak |moment arm| >= 1 mm
RESIDUAL = 1e-9


def tendon_lengths(m, d, qpos, tendon_ids):
    d.qpos[:] = qpos
    mujoco.mj_fwdPosition(m, d)
    return np.array(d.ten_length[tendon_ids])


def evaluate(xml_path, ref, keep_arrays=False):
    m, n_mesh = mc.load_without_meshes(xml_path)
    d = mujoco.MjData(m)
    names = [str(x) for x in ref["coord_names"]]
    swept = [str(x) for x in ref["swept"]]
    labels = [str(x) for x in ref["labels"]]
    muscles = [str(x) for x in ref["muscles"]]
    pair_muscle = np.asarray(ref["pair_muscle"], dtype=int)
    pair_coord = [str(x) for x in ref["pair_coord"]]
    m0 = []

    # muscles: actuator <name> on tendon <name>_tendon
    A, T = mujoco.mjtObj.mjOBJ_ACTUATOR, mujoco.mjtObj.mjOBJ_TENDON
    tid = []
    for mn in muscles:
        a = mujoco.mj_name2id(m, A, mn)
        t = mujoco.mj_name2id(m, T, f"{mn}_tendon")
        if a < 0 or t < 0:
            m0.append(f"muscle {mn}: actuator or tendon {mn}_tendon missing")
            tid.append(0)
            continue
        if int(m.actuator_trntype[a]) != int(mujoco.mjtTrn.mjTRN_TENDON) or int(m.actuator_trnid[a, 0]) != t:
            m0.append(f"muscle {mn}: actuator does not drive {mn}_tendon")
        tid.append(t)
    tid = np.array(tid)
    from_coord, driven, problems = mc.joint_plan(m, names)
    m0 += problems
    if len(labels) != 1000 + 41 * len(swept) or labels.count("random") != 1000:
        m0.append(f"pose count {len(labels)} != 1000 + 41 x {len(swept)}")
    if float(np.max(ref["qerr"])) > RESIDUAL:
        m0.append(f"OpenSim QErr {float(np.max(ref['qerr'])):.2e} > {RESIDUAL}")
    if m0:
        return {"model": os.path.basename(xml_path), "M0": "FAIL", "M0_problems": m0}

    P = len(labels)
    coords_used = sorted(set(pair_coord), key=names.index)
    length = np.zeros((P, len(muscles)))
    ma = np.zeros((P, len(pair_coord)))
    max_res = 0.0
    n_check = int(ref["n_check"])
    ma_10h = np.zeros((n_check, len(pair_coord)))
    pairs_of = {c: [k for k, pc in enumerate(pair_coord) if pc == c] for c in coords_used}
    for p in range(P):
        values = {n: ref["coords"][p, names.index(n)] for n in names}
        mc.set_pose(m, d, from_coord, driven, values)            # full pass: residual check
        res, _ = mc.equality_residual(m, d)
        max_res = max(max_res, res)
        length[p] = d.ten_length[tid]
        for c in coords_used:
            ks = pairs_of[c]
            for h, store in ((H, ma), (10 * H, ma_10h if p < n_check else None)):
                if store is None:
                    continue
                v = dict(values)
                v[c] = values[c] + h
                lp = tendon_lengths(m, d, mc.solve_qpos(m, from_coord, driven, v), tid[pair_muscle[ks]])
                v[c] = values[c] - h
                lm = tendon_lengths(m, d, mc.solve_qpos(m, from_coord, driven, v), tid[pair_muscle[ks]])
                store[p, ks] = -(lp - lm) / (2 * h)
    if max_res > RESIDUAL:
        m0.append(f"MuJoCo equality residual {max_res:.2e} > {RESIDUAL}")

    # M0 self-consistency of both moment-arm methods (first n_check random poses)
    # (station3/GATES.md amendment 2026-10-12: with OpenSim moment arms defined as -dL/dq, OpenSim's check is h vs
    # 10h within 1e-6 m, the same as MuJoCo's)
    definition = str(ref["moment_arm"]) if "moment_arm" in ref else "analytic"
    os_tol = 1e-5 if definition == "analytic" else 1e-6
    os_check = np.abs(np.asarray(ref["ma_fd_check"]) - np.asarray(ref["ma"])[:n_check])
    frac_os = float(np.mean(os_check <= os_tol))
    mj_check = np.abs(ma_10h - ma[:n_check])
    frac_mj = float(np.mean(mj_check <= 1e-6))
    if frac_os < 0.99:
        m0.append(f"OpenSim computeMomentArm vs finite difference: only {100 * frac_os:.2f}% within 1e-5 m"
                  if definition == "analytic" else
                  f"OpenSim -dL/dq h vs 10h: only {100 * frac_os:.2f}% within 1e-6 m")
    if frac_mj < 0.99:
        m0.append(f"MuJoCo finite difference h vs 10h: only {100 * frac_mj:.2f}% within 1e-6 m")

    ref_ma, ref_len = np.asarray(ref["ma"]), np.asarray(ref["length"])
    err = 1000.0 * np.abs(ma - ref_ma)
    lerr = 1000.0 * np.abs(length - ref_len)
    peak = 1000.0 * np.max(np.abs(ref_ma), axis=0)
    analytic_gap = None    # reported, no gate: how far OpenSim's computeMomentArm is from -dL/dq (amendment)
    if definition != "analytic" and "ma_analytic" in ref:
        analytic_gap = 1000.0 * np.abs(np.asarray(ref["ma_analytic"]) - ref_ma)
    pairs = []
    for k in range(len(pair_coord)):
        rms = float(np.sqrt(np.mean(err[:, k] ** 2)))
        pairs.append({"muscle": muscles[pair_muscle[k]], "coordinate": pair_coord[k], "scored": bool(peak[k] >= SCORE_MM),
                      "peak_osim_mm": float(peak[k]), "rms_mm": rms, "max_mm": float(err[:, k].max()),
                      "rms_pct_peak": float(100 * rms / peak[k]) if peak[k] > 0 else None,
                      "worst_pose": labels[int(np.argmax(err[:, k]))]})
        if analytic_gap is not None:
            pairs[-1]["opensim_analytic_vs_length_derivative_rms_mm"] = float(np.sqrt(np.mean(analytic_gap[:, k] ** 2)))
            pairs[-1]["opensim_analytic_vs_length_derivative_max_mm"] = float(analytic_gap[:, k].max())
    per_muscle = {mn: {"length_rms_mm": float(np.sqrt(np.mean(lerr[:, i] ** 2))), "length_max_mm": float(lerr[:, i].max()),
                       "length_mean_signed_mm": float(1000 * np.mean(length[:, i] - ref_len[:, i]))}
                  for i, mn in enumerate(muscles)}
    scored = [q for q in pairs if q["scored"]]
    out = {
        "model": os.path.basename(xml_path), "mujoco_version": mujoco.__version__, "numpy_version": np.__version__,
        "opensim_version": str(ref["opensim_version"]), "poses": P, "muscles": len(muscles),
        "pairs_total": len(pairs), "pairs_scored": len(scored), "mesh_geoms_removed": n_mesh,
        "max_equality_residual": max_res, "opensim_moment_arm": definition,
        ("m0_osim_fd_within_1e-5_frac" if definition == "analytic" else "m0_osim_h_vs_10h_within_1e-6_frac"): frac_os,
        "m0_mujoco_h_vs_10h_within_1e-6_frac": frac_mj,
        "M0": "PASS" if not m0 else "FAIL", "M0_problems": m0,
        "summary": {
            "ma_rms_mm_median": float(np.median([q["rms_mm"] for q in scored])),
            "ma_rms_mm_mean": float(np.mean([q["rms_mm"] for q in scored])),
            "ma_rms_mm_worst": max(scored, key=lambda q: q["rms_mm"]),
            "pairs_within_5pct_peak": sum(q["rms_pct_peak"] <= 5 for q in scored),
            "pairs_within_2mm": sum(q["rms_mm"] <= 2 for q in scored),
            "length_rms_mm_median": float(np.median([v["length_rms_mm"] for v in per_muscle.values()])),
            "length_rms_mm_worst": max(per_muscle.items(), key=lambda kv: kv[1]["length_rms_mm"])[0],
        },
        "pairs": pairs, "per_muscle": per_muscle,
    }
    if keep_arrays:
        out["_ma"], out["_length"] = ma, length
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
    if r["M0"] != "PASS":
        print(f"{r['model']}: M0 FAIL: " + "; ".join(r["M0_problems"]))
        return 1
    s = r["summary"]
    w = s["ma_rms_mm_worst"]
    print(f"{r['model']}: M0 PASS | {r['pairs_scored']} scored pairs | moment-arm RMS median {s['ma_rms_mm_median']:.3f} mm, "
          f"worst {w['rms_mm']:.2f} mm ({w['muscle']} / {w['coordinate']}, {w['rms_pct_peak']:.1f}% of peak) | "
          f"length RMS median {s['length_rms_mm_median']:.3f} mm")
    return 0


if __name__ == "__main__":
    sys.exit(main())
