"""Station 5, MuJoCo side: replay the walking trajectory, compare muscle paths and inverse dynamics with OpenSim.

Needs only MuJoCo and NumPy (works with MuJoCo 2.3.7 / NumPy 1.x and MuJoCo 3.x / NumPy 2.x); reads the .npz
written by osim_gait.py. Checks gate W0 and writes per-muscle, per-pair and per-coordinate errors (W1-W3 compare two
such files: station5/gate_w.py). Method and gates: station5/GATES.md. Usage:
    python -m lowlimb_converter.checks.mujoco_gait model.xml gait_reference.npz out.json
Exit 0 if W0 passes.
"""
import argparse
import json
import os
import sys

import mujoco
import numpy as np

from . import mujoco_compare as mc
from .mujoco_paths import tendon_lengths

H = 1e-6
SCORE_MM = 1.0        # a pair is scored if OpenSim's peak |moment arm| along the trajectory is >= 1 mm
RESIDUAL = 1e-9
GATED_PREFIXES = ("hip_", "knee_", "ankle_", "lumbar_")   # W3 coordinates (all rotational in the gait models)


def _dependency_order(driven, from_coord):
    order, done, todo = [], set(from_coord), dict(driven)
    while todo:
        progressed = False
        for j1, (j2, c) in list(todo.items()):
            if j2 >= 0 and j2 not in done:
                continue
            order.append(j1)
            done.add(j1)
            del todo[j1]
            progressed = True
        if not progressed:
            raise RuntimeError("joint equalities form a cycle")
    return order


def motion(m, from_coord, driven, order, coord_index, q, qd, qdd):
    """qpos, qvel, qacc of every joint, and dq_joint/dq_coordinate (njnt x ncoord), for one frame.

    Independent joints take their coordinate's value; each dependent joint y = y0 + p(x - x0) of the joint x driving it,
    so y' = p'(x) x' and y'' = p''(x) x'^2 + p'(x) x'' (chain rule, in dependency order)."""
    qpos, qvel, qacc = m.qpos0.copy(), np.zeros(m.nv), np.zeros(m.nv)
    S = np.zeros((m.njnt, len(coord_index)))
    for j, n in from_coord.items():
        k = coord_index[n]
        qpos[m.jnt_qposadr[j]], qvel[m.jnt_dofadr[j]], qacc[m.jnt_dofadr[j]] = q[k], qd[k], qdd[k]
        S[j, k] = 1.0
    for j1 in order:
        j2, c = driven[j1]
        a1, v1 = m.jnt_qposadr[j1], m.jnt_dofadr[j1]
        if j2 < 0:
            qpos[a1] = m.qpos0[a1] + c[0]
            continue
        a2, v2 = m.jnt_qposadr[j2], m.jnt_dofadr[j2]
        x = qpos[a2] - m.qpos0[a2]
        p = c[0] + c[1] * x + c[2] * x ** 2 + c[3] * x ** 3 + c[4] * x ** 4
        dp = c[1] + 2 * c[2] * x + 3 * c[3] * x ** 2 + 4 * c[4] * x ** 3
        ddp = 2 * c[2] + 6 * c[3] * x + 12 * c[4] * x ** 2
        qpos[a1] = m.qpos0[a1] + p
        qvel[v1] = dp * qvel[v2]
        qacc[v1] = ddp * qvel[v2] ** 2 + dp * qacc[v2]
        S[j1] = dp * S[j2]
    return qpos, qvel, qacc, S


def evaluate(xml_path, ref, keep_arrays=False):
    m, n_mesh = mc.load_without_meshes(xml_path)
    d = mujoco.MjData(m)
    names = [str(x) for x in ref["coord_names"]]
    coord_index = {n: k for k, n in enumerate(names)}
    muscles = [str(x) for x in ref["muscles"]]
    pair_muscle = np.asarray(ref["pair_muscle"], dtype=int)
    pair_coord = [str(x) for x in ref["pair_coord"]]
    times = np.asarray(ref["times"], dtype=float)
    q, qd, qdd = (np.asarray(ref[k], dtype=float) for k in ("q", "qd", "qdd"))
    w0 = []

    A, T = mujoco.mjtObj.mjOBJ_ACTUATOR, mujoco.mjtObj.mjOBJ_TENDON
    J, B = mujoco.mjtObj.mjOBJ_JOINT, mujoco.mjtObj.mjOBJ_BODY
    tid = []
    for mn in muscles:
        a, t = mujoco.mj_name2id(m, A, mn), mujoco.mj_name2id(m, T, f"{mn}_tendon")
        if a < 0 or t < 0:
            w0.append(f"muscle {mn}: actuator or tendon {mn}_tendon missing")
            t = 0
        elif int(m.actuator_trntype[a]) != int(mujoco.mjtTrn.mjTRN_TENDON) or int(m.actuator_trnid[a, 0]) != t:
            w0.append(f"muscle {mn}: actuator does not drive {mn}_tendon")
        tid.append(t)
    tid = np.array(tid)
    from_coord, driven, problems = mc.joint_plan(m, names)
    w0 += problems
    missing = [n for n in names if mujoco.mj_name2id(m, J, n) < 0]
    if missing:
        w0.append(f"OpenSim coordinates without a MuJoCo joint: {missing}")
    extra = [str(x) for x in ref["ik_labels"] if str(x) not in coord_index and str(x) != "time"]
    if extra:
        w0.append(f"IK columns that are not OpenSim coordinates: {extra}")
    if float(np.max(ref["qerr"])) > RESIDUAL:
        w0.append(f"OpenSim QErr {float(np.max(ref['qerr'])):.2e} > {RESIDUAL}")
    ground = mujoco.mj_name2id(m, B, "ground")
    grf_bodies = [mujoco.mj_name2id(m, B, str(b)) for b in ref["grf_body"]]
    if ground < 0 or min(grf_bodies, default=0) < 0:
        w0.append("MuJoCo body 'ground' or a body the ground reaction forces act on is missing")
    if w0:
        return {"model": os.path.basename(xml_path), "W0": "FAIL", "W0_problems": w0}

    order = _dependency_order(driven, from_coord)
    F, C = len(times), len(names)
    length = np.zeros((F, len(muscles)))
    ma = np.zeros((F, len(pair_coord)))
    tau = np.zeros((F, C))
    check_frames = [int(f) for f in ref["check_frames"]]
    ma_10h = np.zeros((len(check_frames), len(pair_coord)))
    coords_used = sorted(set(pair_coord), key=names.index)
    pairs_of = {c: [k for k, pc in enumerate(pair_coord) if pc == c] for c in coords_used}
    max_res = 0.0
    flags = int(m.opt.disableflags)
    for f in range(F):
        qpos, qvel, qacc, S = motion(m, from_coord, driven, order, coord_index, q[f], qd[f], qdd[f])
        values = {n: q[f, coord_index[n]] for n in names}
        # level 1 (and the W0 residual): positions only, constraints on
        m.opt.disableflags = flags
        d.qpos[:] = qpos
        d.qvel[:] = 0
        mujoco.mj_forward(m, d)
        res, _ = mc.equality_residual(m, d)
        max_res = max(max_res, res)
        length[f] = d.ten_length[tid]
        for c in coords_used:
            ks = pairs_of[c]
            for h, store in ((H, ma), (10 * H, ma_10h if f in check_frames else None)):
                if store is None:
                    continue
                row = f if store is ma else check_frames.index(f)
                v = dict(values)
                v[c] = values[c] + h
                lp = tendon_lengths(m, d, mc.solve_qpos(m, from_coord, driven, v), tid[pair_muscle[ks]])
                v[c] = values[c] - h
                lm = tendon_lengths(m, d, mc.solve_qpos(m, from_coord, driven, v), tid[pair_muscle[ks]])
                store[row, ks] = -(lp - lm) / (2 * h)
        # level 2: inverse dynamics with constraints disabled (the motion satisfies every equality exactly)
        m.opt.disableflags = flags | int(mujoco.mjtDisableBit.mjDSBL_CONSTRAINT)
        d.qpos[:], d.qvel[:], d.qacc[:] = qpos, qvel, qacc
        mujoco.mj_inverse(m, d)
        qfrc = np.array(d.qfrc_inverse)
        R, x0 = d.xmat[ground].reshape(3, 3), np.array(d.xpos[ground])
        qext = np.zeros(m.nv)
        for k, b in enumerate(grf_bodies):
            force = R @ np.asarray(ref["grf_force"][f, k], dtype=float)
            torque = R @ np.asarray(ref["grf_torque"][f, k], dtype=float)
            point = x0 + R @ np.asarray(ref["grf_point"][f, k], dtype=float)
            mujoco.mj_applyFT(m, d, force, torque, point, b, qext)
        tau_joint = qfrc - qext
        tau[f] = np.array([tau_joint[m.jnt_dofadr[j]] for j in range(m.njnt)]) @ S
    m.opt.disableflags = flags
    if max_res > RESIDUAL:
        w0.append(f"MuJoCo equality residual {max_res:.2e} > {RESIDUAL}")

    os_check = np.abs(np.asarray(ref["ma_fd_check"]) - np.asarray(ref["ma"])[check_frames])
    frac_os = float(np.mean(os_check <= 1e-5)) if os_check.size else 1.0
    mj_check = np.abs(ma_10h - ma[check_frames])
    frac_mj = float(np.mean(mj_check <= 1e-6)) if mj_check.size else 1.0
    if frac_os < 0.99:
        w0.append(f"OpenSim computeMomentArm vs finite difference: only {100 * frac_os:.2f}% within 1e-5 m")
    if frac_mj < 0.99:
        w0.append(f"MuJoCo finite difference h vs 10h: only {100 * frac_mj:.2f}% within 1e-6 m")
    if not (np.all(np.isfinite(tau)) and np.all(np.isfinite(length)) and np.all(np.isfinite(ma))):
        w0.append("non-finite MuJoCo result")

    ref_len, ref_ma, ref_tau = (np.asarray(ref[k], dtype=float) for k in ("length", "ma", "tau"))
    mass = float(ref["total_mass"])
    lerr = 1000.0 * (length - ref_len)
    maerr = 1000.0 * (ma - ref_ma)
    peak = 1000.0 * np.max(np.abs(ref_ma), axis=0)
    terr = (tau - ref_tau) / mass
    rot = np.asarray(ref["rotational"], dtype=bool)
    per_muscle = {mn: {"length_rms_mm": float(np.sqrt(np.mean(lerr[:, i] ** 2))),
                       "length_max_mm": float(np.max(np.abs(lerr[:, i]))),
                       "length_mean_signed_mm": float(np.mean(lerr[:, i]))} for i, mn in enumerate(muscles)}
    pairs = [{"muscle": muscles[pair_muscle[k]], "coordinate": pair_coord[k], "scored": bool(peak[k] >= SCORE_MM),
              "peak_osim_mm": float(peak[k]), "rms_mm": float(np.sqrt(np.mean(maerr[:, k] ** 2))),
              "max_mm": float(np.max(np.abs(maerr[:, k])))} for k in range(len(pair_coord))]
    per_coord = {}
    for j, n in enumerate(names):
        e = terr[:, j]
        per_coord[n] = {"unit": "N*m/kg" if rot[j] else "N/kg", "gated": n.startswith(GATED_PREFIXES),
                        "rms": float(np.sqrt(np.mean(e ** 2))), "max": float(np.max(np.abs(e))),
                        "mean_signed": float(np.mean(e)),
                        "osim_peak": float(np.max(np.abs(ref_tau[:, j])) / mass),
                        "rms_pct_peak": float(100 * np.sqrt(np.mean(e ** 2)) / (np.max(np.abs(ref_tau[:, j])) / mass))
                        if np.max(np.abs(ref_tau[:, j])) > 0 else None}
    scored = [p for p in pairs if p["scored"]]
    gated = {n: v for n, v in per_coord.items() if v["gated"]}
    worst_c = max(gated, key=lambda n: gated[n]["rms"]) if gated else None
    out = {
        "model": os.path.basename(xml_path), "mujoco_version": mujoco.__version__, "numpy_version": np.__version__,
        "opensim_version": str(ref["opensim_version"]), "frames": F, "time_range": [float(times[0]), float(times[-1])],
        "muscles": len(muscles), "pairs_total": len(pairs), "pairs_scored": len(scored), "mesh_geoms_removed": n_mesh,
        "total_mass_osim_kg": mass, "total_mass_mujoco_kg": float(np.sum(m.body_mass)),
        "gravity_mujoco": [float(v) for v in m.opt.gravity], "max_equality_residual": max_res,
        "w0_osim_fd_within_1e-5_frac": frac_os, "w0_mujoco_h_vs_10h_within_1e-6_frac": frac_mj,
        "W0": "PASS" if not w0 else "FAIL", "W0_problems": w0,
        "summary": {
            "length_rms_mm_mean": float(np.mean([v["length_rms_mm"] for v in per_muscle.values()])),
            "length_rms_mm_worst": max(per_muscle, key=lambda k: per_muscle[k]["length_rms_mm"]),
            "ma_rms_mm_mean": float(np.mean([p["rms_mm"] for p in scored])) if scored else None,
            "ma_rms_mm_worst": max(scored, key=lambda p: p["rms_mm"]) if scored else None,
            "moment_rms_mean_gated": float(np.mean([v["rms"] for v in gated.values()])) if gated else None,
            "moment_rms_worst_gated": {"coordinate": worst_c, **gated[worst_c]} if worst_c else None},
        "per_muscle": per_muscle, "pairs": pairs, "per_coordinate": per_coord}
    if keep_arrays:
        out["_length"], out["_ma"], out["_tau"] = length, ma, tau
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
    if r["W0"] != "PASS":
        print(f"{os.path.basename(a.xml)}: W0 FAIL: " + "; ".join(r["W0_problems"][:5]))
        return 1
    s = r["summary"]
    w = s["moment_rms_worst_gated"]
    print(f"{os.path.basename(a.xml)}: W0 PASS | length RMS mean {s['length_rms_mm_mean']:.3f} mm "
          f"(worst {s['length_rms_mm_worst']}) | moment arm RMS mean {s['ma_rms_mm_mean']:.3f} mm "
          f"(worst {s['ma_rms_mm_worst']['muscle']}/{s['ma_rms_mm_worst']['coordinate']} "
          f"{s['ma_rms_mm_worst']['rms_mm']:.2f}) | joint moment RMS mean {s['moment_rms_mean_gated']:.4f} N*m/kg "
          f"(worst {w['coordinate']} {w['rms']:.4f})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
