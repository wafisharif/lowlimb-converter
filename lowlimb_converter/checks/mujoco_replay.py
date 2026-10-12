"""Station 5 level 3, MuJoCo side: the same open-loop forward simulation as OpenSim, and when the two part ways.

Needs only MuJoCo and NumPy (works with MuJoCo 2.3.7 / NumPy 1.x and MuJoCo 3.x / NumPy 2.x); reads the .npz written
by osim_replay.py. The MuJoCo model starts in the same state (coordinates, speeds, muscle activations as OpenSim
holds them), gets the same muscle excitations (interpolated in time) and the same ground reaction forces (applied at
the same points on the same bodies); contacts are switched off, because the measured forces replace them. Everything
else is the model as converted. Reported only (no gate): the time until any hip, knee or ankle angle differs from
OpenSim's by more than 5 degrees, and the differences along the way, for the model's own time step and integrator
("as converted") and for a fine one (0.0005 s, RK4) that separates integration error from model differences.
An open-loop replay may blow up; MuJoCo then resets the whole state (time included) with a warning, and the replay
stops there (status "unstable at ..."), as it does at a non-finite state.
Method: station5/GATES.md (level 3). Usage:
    python -m lowlimb_converter.checks.mujoco_replay model.xml replay_reference.npz out.json
"""
import argparse
import json
import os
import sys

import mujoco
import numpy as np

from . import mujoco_compare as mc
from .mujoco_gait import _dependency_order, motion

LIMIT_DEG = 5.0
GATED_PREFIXES = ("hip_", "knee_", "ankle_")
FINE_DT = 0.0005


def simulate(xml_path, ref, timestep=None, integrator=None, edit=None):
    """MuJoCo trajectory of every coordinate on the reference time grid (NaN after a failure), and a status string.

    edit: optional function applied to the compiled model first (for diagnostics)."""
    m, _ = mc.load_without_meshes(xml_path)
    if edit is not None:
        edit(m)
    m.opt.disableflags = int(m.opt.disableflags) | int(mujoco.mjtDisableBit.mjDSBL_CONTACT)
    if "muscles_on" in ref and not bool(ref["muscles_on"]):          # diagnostic variant: muscles off on both sides
        m.opt.disableflags = int(m.opt.disableflags) | int(mujoco.mjtDisableBit.mjDSBL_ACTUATION)
    if timestep is not None:
        m.opt.timestep = timestep
    if integrator is not None:
        m.opt.integrator = integrator
    d = mujoco.MjData(m)
    names = [str(x) for x in ref["coord_names"]]
    muscles = [str(x) for x in ref["muscles"]]
    grid = np.asarray(ref["times"], dtype=float)
    exc = np.asarray(ref["excitation"], dtype=float)
    from_coord, driven, problems = mc.joint_plan(m, names)
    if problems:
        raise ValueError("; ".join(problems))
    coord_index = {n: k for k, n in enumerate(names)}
    order = _dependency_order(driven, from_coord)
    qpos, qvel, _, _ = motion(m, from_coord, driven, order, coord_index, np.asarray(ref["q0"], dtype=float),
                              np.asarray(ref["qd0"], dtype=float), np.zeros(len(names)))
    d.qpos[:], d.qvel[:] = qpos, qvel
    act_id = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, mn) for mn in muscles]
    if min(act_id) < 0:
        raise ValueError("a muscle has no MuJoCo actuator")
    adr = [int(m.actuator_actadr[a]) if hasattr(m, "actuator_actadr") else a - (m.nu - m.na) for a in act_id]
    for a, v in zip(adr, np.asarray(ref["activation0"], dtype=float)):
        d.act[a] = v
    d.time = grid[0]
    mujoco.mj_forward(m, d)
    ground = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "ground")
    R, x0 = d.xmat[ground].reshape(3, 3).copy(), d.xpos[ground].copy()
    bodies = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, str(b)) for b in ref["grf_body"]]
    jq = {n: m.jnt_qposadr[j] for j, n in from_coord.items()}
    F = np.asarray(ref["grf_force"], dtype=float)
    P = np.asarray(ref["grf_point"], dtype=float)
    Tq = np.asarray(ref["grf_torque"], dtype=float)

    def inputs(t):
        k = min(max(int(np.floor((t - grid[0]) / (grid[1] - grid[0]))), 0), len(grid) - 2)
        w = (t - grid[k]) / (grid[k + 1] - grid[k])
        w = min(max(w, 0.0), 1.0)
        lerp = lambda a: (1 - w) * a[k] + w * a[k + 1]
        return lerp(exc), lerp(F), lerp(P), lerp(Tq)

    times, rec = [d.time], [np.array([d.qpos[jq[n]] for n in names])]
    status = "completed"
    bad_qacc = int(mujoco.mjtWarning.mjWARN_BADQACC)
    max_steps = int(np.ceil((grid[-1] - grid[0]) / m.opt.timestep)) + 10
    for _ in range(max_steps):
        if d.time >= grid[-1] - 1e-12:
            break
        e, f, p, tq = inputs(d.time)
        d.ctrl[act_id] = np.clip(e, 0.0, 1.0)
        d.xfrc_applied[:] = 0
        for k, b in enumerate(bodies):
            fw, pw, tw = R @ f[k], x0 + R @ p[k], R @ tq[k]
            d.xfrc_applied[b, :3] += fw
            d.xfrc_applied[b, 3:] += tw + np.cross(pw - d.xipos[b], fw)
        t_before = d.time
        mujoco.mj_step(m, d)
        # MuJoCo resets the whole state (time included) when the accelerations blow up, with a warning
        if d.warning[bad_qacc].number > 0 or d.time <= t_before:
            status = f"unstable at {t_before:.3f} s (MuJoCo reset the state)"
            break
        if not np.all(np.isfinite(d.qpos)):
            status = f"non-finite state at {d.time:.3f} s"
            break
        times.append(d.time)
        rec.append(np.array([d.qpos[jq[n]] for n in names]))
    times, rec = np.array(times), np.array(rec)
    out = np.full((len(grid), len(names)), np.nan)
    ok = grid <= times[-1] + 1e-12
    for j in range(len(names)):
        out[ok, j] = np.interp(grid[ok], times, rec[:, j])
    return out, status, float(m.opt.timestep), int(m.opt.integrator)


def compare(ref, q_mj):
    names = [str(x) for x in ref["coord_names"]]
    grid = np.asarray(ref["times"], dtype=float)
    q_os = np.asarray(ref["q_osim"], dtype=float)
    rot = np.asarray(ref["rotational"], dtype=bool)
    gated = [j for j, n in enumerate(names) if n.startswith(GATED_PREFIXES) and rot[j]]
    valid = np.all(np.isfinite(q_os), axis=1) & np.all(np.isfinite(q_mj), axis=1)
    diff = np.degrees(np.abs(q_mj[:, gated] - q_os[:, gated]))
    over = np.where(valid & np.any(diff > LIMIT_DEG, axis=1))[0]
    last_valid = int(np.max(np.where(valid)[0])) if valid.any() else 0
    if len(over):
        f = int(over[0])
        worst = gated[int(np.argmax(diff[f]))]
        t_div, coord = float(grid[f] - grid[0]), names[worst]
    else:
        t_div, coord = None, None
    marks = {}
    for dt in (0.05, 0.1, 0.2, 0.5):
        f = int(np.argmin(np.abs(grid - (grid[0] + dt))))
        if abs(grid[f] - grid[0] - dt) < 1e-6 and valid[f]:
            marks[f"{dt:g}s"] = {names[j]: float(diff[f, i]) for i, j in enumerate(gated)}
    return {"time_to_5deg_s": t_div, "first_coordinate_over_5deg": coord,
            "compared_until_s": float(grid[last_valid] - grid[0]), "max_diff_deg_at": marks}


def evaluate(xml_path, ref):
    out = {"model": os.path.basename(xml_path), "mujoco_version": mujoco.__version__,
           "opensim_status": str(ref["status"]), "replay_start_s": float(ref["times"][0]),
           "replay_length_s": float(ref["times"][-1] - ref["times"][0]), "limit_deg": LIMIT_DEG}
    for label, kw in (("as_converted", {}),
                      ("fine_integration", {"timestep": FINE_DT, "integrator": int(mujoco.mjtIntegrator.mjINT_RK4)})):
        q_mj, status, dt, integ = simulate(xml_path, ref, **kw)
        out[label] = {"timestep_s": dt, "integrator": integ, "mujoco_status": status, **compare(ref, q_mj)}
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
    parts = []
    for label in ("as_converted", "fine_integration"):
        v = r[label]
        t = v["time_to_5deg_s"]
        parts.append(f"{label}: " + (f"apart by > {LIMIT_DEG:g} deg after {t:.3f} s ({v['first_coordinate_over_5deg']})"
                                     if t is not None else f"within {LIMIT_DEG:g} deg for all "
                                     f"{v['compared_until_s']:.3f} s compared"))
    print(f"{os.path.basename(a.xml)}: " + " | ".join(parts))
    return 0


if __name__ == "__main__":
    sys.exit(main())
