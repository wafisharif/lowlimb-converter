"""Station 6, gate L5: PathActuators (other than muscles) and PathSprings in OpenSim vs MuJoCo.

Poses: Station 2's (osim2mujoco.checks.osim_reference.build_poses). For every such force and every pose:
- path length: OpenSim's GeometryPath length vs the MuJoCo tendon's length;
- generalized force on every coordinate the path spans (Station 3's rule): OpenSim: tension times the moment arm
  -dL/dq (central difference, h = 1e-6, couplers followed), with tension = optimal_force (PathActuator at control 1) or
  OpenSim's own PathSpring tension at zero speed. MuJoCo: the force MuJoCo actually applies (qfrc_actuator of this
  actuator alone at control 1; qfrc_spring with every other spring's stiffness set to zero), projected onto the
  coordinate by virtual work (the sum over its joints of dq_joint/dq_coordinate times the joint's generalized force,
  so dependent joints such as the knee translations count).
Gate: length RMS <= 1 mm and max <= 3 mm; generalized-force RMS error <= 2% of the largest |OpenSim generalized force|
of that element (each coordinate). MyoConverter has no parser for either (the file fails), so it has no column.

Usage: python -m osim2mujoco.checks.path_forces model.osim converted.xml out.json
"""
import argparse
import json
import sys

import mujoco
import numpy as np
import opensim as osim

from osim2mujoco.checks import mujoco_compare as mc
from osim2mujoco.checks.osim_paths import spanned_coordinates
from osim2mujoco.checks.osim_reference import build_poses

H = 1e-6
LENGTH_RMS_MM, LENGTH_MAX_MM, FORCE_RMS_PCT = 1.0, 3.0, 2.0


def _path_forces(model):
    out = []
    fs = model.getForceSet()
    for i in range(fs.getSize()):
        f = fs.get(i)
        if osim.Muscle.safeDownCast(f) is not None or not f.get_appliesForce():
            continue
        pa, ps = osim.PathActuator.safeDownCast(f), osim.PathSpring.safeDownCast(f)
        if pa is not None:
            out.append(("PathActuator", pa))
        elif ps is not None:
            out.append(("PathSpring", ps))
    return out


def _set(model, state, cset, names, values, couplers):
    v = dict(values)
    for d, ind, cc in couplers:
        x = osim.Vector(len(ind), 0.0)
        for k, n in enumerate(ind):
            x.set(k, v[n])
        v[d] = cc.getFunction().calcValue(x)
    for n in names:
        c = cset.get(n)
        if not c.getLocked(state):
            c.setValue(state, v[n], False)
            c.setSpeedValue(state, 0.0)
    model.realizePosition(state)
    return v


def evaluate(osim_path, xml_path):
    model = osim.Model(osim_path)
    state = model.initSystem()
    names, swept, poses, labels, defaults, couplers = build_poses(model)
    cset = model.getCoordinateSet()
    forces = _path_forces(model)
    if not forces:
        return {"L5": "NONE", "problems": [], "forces": {}}

    m, _ = mc.load_without_meshes(xml_path)
    d = mujoco.MjData(m)
    from_coord, driven, problems = mc.joint_plan(m, names)
    results = {}
    for kind, f in forces:
        name = f.getName()
        geometry = f.getGeometryPath()
        span = spanned_coordinates(model, f, swept, couplers)
        t = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, f"{name}_tendon")
        a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name) if kind == "PathActuator" else -1
        if t < 0 or (kind == "PathActuator" and a < 0):
            problems.append(f"{name}: tendon or actuator missing in the MuJoCo model")
            continue
        P = len(poses)
        L_os, L_mj = np.zeros(P), np.zeros(P)
        G_os, G_mj = np.zeros((P, len(span))), np.zeros((P, len(span)))
        stiff, jstiff = m.tendon_stiffness.copy(), m.jnt_stiffness.copy()
        if kind == "PathSpring":
            m.tendon_stiffness[:] = 0.0
            m.tendon_stiffness[t] = stiff[t]
            m.jnt_stiffness[:] = 0.0
        for p, row in enumerate(poses):
            values = dict(defaults)
            values.update(zip(swept, row))
            # OpenSim
            _set(model, state, cset, names, values, couplers)
            L_os[p] = geometry.getLength(state)
            if kind == "PathActuator":
                tension = f.getOptimalForce()
            else:
                model.realizeVelocity(state)        # PathSpring's tension needs the (zero) lengthening speed
                tension = f.getTension(state)
            for k, c in enumerate(span):
                ls = []
                for sgn in (1, -1):
                    v = dict(values)
                    v[c] += sgn * H
                    _set(model, state, cset, names, v, couplers)
                    ls.append(geometry.getLength(state))
                G_os[p, k] = tension * (-(ls[0] - ls[1]) / (2 * H))
            # MuJoCo: the force it applies, projected on each coordinate by virtual work
            q0 = mc.solve_qpos(m, from_coord, driven, values)
            d.qpos[:] = q0
            d.qvel[:] = 0.0
            d.ctrl[:] = 0.0
            if d.act.size:
                d.act[:] = 0.0
            mujoco.mj_forward(m, d)
            L_mj[p] = d.ten_length[t]
            if kind == "PathActuator":
                # this actuator's own generalized force: control 1 minus control 0 (other actuators unchanged)
                base = d.qfrc_actuator.copy()
                d.ctrl[a] = 1.0
                mujoco.mj_forward(m, d)
                qf = d.qfrc_actuator - base
            else:
                qf = d.qfrc_spring.copy()
            for k, c in enumerate(span):
                v = dict(values)
                v[c] += H
                qp = mc.solve_qpos(m, from_coord, driven, v)
                v[c] -= 2 * H
                qm = mc.solve_qpos(m, from_coord, driven, v)
                dq = np.zeros(m.nv)
                mujoco.mj_differentiatePos(m, dq, 2 * H, qm, qp)
                G_mj[p, k] = float(dq @ qf)
        m.tendon_stiffness[:] = stiff
        m.jnt_stiffness[:] = jstiff

        le = 1000 * np.abs(L_mj - L_os)
        per = {}
        for k, c in enumerate(span):
            peak = float(np.max(np.abs(G_os[:, k])))
            rms = float(np.sqrt(np.mean((G_mj[:, k] - G_os[:, k]) ** 2)))
            per[c] = {"peak_opensim": peak, "rms_error": rms, "rms_pct_peak": 100 * rms / peak if peak > 0 else 0.0,
                      "max_error": float(np.max(np.abs(G_mj[:, k] - G_os[:, k])))}
        ok = (float(np.sqrt(np.mean(le ** 2))) <= LENGTH_RMS_MM and float(le.max()) <= LENGTH_MAX_MM
              and all(v["rms_pct_peak"] <= FORCE_RMS_PCT for v in per.values()))
        results[name] = {"kind": kind, "poses": P, "length_rms_mm": float(np.sqrt(np.mean(le ** 2))),
                         "length_max_mm": float(le.max()), "worst_length_pose": labels[int(np.argmax(le))],
                         "generalized_force": per, "pass": bool(ok)}
    gate = "PASS" if results and not problems and all(r["pass"] for r in results.values()) else "FAIL"
    return {"L5": gate, "problems": problems, "forces": results,
            "limits": {"length_rms_mm": LENGTH_RMS_MM, "length_max_mm": LENGTH_MAX_MM,
                       "force_rms_pct_peak": FORCE_RMS_PCT}}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("osim")
    ap.add_argument("xml")
    ap.add_argument("out")
    a = ap.parse_args(argv)
    r = evaluate(a.osim, a.xml)
    with open(a.out, "w") as fh:
        json.dump(r, fh, indent=1)
    for n, v in r["forces"].items():
        worst = max(v["generalized_force"].items(), key=lambda kv: kv[1]["rms_pct_peak"])
        print(f"{n} ({v['kind']}): length RMS {v['length_rms_mm']:.3f} mm, "
              f"max {v['length_max_mm']:.3f} mm; worst force {worst[0]} {worst[1]['rms_pct_peak']:.3f}% of peak -> {'pass' if v['pass'] else 'FAIL'}")
    print(f"L5 {r['L5']}" + (f" {r['problems']}" if r["problems"] else ""))
    return 0 if r["L5"] in ("PASS", "NONE") else 1


if __name__ == "__main__":
    sys.exit(main())
