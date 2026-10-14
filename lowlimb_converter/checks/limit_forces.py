"""Station 6, gate L4: CoordinateLimitForces in OpenSim vs their conversion in MuJoCo.

For every CoordinateLimitForce (CLF) that applies force: sweep its coordinate (every other coordinate at its default;
every coordinate unlocked in the check's copy of the OpenSim model, 2026-10-13 amendment)
over [lower - 3 transition - 10 deg, upper + 3 transition + 10 deg] (10 mm instead of 10 deg for a translation), 2001
steps, speed 0. OpenSim: the CLF's own generalized force (calcLimitForce). MuJoCo: the generalized spring force on the
coordinate's joint from the CLF's tendons only (every other tendon's and joint's stiffness set to zero in a copy of the
model). Gate: |MuJoCo - OpenSim| <= 1% of the largest |OpenSim force| over the sweep, at every step. Reported: OpenSim's
force at +-1 rad/s (m/s) beyond each limit minus its static force (the damping, which isn't converted).

Usage: python -m lowlimb_converter.checks.limit_forces model.osim converted.xml out.json
"""
import argparse
import json
import re
import sys

import mujoco
import numpy as np
import opensim as osim

N_STEPS = 2001
TOLERANCE = 0.01   # of the largest |OpenSim force| over the sweep


def _load(xml_path):
    from lowlimb_converter.checks.mujoco_compare import load_without_meshes
    return load_without_meshes(xml_path)[0]


def evaluate(osim_path, xml_path):
    model = osim.Model(osim_path)
    # 2026-10-13 amendment: every coordinate unlocked in this copy, so a CLF on a coordinate the file locks
    # (ToyLanding's left knee) is evaluated as a force law too (a locked coordinate can't be swept: OpenSim would
    # report no force)
    cs0 = model.getCoordinateSet()
    for i in range(cs0.getSize()):
        cs0.get(i).setDefaultLocked(False)
    state = model.initSystem()
    cset = model.getCoordinateSet()
    clfs = []
    fs = model.getForceSet()
    for i in range(fs.getSize()):
        f = osim.CoordinateLimitForce.safeDownCast(fs.get(i))
        if f is not None and f.get_appliesForce():
            clfs.append(f)

    m = _load(xml_path)
    d = mujoco.MjData(m)
    results, problems = {}, []
    for f in clfs:
        name, cname = f.getName(), f.get_coordinate()
        c = cset.get(cname)
        rotational = c.getMotionType() == 1   # Rotational
        unit = np.pi / 180.0 if rotational else 1.0
        margin = (10.0 * np.pi / 180.0) if rotational else 0.010
        lo = f.get_lower_limit() * unit - 3 * f.get_transition() * unit - margin
        hi = f.get_upper_limit() * unit + 3 * f.get_transition() * unit + margin
        qs = np.linspace(lo, hi, N_STEPS)

        # OpenSim
        for k in range(cset.getSize()):
            cc = cset.get(k)
            if not cc.getLocked(state):
                cc.setValue(state, cc.getDefaultValue(), False)
                cc.setSpeedValue(state, 0.0)
        os_force = np.zeros(N_STEPS)
        for k, q in enumerate(qs):
            c.setValue(state, q, False)
            model.realizeDynamics(state)
            os_force[k] = f.calcLimitForce(state)
        damping = {}
        for side, q0 in (("upper", f.get_upper_limit() * unit + 2 * f.get_transition() * unit),
                         ("lower", f.get_lower_limit() * unit - 2 * f.get_transition() * unit)):
            c.setValue(state, q0, False)
            out = []
            for v in (0.0, 1.0, -1.0):
                c.setSpeedValue(state, v)
                model.realizeDynamics(state)
                out.append(f.calcLimitForce(state))
            c.setSpeedValue(state, 0.0)
            damping[side] = {"at": float(q0), "force_static": out[0], "extra_at_plus_1": out[1] - out[0],
                             "extra_at_minus_1": out[2] - out[0]}

        # MuJoCo: only this CLF's tendons keep their stiffness
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, cname)
        pattern = re.compile(re.escape(name) + r"_(upper|lower)_\d+$")   # forces/CoordinateLimitForce.py's names
        mine = [t for t in range(m.ntendon)
                if pattern.match(mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_TENDON, t) or "")]
        if j < 0 or not mine:
            problems.append(f"{name}: joint {cname} or its tendons not found in the MuJoCo model")
            continue
        stiff = m.tendon_stiffness.copy()
        jstiff = m.jnt_stiffness.copy()
        m.tendon_stiffness[:] = 0.0
        m.tendon_stiffness[mine] = stiff[mine]
        m.jnt_stiffness[:] = 0.0
        dof = m.jnt_dofadr[j]
        mj_force = np.zeros(N_STEPS)
        for k, q in enumerate(qs):
            d.qpos[:] = m.qpos0
            d.qpos[m.jnt_qposadr[j]] = q
            d.qvel[:] = 0.0
            mujoco.mj_forward(m, d)
            mj_force[k] = d.qfrc_spring[dof]
        m.tendon_stiffness[:] = stiff
        m.jnt_stiffness[:] = jstiff

        peak = float(np.max(np.abs(os_force)))
        err = np.abs(mj_force - os_force)
        worst = int(np.argmax(err))
        results[name] = {
            "coordinate": cname, "rotational": bool(rotational), "steps": N_STEPS, "range": [float(lo), float(hi)],
            "tendons": len(mine), "peak_opensim": peak, "max_error": float(err.max()),
            "max_error_pct_peak": float(100 * err.max() / peak) if peak > 0 else 0.0,
            "worst_at": float(qs[worst]), "rms_error": float(np.sqrt(np.mean(err ** 2))),
            "myoconverter_max_error_pct_peak": 100.0 if peak > 0 else 0.0,   # it drops CLFs
            "pass": bool(err.max() <= TOLERANCE * peak + 1e-12), "damping_not_converted": damping}
    gate = "PASS" if results and not problems and all(r["pass"] for r in results.values()) else \
        ("NONE" if not clfs else "FAIL")
    return {"L4": gate, "problems": problems, "forces": results, "tolerance_pct_peak": 100 * TOLERANCE}


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
        print(f"{n} ({v['coordinate']}): max error {v['max_error_pct_peak']:.3f}% of peak {v['peak_opensim']:.4g}"
              f" -> {'pass' if v['pass'] else 'FAIL'}")
    print(f"L4 {r['L4']}" + (f" {r['problems']}" if r["problems"] else ""))
    return 0 if r["L4"] in ("PASS", "NONE") else 1


if __name__ == "__main__":
    sys.exit(main())
