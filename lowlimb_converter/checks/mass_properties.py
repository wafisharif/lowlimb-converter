"""Station 6, gate L3: every body's mass properties, OpenSim vs the compiled MuJoCo model.

For every OpenSim body with a valid inertia (mass > 0, inertia about the centre of mass positive definite and
satisfying the triangle inequality): MuJoCo's compiled mass, centre of mass (body frame) and inertia tensor about the
centre of mass (body frame, rebuilt from body_inertia and body_iquat) must equal OpenSim's within 1e-9 relative
(1e-9 kg; 1e-9 m; 1e-9 times the largest principal moment). Bodies with an invalid OpenSim inertia are listed.
OpenSim's inertia property is about the body's centre of mass (OpenSim 4 Body).

Usage: python -m lowlimb_converter.checks.mass_properties model.osim converted.xml out.json
"""
import argparse
import json
import sys

import mujoco
import numpy as np
import opensim as osim

REL = 1e-9


def _osim_body(b):
    I = b.getInertia()
    mom, prod = I.getMoments(), I.getProducts()
    M = np.array([[mom.get(0), prod.get(0), prod.get(1)],
                  [prod.get(0), mom.get(1), prod.get(2)],
                  [prod.get(1), prod.get(2), mom.get(2)]])
    c = b.getMassCenter()
    return b.getMass(), np.array([c.get(i) for i in range(3)]), M


def valid(mass, M):
    ev = np.linalg.eigvalsh(M)
    return mass > 0 and np.all(ev > 0) and ev[0] + ev[1] >= ev[2] * (1 - 1e-12)


def evaluate(osim_path, xml_path):
    from lowlimb_converter.checks.mujoco_compare import load_without_meshes
    model = osim.Model(osim_path)
    model.initSystem()
    m, _ = load_without_meshes(xml_path)   # every converted body has an explicit inertial: meshes don't matter
    bodies, invalid, problems = {}, [], []
    bs = model.getBodySet()
    for i in range(bs.getSize()):
        b = bs.get(i)
        mass, com, M = _osim_body(b)
        if not valid(mass, M):
            invalid.append(b.getName())
            continue
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b.getName())
        if bid < 0:
            problems.append(f"body {b.getName()} not in the MuJoCo model")
            continue
        R = np.zeros(9)
        mujoco.mju_quat2Mat(R, m.body_iquat[bid])
        R = R.reshape(3, 3)
        M_mj = R @ np.diag(m.body_inertia[bid]) @ R.T
        scale = float(np.max(np.linalg.eigvalsh(M)))
        e_mass = abs(float(m.body_mass[bid]) - mass)
        e_com = float(np.max(np.abs(m.body_ipos[bid] - com)))
        e_I = float(np.max(np.abs(M_mj - M)))
        ok = e_mass <= REL * max(mass, 1.0) and e_com <= REL and e_I <= REL * scale
        bodies[b.getName()] = {"mass": mass, "mass_error": e_mass, "com_error_m": e_com,
                               "inertia_error_rel": e_I / scale, "pass": bool(ok),
                               "opensim_principal": np.sort(np.linalg.eigvalsh(M)).tolist(),
                               "mujoco_principal": np.sort(m.body_inertia[bid]).tolist()}
    ok_all = not problems and all(v["pass"] for v in bodies.values())
    return {"L3": "PASS" if ok_all else "FAIL", "problems": problems, "bodies": bodies,
            "invalid_opensim_inertia": invalid, "failing": sorted(n for n, v in bodies.items() if not v["pass"])}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("osim")
    ap.add_argument("xml")
    ap.add_argument("out")
    a = ap.parse_args(argv)
    r = evaluate(a.osim, a.xml)
    with open(a.out, "w") as fh:
        json.dump(r, fh, indent=1)
    print(f"L3 {r['L3']}: {len(r['bodies'])} bodies checked, failing {r['failing']}, invalid OpenSim inertia "
          f"{r['invalid_opensim_inertia']}" + (f" {r['problems']}" if r["problems"] else ""))
    return 0 if r["L3"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
