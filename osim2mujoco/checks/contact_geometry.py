"""Station 6, gate L6: contact geometry (OpenSim ContactSphere / ContactHalfSpace vs the converted MuJoCo geoms).

For every converted contact pair (MuJoCo <pair> named <force>_<geom1>_<geom2>) and every Station 2 pose: the signed
distance between the two geometries from OpenSim's frames (sphere-sphere: centre distance minus both radii;
sphere-half-space: the centre's height along the half-space's outward normal, OpenSim's -x of its frame, minus the
radius) vs the same distance of the two MuJoCo geoms, each placed by its own offset in its MuJoCo body, with the bodies
at OpenSim's body frames in that pose (2026-10-13 amendment: this isolates the contact geometry from the body
kinematics, which gate L2 checks; MuJoCo's own kinematics, mj_geomDistance, is reported). Gate: they agree within
1e-6 m wherever OpenSim's distance is below 0.1 m. Sign convention, per force (amendment: a HuntCrossleyForce's
record sums its pairs): OpenSim's force (at zero speed) is non-zero exactly where at least one of its pairs penetrates
(poses within 1e-9 m of touching are left out).

Usage: python -m osim2mujoco.checks.contact_geometry model.osim converted.xml out.json
"""
import argparse
import json
import sys

import mujoco
import numpy as np
import opensim as osim
from scipy.spatial.transform import Rotation

from osim2mujoco.checks import mujoco_compare as mc
from osim2mujoco.checks.osim_reference import build_poses

TOLERANCE = 1e-6
NEAR = 0.1
TOUCH = 1e-9


def _vec(v):
    return np.array([v.get(i) for i in range(3)])


def _geometry_pose(cg, state):
    """Position (ground) and rotation matrix (ground) of a contact geometry's own frame."""
    X = cg.getFrame().getTransformInGround(state)
    R = np.array([[X.R().get(i, j) for j in range(3)] for i in range(3)])
    p = _vec(X.p())
    loc, ori = _vec(cg.get_location()), _vec(cg.get_orientation())
    return p + R @ loc, R @ Rotation.from_euler("XYZ", ori).as_matrix()


def _signed_distance(a, b, state):
    pa, Ra = _geometry_pose(a, state)
    pb, Rb = _geometry_pose(b, state)
    sa, sb = osim.ContactSphere.safeDownCast(a), osim.ContactSphere.safeDownCast(b)
    if sa is not None and sb is not None:
        return float(np.linalg.norm(pa - pb) - sa.getRadius() - sb.getRadius())
    if sa is None:
        a, b, pa, Ra, pb, Rb, sa, sb = b, a, pb, Rb, pa, Ra, sb, sa
    if osim.ContactHalfSpace.safeDownCast(b) is None or sa is None:
        raise NotImplementedError(f"pair {a.getName()} / {b.getName()}")
    normal = -Rb[:, 0]                 # solid where x > 0: outward normal is -x
    return float(normal @ (pa - pb) - sa.getRadius())


def _body_pose(model, state, name):
    """OpenSim's frame of the body MuJoCo calls `name` (MuJoCo bodies sit at OpenSim's body frames; Station 2)."""
    if name in ("world", "ground") or not model.getBodySet().contains(name):
        return np.zeros(3), np.eye(3)
    X = model.getBodySet().get(name).getTransformInGround(state)
    return _vec(X.p()), np.array([[X.R().get(i, j) for j in range(3)] for i in range(3)])


def _mujoco_geometry_distance(m, g1, g2, poses):
    """Signed distance of MuJoCo geoms g1, g2 placed by their own (MuJoCo) offsets in their bodies, with the bodies at
    `poses` (body id -> (position, rotation) in ground): sphere-sphere, sphere-plane, sphere-box (change 6.8)."""
    def world(g):
        p, R = poses[int(m.geom_bodyid[g])]
        q = np.zeros(9)
        mujoco.mju_quat2Mat(q, m.geom_quat[g])
        return p + R @ m.geom_pos[g], R @ q.reshape(3, 3)
    S, P, BX = (int(mujoco.mjtGeom.mjGEOM_SPHERE), int(mujoco.mjtGeom.mjGEOM_PLANE), int(mujoco.mjtGeom.mjGEOM_BOX))
    t1, t2 = int(m.geom_type[g1]), int(m.geom_type[g2])
    if t1 == S and t2 != S:
        g1, g2, t1, t2 = g2, g1, t2, t1
    c2, R2 = world(g2)
    if t1 == S and t2 == S:
        c1, _ = world(g1)
        return float(np.linalg.norm(c1 - c2) - m.geom_size[g1, 0] - m.geom_size[g2, 0])
    c1, R1 = world(g1)            # g1: plane or box, g2: sphere
    r = m.geom_size[g2, 0]
    if t1 == P:
        return float(R1[:, 2] @ (c2 - c1) - r)
    if t1 == BX:
        h = m.geom_size[g1]
        q = np.abs(R1.T @ (c2 - c1)) - h
        outside = np.linalg.norm(np.maximum(q, 0.0))
        return float((outside if outside > 0 else q.max()) - r)
    raise NotImplementedError(f"geom types {t1}, {t2}")


def evaluate(osim_path, xml_path):
    model = osim.Model(osim_path)
    state = model.initSystem()
    names, swept, poses, labels, defaults, couplers = build_poses(model)
    cset = model.getCoordinateSet()
    cgs = model.getContactGeometrySet()
    hcs = {}
    fs = model.getForceSet()
    for i in range(fs.getSize()):
        h = osim.HuntCrossleyForce.safeDownCast(fs.get(i))
        if h is not None and h.get_appliesForce():
            hcs[h.getName()] = h

    m, _ = mc.load_without_meshes(xml_path)
    d = mujoco.MjData(m)
    from_coord, driven, problems = mc.joint_plan(m, names)
    pairs = []
    for k in range(m.npair):
        pname = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_PAIR, k)
        g1, g2 = int(m.pair_geom1[k]), int(m.pair_geom2[k])
        n1, n2 = (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_GEOM, g) for g in (g1, g2))
        force = next((f for f in hcs if pname == f"{f}_{n1}_{n2}"), None)
        if force is None or not cgs.contains(n1) or not cgs.contains(n2):
            continue          # e.g. MyoConverter's ground-plane pairs: not OpenSim contact
        pairs.append((pname, force, g1, g2, cgs.get(n1), cgs.get(n2)))
    if not pairs:
        return {"L6": "NONE" if not hcs else "FAIL", "problems": problems + (["no converted pairs"] if hcs else []),
                "pairs": {}}

    P = len(poses)
    os_d = np.zeros((P, len(pairs)))
    mj_d = np.zeros((P, len(pairs)))         # MuJoCo geometry at OpenSim's body frames (gated, amendment)
    mj_full = np.zeros((P, len(pairs)))      # MuJoCo's own kinematics, mj_geomDistance (reported)
    os_f = np.zeros((P, len(pairs)), dtype=bool)
    bodies = {int(m.geom_bodyid[g]) for _, _, g1, g2, _, _ in pairs for g in (g1, g2)}
    fromto = np.zeros(6)
    for p, row in enumerate(poses):
        values = dict(defaults)
        values.update(zip(swept, row))
        for dep, ind, cc in couplers:
            x = osim.Vector(len(ind), 0.0)
            for k, n in enumerate(ind):
                x.set(k, values[n])
            values[dep] = cc.getFunction().calcValue(x)
        for n in names:
            c = cset.get(n)
            if not c.getLocked(state):
                c.setValue(state, values[n], False)
                c.setSpeedValue(state, 0.0)
        model.realizeDynamics(state)
        mc.set_pose(m, d, from_coord, driven, values)
        frames = {b: _body_pose(model, state, mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b) or "world")
                  for b in bodies}
        for k, (pname, force, g1, g2, a, b) in enumerate(pairs):
            os_d[p, k] = _signed_distance(a, b, state)
            mj_d[p, k] = _mujoco_geometry_distance(m, g1, g2, frames)
            mj_full[p, k] = mujoco.mj_geomDistance(m, d, g1, g2, 10.0, fromto)
            rec = hcs[force].getRecordValues(state)
            os_f[p, k] = any(abs(rec.get(j)) > 0 for j in range(rec.size()))

    # sign convention, per force (amendment): a HuntCrossleyForce's record is the sum over its pairs, so it must be
    # non-zero exactly where at least one of its pairs penetrates (poses where any of its pairs is within TOUCH of
    # touching are left out)
    sign = {}
    for force in {f for _, f, *_ in pairs}:
        ks = [k for k, (_, f, *_) in enumerate(pairs) if f == force]
        clear = np.all(np.abs(os_d[:, ks]) > TOUCH, axis=1)
        sign[force] = bool(np.all(os_f[clear, ks[0]] == np.any(os_d[clear][:, ks] < 0, axis=1)))
    results = {}
    ok_all = True
    for k, (pname, force, g1, g2, a, b) in enumerate(pairs):
        near = os_d[:, k] < NEAR
        err = np.abs(mj_d[:, k] - os_d[:, k])[near]
        err_full = np.abs(mj_full[:, k] - os_d[:, k])[near]
        max_err = float(err.max()) if err.size else 0.0
        ok = max_err <= TOLERANCE and sign[force]
        ok_all &= ok
        results[pname] = {"force": force, "poses_near": int(near.sum()), "penetrating": int((os_d[:, k] < 0).sum()),
                          "max_abs_error_m": max_err, "sign_convention": sign[force], "pass": bool(ok),
                          "max_abs_error_mujoco_kinematics_m": float(err_full.max()) if err_full.size else 0.0}
    return {"L6": "PASS" if ok_all and not problems else "FAIL", "problems": problems, "poses": P, "pairs": results}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("osim")
    ap.add_argument("xml")
    ap.add_argument("out")
    a = ap.parse_args(argv)
    r = evaluate(a.osim, a.xml)
    with open(a.out, "w") as fh:
        json.dump(r, fh, indent=1)
    for n, v in r["pairs"].items():
        print(f"{n}: {v['poses_near']} poses within {NEAR} m ({v['penetrating']} penetrating), max error "
              f"{v['max_abs_error_m']:.2e} m, sign {'ok' if v['sign_convention'] else 'WRONG'}")
    print(f"L6 {r['L6']}" + (f" {r['problems']}" if r["problems"] else ""))
    return 0 if r["L6"] in ("PASS", "NONE") else 1


if __name__ == "__main__":
    sys.exit(main())
