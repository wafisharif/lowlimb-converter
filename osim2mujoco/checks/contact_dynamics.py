"""Station 6, gate L7: contact dynamics, OpenSim (Hunt-Crossley) vs the converted MuJoCo model.

The model is made rigid: every coordinate locked at its default value except the root joint's (the first joint from
ground with an unlocked translation; for WalkerModel the pelvis's planar joint) and those a coordinate coupler drives
(they follow it), and every force other than the contact forces is switched off. OpenSim: locked coordinates, other
forces' appliesForce false. MuJoCo: the converted model with every non-root joint removed and its body fixed where the
default pose puts it (MjSpec), actuators, joint equalities and fixed tendons on removed joints removed. The rigid model
is dropped from a height h (above the root height at which the lowest contact geometry touches) with an initial root
rotation and horizontal speed, and both are simulated for DURATION s (OpenSim: its default integrator at accuracy 1e-8,
sampled every 1 ms; MuJoCo: the model's own options, linearly interpolated to the same 1 ms grid). Error per run: RMS
over time of the root position difference (mm) and of its rotation (deg).

Scenarios (station6/GATES.md, L7): calibration (the converter may fit to these) h = 0.02 and 0.10 m at rest; held out
h = 0.05; h = 0.05 with rotation +-0.15 rad; h = 0.02 with 0.5 m/s along ground X. Gate C1: every held-out run within
10 mm and 2 deg RMS; C2: mean held-out position RMS <= half of MuJoCo's default contact parameters'.

This harness is written independently of the converter's calibration (src/myoconverter/xml/contact_calibration.py);
station6/tests/test_contacts.py (WalkerModel) and test_more_files.py (DynamicJumperModel) check that both simulate the
OpenSim side alike.

Usage: python -m osim2mujoco.checks.contact_dynamics model.osim converted.xml out.json [--scenarios held-out]
"""
import argparse
import json
import sys

import mujoco
import numpy as np
import opensim as osim

DURATION = 1.0
SAMPLE = 0.001
ACCURACY = 1e-8
SCENARIOS = {
    "calibration": {"cal_h0.02": {"h": 0.02}, "cal_h0.10": {"h": 0.10}},
    "held-out": {"h0.05": {"h": 0.05}, "h0.05_rot+0.15": {"h": 0.05, "rot": 0.15},
                 "h0.05_rot-0.15": {"h": 0.05, "rot": -0.15}, "h0.02_vx0.5": {"h": 0.02, "vx": 0.5}},
}
POSITION_RMS_MM, ROTATION_RMS_DEG, DEFAULT_RATIO = 10.0, 2.0, 0.5
MUJOCO_DEFAULT_CONTACT = {"solref": [0.02, 1.0], "solimp": [0.9, 0.95, 0.001, 0.5, 2.0],
                          "friction": [1.0, 1.0, 0.005, 0.0001, 0.0001]}


# ---------------------------------------------------------------------------------------------------------- OpenSim
def _vec(v):
    return np.array([v.get(i) for i in range(3)])


def root_joint(model):
    """The first joint whose parent is ground and that has a translational coordinate that isn't locked (a joint
    locked in place, like ToyLandingModel's platform, can't be dropped)."""
    js = model.getJointSet()
    for i in range(js.getSize()):
        j = js.get(i)
        if j.getParentFrame().findBaseFrame().getName() != model.getGround().getName():
            continue
        if any(j.get_coordinates(k).getMotionType() == 2 and not j.get_coordinates(k).getDefaultLocked()
               for k in range(j.numCoordinates())):
            return j
    raise NotImplementedError("no root joint with a translational coordinate: the model can't be dropped")


def root_coordinates(model, state):
    """(rotation coordinate or None, horizontal-X translation, vertical translation) names of the root joint, chosen
    by the direction each translation moves the root body in ground (vertical: most against gravity)."""
    j = root_joint(model)
    body = j.getChildFrame().findBaseFrame()
    g = _vec(model.get_gravity())
    up = -g / np.linalg.norm(g)
    trans, rot = {}, []
    for k in range(j.numCoordinates()):
        c = j.get_coordinates(k)
        if c.getMotionType() == 2:
            v0 = c.getValue(state)
            p0 = _vec(body.getPositionInGround(state))
            c.setValue(state, v0 + 1e-3, False)
            model.realizePosition(state)
            dpos = (_vec(body.getPositionInGround(state)) - p0) / 1e-3
            c.setValue(state, v0, False)
            model.realizePosition(state)
            trans[c.getName()] = dpos
        else:
            rot.append(c.getName())
    vertical = max(trans, key=lambda n: trans[n] @ up)
    horizontal = max((n for n in trans if n != vertical), key=lambda n: abs(trans[n][0]), default=None)
    return (rot[0] if len(rot) == 1 else None), horizontal, vertical


def _signed_distance(cg_a, cg_b, state):
    from osim2mujoco.checks.contact_geometry import _signed_distance as sd
    return sd(cg_a, cg_b, state)


class RigidOpenSim:
    def __init__(self, osim_path):
        self.path = osim_path
        model = osim.Model(osim_path)
        self.root = root_joint(model).getName()
        root_coords = {root_joint(model).get_coordinates(k).getName()
                       for k in range(root_joint(model).numCoordinates())}
        cs = model.getCoordinateSet()
        # coordinates a coupler drives follow it (locking them against it leaves OpenSim nothing to assemble)
        cons = model.getConstraintSet()
        coupled = set()
        for i in range(cons.getSize()):
            cc = osim.CoordinateCouplerConstraint.safeDownCast(cons.get(i))
            if cc is not None and cc.get_isEnforced():
                coupled.add(cc.getDependentCoordinateName())
        for i in range(cs.getSize()):
            if cs.get(i).getName() not in root_coords and cs.get(i).getName() not in coupled:
                cs.get(i).setDefaultLocked(True)
        fs = model.getForceSet()
        self.contact_forces = []
        for i in range(fs.getSize()):
            if osim.HuntCrossleyForce.safeDownCast(fs.get(i)) is None:
                fs.get(i).set_appliesForce(False)
            else:
                self.contact_forces.append(fs.get(i).getName())
        self.model = model
        self.state = model.initSystem()
        self.rot, self.horizontal, self.vertical = root_coordinates(model, self.state)
        self.coords = [c for c in (self.rot, self.horizontal, self.vertical) if c is not None]
        self.pairs = self._pairs()

    def _pairs(self):
        from osim2mujoco.support import read_osim     # the file's ContactParameters (geometry names)
        root = read_osim(self.path).getroot().find("Model")
        cgs = self.model.getContactGeometrySet()
        out = []
        for name in self.contact_forces:
            force = root.find(f"ForceSet/objects/HuntCrossleyForce[@name='{name}']")
            for params in force.iter("HuntCrossleyForce__ContactParameters"):
                names = (params.findtext("geometry") or "").split()
                for a in range(len(names)):
                    for b in range(a + 1, len(names)):
                        ga, gb = cgs.get(names[a]), cgs.get(names[b])
                        if ga.getFrame().findBaseFrame().getName() != gb.getFrame().findBaseFrame().getName():
                            if not (osim.ContactHalfSpace.safeDownCast(ga) and osim.ContactHalfSpace.safeDownCast(gb)):
                                out.append((ga, gb))
        return out

    def touch_height(self, rot=0.0, x=None):
        """Vertical root coordinate at which the closest contact pair just touches."""
        cs = self.model.getCoordinateSet()
        s = self.state
        if self.rot:
            cs.get(self.rot).setValue(s, cs.get(self.rot).getDefaultValue() + rot, False)
        y0 = cs.get(self.vertical).getDefaultValue()

        def gaps(y):
            cs.get(self.vertical).setValue(s, y, False)
            self.model.realizePosition(s)
            return np.array([_signed_distance(a, b, s) for a, b in self.pairs])
        # only pairs that raising the root separates (not two parts of the rigid model, e.g. heel and toes)
        moving = np.abs(gaps(y0 + 0.1) - gaps(y0)) > 1e-9

        def gap(y):
            return float(np.min(gaps(y)[moving]))
        # the gap changes linearly with the height (rigid body): two evaluations give it, a third checks it
        g0, g1 = gap(y0), gap(y0 + 0.1)
        y = y0 - g0 * 0.1 / (g1 - g0)
        if abs(gap(y)) > 1e-9:
            y = y - gap(y) * 0.1 / (g1 - g0)
        return y

    def initial(self, scenario):
        cs = self.model.getCoordinateSet()
        q = {n: cs.get(n).getDefaultValue() for n in self.coords}
        rot = scenario.get("rot", 0.0)
        if self.rot:
            q[self.rot] += rot
        q[self.vertical] = self.touch_height(rot) + scenario["h"]
        v = {n: 0.0 for n in self.coords}
        if self.horizontal:
            v[self.horizontal] = scenario.get("vx", 0.0)
        return q, v

    def simulate(self, scenario, duration=DURATION):
        q, v = self.initial(scenario)
        model = self.model
        s = model.initializeState()
        cs = model.getCoordinateSet()
        for n in self.coords:
            cs.get(n).setValue(s, q[n], False)
            cs.get(n).setSpeedValue(s, v[n])
        model.realizeVelocity(s)
        manager = osim.Manager(model)
        manager.setIntegratorAccuracy(ACCURACY)
        s.setTime(0.0)
        manager.initialize(s)
        times = np.arange(0.0, duration + 1e-12, SAMPLE)
        out = np.zeros((len(times), len(self.coords)))
        for k, t in enumerate(times):
            if t > 0:
                s = manager.integrate(float(t))
            out[k] = [cs.get(n).getValue(s) for n in self.coords]
        return times, out, q, v


# ----------------------------------------------------------------------------------------------------------- MuJoCo
def rigid_mujoco(xml_path, osim_path, root_coords, contact=None):
    """The converted model made rigid in its default pose, root joints kept; returns a compiled MjModel.
    contact: optional {"solref", "solimp", "friction"} to set on every pair (the 'MuJoCo defaults' column)."""
    from osim2mujoco.checks import mujoco_compare as mc
    spec = mujoco.MjSpec.from_file(xml_path)
    for mesh in list(spec.meshes):          # meshes don't matter here and need the files
        spec.delete(mesh)
    for g in list(spec.geoms):
        if g.type == mujoco.mjtGeom.mjGEOM_MESH:
            spec.delete(g)
    m0 = spec.compile()
    d0 = mujoco.MjData(m0)
    om = osim.Model(osim_path)
    om.initSystem()
    cset = om.getCoordinateSet()
    names = [cset.get(i).getName() for i in range(cset.getSize())]
    defaults = {n: cset.get(n).getDefaultValue() for n in names}
    from osim2mujoco.checks.osim_reference import _couplers
    for dep, ind, cc in _couplers(om):
        x = osim.Vector(len(ind), 0.0)
        for k, n in enumerate(ind):
            x.set(k, defaults[n])
        defaults[dep] = cc.getFunction().calcValue(x)
    from_coord, driven, _ = mc.joint_plan(m0, names)
    mc.set_pose(m0, d0, from_coord, driven, defaults)
    keep = set(root_coords)
    removed = set()
    for j in list(spec.joints):
        if j.name in keep:
            continue
        body = j.parent
        removed.add(j.name)
        spec.delete(j)
        bid = mujoco.mj_name2id(m0, mujoco.mjtObj.mjOBJ_BODY, body.name)
        pid = int(m0.body_parentid[bid])
        # body pose relative to its parent, in the default pose
        Rp = d0.xmat[pid].reshape(3, 3)
        pos = Rp.T @ (d0.xpos[bid] - d0.xpos[pid])
        qp = np.zeros(4)
        mujoco.mju_negQuat(qp, d0.xquat[pid])
        qrel = np.zeros(4)
        mujoco.mju_mulQuat(qrel, qp, d0.xquat[bid])
        body.pos = pos
        body.alt.type = mujoco.mjtOrientation.mjORIENTATION_QUAT
        body.quat = qrel
    for e in list(spec.equalities):
        if e.name1 in removed or e.name2 in removed or e.type == mujoco.mjtEq.mjEQ_JOINT:
            spec.delete(e)
    for a in list(spec.actuators):
        spec.delete(a)
    for t in list(spec.tendons):
        spec.delete(t)
    for k in list(spec.keys):
        spec.delete(k)
    if contact is not None:
        for p in spec.pairs:
            p.solref = contact["solref"]
            p.solimp = contact["solimp"]
            p.friction = contact["friction"]
    return spec.compile()


def simulate_mujoco(m, coords, q, v, times):
    d = mujoco.MjData(m)
    mujoco.mj_resetData(m, d)
    adr = {}
    for n in coords:
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        adr[n] = (m.jnt_qposadr[j], m.jnt_dofadr[j])
        d.qpos[adr[n][0]] = q[n]
        d.qvel[adr[n][1]] = v[n]
    mujoco.mj_forward(m, d)
    t_mj, out = [0.0], [[d.qpos[adr[n][0]] for n in coords]]
    while d.time < times[-1] - 1e-12:
        mujoco.mj_step(m, d)
        t_mj.append(d.time)
        out.append([d.qpos[adr[n][0]] for n in coords])
    out = np.array(out)
    return np.column_stack([np.interp(times, t_mj, out[:, k]) for k in range(len(coords))])


def errors(coords, rot, a, b):
    """(position RMS mm, rotation RMS deg) between two root trajectories."""
    tr = [k for k, n in enumerate(coords) if n != rot]
    pos = 1000 * np.sqrt(np.mean(np.sum((a[:, tr] - b[:, tr]) ** 2, axis=1)))
    ang = float(np.degrees(np.sqrt(np.mean((a[:, coords.index(rot)] - b[:, coords.index(rot)]) ** 2)))) if rot else 0.0
    return float(pos), ang


def evaluate(osim_path, xml_path, which=("held-out",)):
    rig = RigidOpenSim(osim_path)
    coords = rig.coords
    m = rigid_mujoco(xml_path, osim_path, coords)
    m_default = rigid_mujoco(xml_path, osim_path, coords, MUJOCO_DEFAULT_CONTACT)
    runs = {}
    for group in which:
        for name, sc in SCENARIOS[group].items():
            times, ref, q, v = rig.simulate(sc)
            ours = simulate_mujoco(m, coords, q, v, times)
            dflt = simulate_mujoco(m_default, coords, q, v, times)
            p, r = errors(coords, rig.rot, ours, ref)
            pd, rd = errors(coords, rig.rot, dflt, ref)
            runs[name] = {"group": group, "scenario": sc, "position_rms_mm": p, "rotation_rms_deg": r,
                          "mujoco_default_position_rms_mm": pd, "mujoco_default_rotation_rms_deg": rd,
                          "opensim_final": dict(zip(coords, ref[-1].tolist())),
                          "mujoco_final": dict(zip(coords, ours[-1].tolist()))}
    held = [r for r in runs.values() if r["group"] == "held-out"]
    c1 = c2 = None
    if held:
        c1 = all(r["position_rms_mm"] <= POSITION_RMS_MM and r["rotation_rms_deg"] <= ROTATION_RMS_DEG for r in held)
        mean_ours = float(np.mean([r["position_rms_mm"] for r in held]))
        mean_dflt = float(np.mean([r["mujoco_default_position_rms_mm"] for r in held]))
        c2 = mean_ours <= DEFAULT_RATIO * mean_dflt
    return {"C1": None if c1 is None else ("PASS" if c1 else "FAIL"),
            "C2": None if c2 is None else ("PASS" if c2 else "FAIL"),
            "root": rig.root, "coordinates": coords, "runs": runs,
            "pair_parameters": [{"name": mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_PAIR, k),
                                 "solref": m.pair_solref[k].tolist(), "solimp": m.pair_solimp[k].tolist(),
                                 "friction": m.pair_friction[k].tolist()} for k in range(m.npair)]}


def free_run(osim_path, xml_path, duration=DURATION):
    """Reported only (L7): the whole model, nothing locked, released from its default pose with every force on, in
    both simulators (muscles, if any, unexcited). Returns RMS differences over time per coordinate."""
    from osim2mujoco.checks import mujoco_compare as mc
    model = osim.Model(osim_path)
    s = model.initSystem()
    cs = model.getCoordinateSet()
    names = [cs.get(i).getName() for i in range(cs.getSize())]
    manager = osim.Manager(model)
    manager.setIntegratorAccuracy(ACCURACY)
    s.setTime(0.0)
    manager.initialize(s)
    times = np.arange(0.0, duration + 1e-12, SAMPLE)
    ref = np.zeros((len(times), len(names)))
    for k, t in enumerate(times):
        if t > 0:
            s = manager.integrate(float(t))
        ref[k] = [cs.get(n).getValue(s) for n in names]
    m, _ = mc.load_without_meshes(xml_path)
    d = mujoco.MjData(m)
    from_coord, driven, _ = mc.joint_plan(m, names)
    mc.set_pose(m, d, from_coord, driven, dict(zip(names, ref[0])))
    d.qvel[:] = 0.0
    adr = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in names]
    t_mj, out = [0.0], [d.qpos[adr].copy()]
    while d.time < times[-1] - 1e-12:
        mujoco.mj_step(m, d)
        t_mj.append(d.time)
        out.append(d.qpos[adr].copy())
    out = np.array(out)
    sim = np.column_stack([np.interp(times, t_mj, out[:, k]) for k in range(len(names))])
    per = {}
    for k, n in enumerate(names):
        e = np.sqrt(np.mean((sim[:, k] - ref[:, k]) ** 2))
        per[n] = {"rms": float(np.degrees(e) if cs.get(n).getMotionType() == 1 else 1000 * e),
                  "unit": "deg" if cs.get(n).getMotionType() == 1 else "mm"}
    return {"duration_s": duration, "coordinates": per,
            "opensim_final": dict(zip(names, ref[-1].tolist())), "mujoco_final": dict(zip(names, sim[-1].tolist()))}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("osim")
    ap.add_argument("xml")
    ap.add_argument("out")
    ap.add_argument("--scenarios", nargs="+", default=["held-out"], choices=list(SCENARIOS))
    a = ap.parse_args(argv)
    r = evaluate(a.osim, a.xml, tuple(a.scenarios))
    with open(a.out, "w") as fh:
        json.dump(r, fh, indent=1)
    for n, v in r["runs"].items():
        print(f"{n:16s} ({v['group']}): position RMS {v['position_rms_mm']:.2f} mm, rotation "
              f"{v['rotation_rms_deg']:.2f} deg | MuJoCo defaults {v['mujoco_default_position_rms_mm']:.2f} mm, "
              f"{v['mujoco_default_rotation_rms_deg']:.2f} deg")
    print(f"C1 {r['C1']}  C2 {r['C2']}")
    return 0 if r["C1"] in ("PASS", None) and r["C2"] in ("PASS", None) else 1


if __name__ == "__main__":
    sys.exit(main())
