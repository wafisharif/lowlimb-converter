"""Station 5, OpenSim side: a walking trajectory, the muscle paths along it, and inverse dynamics.

Writes one .npz that the MuJoCo side (mujoco_gait.py) reads, so the MuJoCo side can also run in MuJoCo 2.3.7.
Method and gates: station5/GATES.md. Usage:
    python -m osim2mujoco.checks.osim_gait model.osim ik.mot grf.xml out.npz [--start T] [--end T]

The coordinates are prepared exactly as OpenSim's InverseDynamicsTool prepares them (Storage padded by half its
length, 6 Hz IIR low-pass, degrees to radians, quintic GCV splines); the generalized forces then equal the tool's
bit for bit (station5/tests/test_gait_harness.py), and the same coordinates, speeds and accelerations go to MuJoCo.

Moment arm (2026-10-12, Station 5 amendment for Rajagopal 2016): "analytic" (default) is OpenSim's
`computeMomentArm`; "length-derivative" is -dL/dq of OpenSim's muscle length by central difference (h = 1e-6),
needed where paths wrap over cylinders (see station3/GATES.md, amendment). Finite differences move coupled
coordinates with their constraint (e.g. Rajagopal's knee_angle_beta with knee_angle).

Coupled coordinates (2026-10-13 amendment, station5/GATES.md): their value, speed and acceleration along the trial
come from their constraint and the independent coordinates (chain rule), not from the IK file's own column, which
satisfies the constraint only to IK's accuracy (Rajagopal 2016: QErr 2e-9); and their generalized force from inverse
dynamics is carried to the independent coordinates by the coupler's derivative (`tau`; the solver's own output is kept
as `tau_raw`). Models without couplers are unchanged.
"""
import argparse
import os
import sys

import numpy as np
import opensim as osim

from .osim_paths import spanned_coordinates
from .osim_reference import _couplers

CUTOFF_HZ = 6.0
H = 1e-6           # central-difference step for the moment-arm self-check
N_CHECK = 20       # frames used by the moment-arm self-check (evenly spread)


def prepare_coordinates(model, ik_path, cutoff=CUTOFF_HZ):
    """Filtered coordinates as quintic GCV splines, as InverseDynamicsTool does; returns (splines, storage times)."""
    st = osim.Storage(ik_path)
    st.pad(st.getSize() // 2)
    st.lowpassIIR(cutoff)
    if st.isInDegrees():
        model.getSimbodyEngine().convertDegreesToRadians(st)
    times = osim.ArrayDouble()
    st.getTimeColumn(times)
    return osim.GCVSplineSet(5, st), np.array([times.get(i) for i in range(times.getSize())])


def _u_index(model, state, names):
    """Index in the state's u vector of each coordinate's speed."""
    cs = model.getCoordinateSet()
    out = {}
    for n in names:
        for m in names:
            cs.get(m).setSpeedValue(state, 0.0)
        cs.get(n).setSpeedValue(state, 1.0)
        u = state.getU()
        hits = [k for k in range(u.size()) if u.get(k) == 1.0]
        if len(hits) != 1:
            raise RuntimeError(f"coordinate {n}: can't locate its speed in the state")
        out[n] = hits[0]
        cs.get(n).setSpeedValue(state, 0.0)
    return out


MOMENT_ARMS = ("analytic", "length-derivative")


def _lengths_at(model, state, cs, names, qrow, couplers, coord, delta, idx, ms):
    """Lengths of muscles `idx` with coordinate `coord` moved by delta, coupled coordinates following."""
    values = dict(zip(names, qrow))
    values[coord] += delta
    for d, ind, cc in couplers:
        x = osim.Vector(len(ind), 0.0)
        for k, n in enumerate(ind):
            x.set(k, values[n])
        values[d] = cc.getFunction().calcValue(x)
    for n in names:
        if not cs.get(n).getLocked(state):
            cs.get(n).setValue(state, values[n], False)
    model.realizePosition(state)
    return [ms.get(i).getLength(state) for i in idx]


def _length_derivatives(model, state, cs, names, qrow, couplers, pairs, ms, h):
    """-dL/dq by central difference for every (muscle, coordinate) pair, one coordinate at a time."""
    by_coord = {}
    for i, c in pairs:
        by_coord.setdefault(c, []).append(i)
    out = {}
    for c, idx in by_coord.items():
        lp = _lengths_at(model, state, cs, names, qrow, couplers, c, h, idx, ms)
        lm = _lengths_at(model, state, cs, names, qrow, couplers, c, -h, idx, ms)
        for i, a, b in zip(idx, lp, lm):
            out[(i, c)] = -(a - b) / (2 * h)
    return out


def reference(osim_path, ik_path, grf_path, start=None, end=None, moment_arm="analytic"):
    if moment_arm not in MOMENT_ARMS:
        raise ValueError(f"moment_arm must be one of {MOMENT_ARMS}")
    model = osim.Model(osim_path)
    loads = osim.ExternalLoads(grf_path, True)
    model.addModelComponent(loads)
    ms = model.getMuscles()
    for i in range(ms.getSize()):                  # inverse dynamics excludes muscles, as the pipelines' ID setups
        ms.get(i).set_appliesForce(False)
    state = model.initSystem()
    cs = model.getCoordinateSet()
    names = [cs.get(i).getName() for i in range(cs.getSize())]
    couplers = _couplers(model)
    dependent = {d for d, _, _ in couplers}
    free = [n for n in names if n not in dependent and not cs.get(n).getDefaultLocked()]

    ik_labels = list(osim.TimeSeriesTable(ik_path).getColumnLabels())
    splines, times = prepare_coordinates(model, ik_path)
    lo = times[0] if start is None else start
    hi = times[-1] if end is None else end
    # the IK file's own times within the range (the padded part of the storage lies outside it)
    ik_times = np.array(osim.TimeSeriesTable(ik_path).getIndependentColumn())
    tol = 1e-5                         # the filtered storage's times differ from the file's by round-off (~5e-7 s)
    frames = np.array([t for t in times if lo - tol <= t <= hi + tol and ik_times[0] - tol <= t <= ik_times[-1] + tol])
    in_file = [n for n in names if n in ik_labels]
    missing = [n for n in names if n not in ik_labels]

    def value(n, t, order):
        if n not in in_file:
            return cs.get(n).getDefaultValue() if order == 0 else 0.0
        x = osim.Vector(1, float(t))
        if order == 0:
            return splines.get(n).calcValue(x)
        return splines.get(n).calcDerivative(osim.StdVectorInt([0] * order), x)

    uidx = _u_index(model, state, names)
    solver = osim.InverseDynamicsSolver(model)
    muscles = [ms.get(i).getName() for i in range(ms.getSize())]
    pairs = [(i, c) for i, mn in enumerate(muscles) for c in spanned_coordinates(model, ms.get(i), free, couplers)]
    ext = [osim.ExternalForce.safeDownCast(loads.get(k)) for k in range(loads.getSize())]

    F = len(frames)
    q = np.zeros((F, len(names)))
    qd = np.zeros_like(q)
    qdd = np.zeros_like(q)
    tau = np.zeros_like(q)
    tau_raw = np.zeros_like(q)
    qerr = np.zeros(F)
    length = np.zeros((F, len(muscles)))
    ma = np.zeros((F, len(pairs)))
    ma_analytic = np.zeros((F, len(pairs)))
    grf_force, grf_point, grf_torque = (np.zeros((F, len(ext), 3)) for _ in range(3))
    col = {n: j for j, n in enumerate(names)}
    for f, t in enumerate(frames):
        for j, n in enumerate(names):
            q[f, j], qd[f, j], qdd[f, j] = value(n, t, 0), value(n, t, 1), value(n, t, 2)
        # coupled coordinates follow their constraint exactly (2026-10-13 amendment): the IK file's own column for
        # them (Rajagopal's knee_angle_beta) satisfies it only to IK's accuracy
        for dep, ind, cc in couplers:
            x = osim.Vector(len(ind), 0.0)
            for k, n in enumerate(ind):
                x.set(k, q[f, col[n]])
            fn = cc.getFunction()
            grad = [fn.calcDerivative(osim.StdVectorInt([k]), x) for k in range(len(ind))]
            q[f, col[dep]] = fn.calcValue(x)
            qd[f, col[dep]] = sum(grad[k] * qd[f, col[n]] for k, n in enumerate(ind))
            qdd[f, col[dep]] = sum(grad[k] * qdd[f, col[n]] for k, n in enumerate(ind)) + sum(
                fn.calcDerivative(osim.StdVectorInt([k, l]), x) * qd[f, col[a]] * qd[f, col[b]]
                for k, a in enumerate(ind) for l, b in enumerate(ind))
        for j, n in enumerate(names):
            if not cs.get(n).getLocked(state):
                cs.get(n).setValue(state, q[f, j], False)
        for j, n in enumerate(names):
            cs.get(n).setSpeedValue(state, qd[f, j])
        state.setTime(float(t))
        model.realizeVelocity(state)
        qe = state.getQErr()
        qerr[f] = max((abs(qe.get(k)) for k in range(qe.size())), default=0.0)
        udot = osim.Vector(state.getNU(), 0.0)
        for j, n in enumerate(names):
            udot.set(uidx[n], qdd[f, j])
        gen = solver.solve(state, udot)
        tau[f] = [gen.get(uidx[n]) for n in names]
        tau_raw[f] = tau[f]
        # 2026-10-13 amendment: a coupled coordinate's generalized force is carried to its independent coordinates by
        # the coupler's derivative (virtual work, as the MuJoCo side's chain rule does); its own entry becomes 0
        for dep, ind, cc in couplers:
            x = osim.Vector(len(ind), 0.0)
            for k, n in enumerate(ind):
                x.set(k, q[f, col[n]])
            for k, n in enumerate(ind):
                tau[f, col[n]] += cc.getFunction().calcDerivative(osim.StdVectorInt([k]), x) * tau[f, col[dep]]
            tau[f, col[dep]] = 0.0
        for i in range(len(muscles)):
            length[f, i] = ms.get(i).getLength(state)
        for k, (i, c) in enumerate(pairs):
            ma_analytic[f, k] = ms.get(i).computeMomentArm(state, cs.get(c))
        for k, e in enumerate(ext):
            for arr, getter in ((grf_force, e.getForceAtTime), (grf_point, e.getPointAtTime),
                                (grf_torque, e.getTorqueAtTime)):
                v = getter(float(t))
                arr[f, k] = [v.get(0), v.get(1), v.get(2)]

    if moment_arm == "analytic":
        ma[:] = ma_analytic
    else:
        for f in range(F):
            fd = _length_derivatives(model, state, cs, names, q[f], couplers, pairs, ms, H)
            ma[f] = [fd[p] for p in pairs]

    # moment-arm self-check (as Station 3's M0): analytic -> -dL/dq with h; length-derivative -> the same with 10 h
    check_frames = np.unique(np.linspace(0, F - 1, min(N_CHECK, F)).astype(int))
    check = np.zeros((len(check_frames), len(pairs)))
    h_check = H if moment_arm == "analytic" else 10 * H
    for r, f in enumerate(check_frames):
        fd = _length_derivatives(model, state, cs, names, q[f], couplers, pairs, ms, h_check)
        check[r] = [fd[p] for p in pairs]

    for e in ext:
        for prop in ("force_expressed_in_body", "point_expressed_in_body"):
            if e.getPropertyByName(prop).toString().strip() != "ground":
                raise NotImplementedError(f"{e.getName()}: {prop} must be ground")
    total_mass = model.getTotalMass(state)
    return dict(coord_names=np.array(names), free=np.array(free), dependent=np.array(sorted(dependent)),
                missing_in_file=np.array(missing), ik_labels=np.array(ik_labels), times=frames, q=q, qd=qd,
                qdd=qdd, tau=tau, tau_raw=tau_raw, qerr=qerr, muscles=np.array(muscles),
                pair_muscle=np.array([i for i, _ in pairs]), pair_coord=np.array([c for _, c in pairs]),
                length=length, ma=ma, ma_fd_check=check, check_frames=check_frames, h=np.array(H),
                grf_body=np.array([e.get_applied_to_body() for e in ext]), grf_force=grf_force, grf_point=grf_point,
                grf_torque=grf_torque, total_mass=np.array(total_mass),
                rotational=np.array([cs.get(n).getMotionType() == 1 for n in names]),
                gravity=np.array([model.getGravity().get(k) for k in range(3)]),
                opensim_version=np.array(osim.__version__), cutoff_hz=np.array(CUTOFF_HZ),
                moment_arm=np.array(moment_arm), ma_analytic=ma_analytic)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("osim")
    ap.add_argument("ik")
    ap.add_argument("grf")
    ap.add_argument("out")
    ap.add_argument("--start", type=float)
    ap.add_argument("--end", type=float)
    ap.add_argument("--moment-arm", choices=MOMENT_ARMS, default="analytic",
                    help="OpenSim's computeMomentArm (default), or -dL/dq of OpenSim's length (wrapped paths)")
    a = ap.parse_args(argv)
    r = reference(a.osim, a.ik, a.grf, a.start, a.end, a.moment_arm)
    np.savez_compressed(a.out, **r)
    print(f"{os.path.basename(a.osim)}: {len(r['times'])} frames {r['times'][0]:.4f}-{r['times'][-1]:.4f} s, "
          f"{len(r['coord_names'])} coordinates ({len(r['missing_in_file'])} not in the file), "
          f"{len(r['muscles'])} muscles, {len(r['pair_coord'])} muscle-coordinate pairs, "
          f"max QErr {r['qerr'].max():.1e}, total mass {float(r['total_mass']):.4f} kg")
    return 0


if __name__ == "__main__":
    sys.exit(main())
