"""Station 5 level 3, OpenSim side: open-loop forward simulation with recorded muscle excitations (reported only).

From the walking trajectory's state at the start time, the model is simulated with the muscle excitations of
OpenSim's CMC result for the same trial (linearly interpolated, open loop) and the measured ground reaction forces
applied as external loads; nothing else acts (no reserve or residual actuators, no contact). Writes one .npz with
the inputs (initial state, excitations and ground reaction forces on a fixed time grid) and OpenSim's resulting
coordinates, so the MuJoCo side (mujoco_replay.py) can rerun the same experiment, also in MuJoCo 2.3.7.
Method: station5/GATES.md (level 3). Usage:
    python -m osim2mujoco.checks.osim_replay model.osim ik.mot grf.xml controls.sto out.npz [--duration S]
"""
import argparse
import os
import sys

import numpy as np
import opensim as osim

from .osim_gait import prepare_coordinates

DT = 0.001            # grid for the inputs given to MuJoCo and for OpenSim's reported trajectory (s)
DURATION = 1.0        # simulated time (s), or less if the excitations end sooner
ACCURACY = 1e-6       # OpenSim integrator accuracy


def _read_controls(path, muscles):
    table = osim.TimeSeriesTable(path)
    labels = list(table.getColumnLabels())
    missing = [m for m in muscles if m not in labels]
    if missing:
        raise ValueError(f"controls file has no column for {missing}")
    t = np.array(table.getIndependentColumn())
    return t, np.column_stack([np.array(table.getDependentColumn(m).to_numpy()) for m in muscles])


def reference(osim_path, ik_path, grf_path, controls_path, duration=DURATION, muscles_on=True, grf_on=True):
    """Diagnostic variants: muscles_on=False switches every muscle off (skeleton, gravity and ground reaction forces
    only), separating the multibody dynamics from the muscle models; grf_on=False leaves the ground reaction forces
    out (muscles, skeleton and gravity only)."""
    model = osim.Model(osim_path)
    loads = osim.ExternalLoads(grf_path, True)
    if grf_on:
        model.addModelComponent(loads)
    ms = model.getMuscles()
    muscles = [ms.get(i).getName() for i in range(ms.getSize())]
    ct, cx = _read_controls(controls_path, muscles)
    t0 = float(ct[0])
    t1 = min(float(ct[-1]), t0 + duration)
    grid = np.arange(t0, t1 + 1e-12, DT)
    excitation = np.column_stack([np.interp(grid, ct, cx[:, i]) for i in range(len(muscles))])

    controller = osim.PrescribedController()
    for i, mn in enumerate(muscles):
        controller.addActuator(ms.get(i))
        f = osim.PiecewiseLinearFunction()
        for t, x in zip(grid, excitation[:, i]):
            f.addPoint(float(t), float(x))
        controller.prescribeControlForActuator(mn, f)
    model.addController(controller)
    if not muscles_on:
        for i in range(len(muscles)):
            ms.get(i).set_appliesForce(False)
    state = model.initSystem()

    cs = model.getCoordinateSet()
    names = [cs.get(i).getName() for i in range(cs.getSize())]
    splines, _ = prepare_coordinates(model, ik_path)
    x0 = osim.Vector(1, t0)
    q0 = np.array([splines.get(n).calcValue(x0) for n in names])
    qd0 = np.array([splines.get(n).calcDerivative(osim.StdVectorInt([0]), x0) for n in names])
    for j, n in enumerate(names):
        cs.get(n).setValue(state, float(q0[j]), False)
    for j, n in enumerate(names):
        cs.get(n).setSpeedValue(state, float(qd0[j]))
    for i in range(len(muscles)):
        ms.get(i).setActivation(state, float(excitation[0, i]))
    state.setTime(t0)
    model.equilibrateMuscles(state)
    held = np.array([ms.get(i).getActivation(state) for i in range(len(muscles))])

    ext = [osim.ExternalForce.safeDownCast(loads.get(k)) for k in range(loads.getSize())] if grf_on else []
    for e in ext:
        for prop in ("force_expressed_in_body", "point_expressed_in_body"):
            if e.getPropertyByName(prop).toString().strip() != "ground":
                raise NotImplementedError(f"{e.getName()}: {prop} must be ground")
    grf_force, grf_point, grf_torque = (np.zeros((len(grid), len(ext), 3)) for _ in range(3))
    for f, t in enumerate(grid):
        for k, e in enumerate(ext):
            for arr, getter in ((grf_force, e.getForceAtTime), (grf_point, e.getPointAtTime),
                                (grf_torque, e.getTorqueAtTime)):
                v = getter(float(t))
                arr[f, k] = [v.get(0), v.get(1), v.get(2)]

    manager = osim.Manager(model)
    manager.setIntegratorAccuracy(ACCURACY)
    manager.initialize(state)
    q = np.full((len(grid), len(names)), np.nan)
    q[0] = [cs.get(n).getValue(state) for n in names]
    status = "completed"
    for f in range(1, len(grid)):
        try:
            state = manager.integrate(float(grid[f]))
        except Exception as e:                     # an exploding open-loop simulation ends the record here
            status = f"stopped at {grid[f - 1]:.3f} s: {str(e)[:200]}"
            break
        q[f] = [cs.get(n).getValue(state) for n in names]
        if not np.all(np.isfinite(q[f])):
            status = f"non-finite state at {grid[f]:.3f} s"
            break
    return dict(coord_names=np.array(names), muscles=np.array(muscles), times=grid, q0=q0, qd0=qd0,
                excitation=excitation, activation0=held, grf_body=np.array([e.get_applied_to_body() for e in ext]),
                grf_force=grf_force, grf_point=grf_point, grf_torque=grf_torque, q_osim=q,
                rotational=np.array([cs.get(n).getMotionType() == 1 for n in names]),
                status=np.array(status), accuracy=np.array(ACCURACY), muscles_on=np.array(muscles_on),
                grf_on=np.array(grf_on),
                opensim_version=np.array(osim.__version__))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("osim")
    ap.add_argument("ik")
    ap.add_argument("grf")
    ap.add_argument("controls")
    ap.add_argument("out")
    ap.add_argument("--duration", type=float, default=DURATION)
    ap.add_argument("--muscles-off", action="store_true", help="diagnostic: every muscle switched off")
    ap.add_argument("--grf-off", action="store_true", help="diagnostic: no ground reaction forces")
    a = ap.parse_args(argv)
    r = reference(a.osim, a.ik, a.grf, a.controls, a.duration, muscles_on=not a.muscles_off, grf_on=not a.grf_off)
    np.savez_compressed(a.out, **r)
    done = int(np.sum(np.all(np.isfinite(r["q_osim"]), axis=1)))
    print(f"{os.path.basename(a.osim)}: replay {r['times'][0]:.3f}-{r['times'][-1]:.3f} s, {len(r['muscles'])} "
          f"muscles, OpenSim {str(r['status'])} ({done} of {len(r['times'])} samples)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
