"""Station 4, OpenSim side: static muscle forces in every Station 2 pose at several activation levels.

Writes one .npz that the MuJoCo side reads (so the MuJoCo side can run in MuJoCo 2.3.7, which has no OpenSim).
Method and gates: station4/GATES.md. Usage:
    python station4/osim_forces.py model.osim out.npz
"""
import argparse
import os
import sys

import numpy as np
import opensim as osim

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(REPO, "station2"))
sys.path.insert(0, os.path.join(REPO, "station3"))
from osim_reference import build_poses   # noqa: E402  (same poses as Stations 2-3)
from osim_paths import _set, _full_coords  # noqa: E402  (same pose setting as Station 3)

ACTIVATIONS = (0.0, 0.25, 0.5, 1.0)


def reference(osim_path):
    model = osim.Model(osim_path)
    state = model.initSystem()
    names, swept, poses, labels, defaults, couplers = build_poses(model)
    cset = model.getCoordinateSet()
    ms = model.getMuscles()
    muscles = [ms.get(i).getName() for i in range(ms.getSize())]
    fmax = np.array([ms.get(i).getMaxIsometricForce() for i in range(len(muscles))])
    concrete = [getattr(osim, ms.get(i).getConcreteClassName()).safeDownCast(ms.get(i)) for i in range(len(muscles))]
    min_fiber = np.array([c.getMinimumFiberLength() for c in concrete])   # Thelen and Millard both define it
    P, M, A = len(poses), len(muscles), len(ACTIVATIONS)
    force = np.full((A, P, M), np.nan)
    fiber_at = np.full((A, P, M), np.nan)
    held = np.full((A, P, M), np.nan)
    clamped = np.zeros((A, P, M), dtype=bool)     # fiber at its minimum length: no equilibrium (GATES amendment)
    length = np.zeros((P, M))
    qerr = np.zeros(P)
    failures = []
    for p, row in enumerate(poses):
        _set(model, state, cset, names, swept, defaults, couplers, row)
        qe = state.getQErr()
        qerr[p] = max((abs(qe.get(k)) for k in range(qe.size())), default=0.0)
        for i in range(M):
            length[p, i] = ms.get(i).getLength(state)
        for k, a in enumerate(ACTIVATIONS):
            for i in range(M):
                ms.get(i).setActivation(state, a)
            try:
                model.equilibrateMuscles(state)
                model.realizeDynamics(state)
            except Exception as e:  # equilibrium failed for this pose: every muscle's sample left out
                failures.append((p, a, str(e)[:200]))
                continue
            for i in range(M):
                mu = ms.get(i)
                held[k, p, i] = mu.getActivation(state)
                force[k, p, i] = mu.getTendonForce(state)
                fiber_at[k, p, i] = mu.getFiberForceAlongTendon(state)
                clamped[k, p, i] = mu.getFiberLength(state) <= min_fiber[i]
    return dict(coord_names=np.array(names), swept=np.array(swept), labels=np.array(labels),
                coords=_full_coords(poses, names, swept, defaults, couplers), muscles=np.array(muscles),
                fmax=fmax, activations=np.array(ACTIVATIONS), force=force, fiber_along_tendon=fiber_at,
                held_activation=held, clamped=clamped, min_fiber_length=min_fiber, length=length, qerr=qerr, n_failures=np.array(len(failures)),
                failures=np.array([f"{p} {a} {msg}" for p, a, msg in failures]),
                muscle_types=np.array([ms.get(i).getConcreteClassName() for i in range(M)]),
                opensim_version=np.array(osim.__version__))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("osim")
    ap.add_argument("out")
    a = ap.parse_args(argv)
    r = reference(a.osim)
    np.savez_compressed(a.out, **r)
    finite = np.isfinite(r["force"]).mean()
    print(f"{os.path.basename(a.osim)}: {len(r['labels'])} poses, {len(r['muscles'])} muscles "
          f"({', '.join(sorted(set(str(t) for t in r['muscle_types'])))}), activations {[float(x) for x in r['activations']]}, "
          f"finite forces {100 * finite:.2f}%, equilibrium failures {int(r['n_failures'])}, "
          f"fibers at their minimum length {100 * r['clamped'].mean():.2f}% of samples (slack or clamped)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
