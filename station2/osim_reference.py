"""Station 2, OpenSim side: build the poses and record every body's pose in ground.

Needs OpenSim (the modern stack). Writes one .npz that the MuJoCo side reads, so the MuJoCo side can also run in
MuJoCo 2.3.7 (MyoConverter's own output), which has no OpenSim.

Method and gates: station2/GATES.md. Usage:
    python station2/osim_reference.py model.osim out.npz
"""
import argparse
import sys

import numpy as np
import opensim as osim

SEED = 20261006
N_RANDOM = 1000
N_SWEEP = 41


def _couplers(model):
    """Enforced CoordinateCouplerConstraints as (dependent, [independents], constraint)."""
    out = []
    cs = model.getConstraintSet()
    for i in range(cs.getSize()):
        c = cs.get(i)
        cc = osim.CoordinateCouplerConstraint.safeDownCast(c)
        if cc is None:
            raise NotImplementedError(f"constraint {c.getName()} ({c.getConcreteClassName()}): Station 2 only "
                                      "handles CoordinateCouplerConstraint")
        if not cc.get_isEnforced():
            continue
        ind = cc.getIndependentCoordinateNames()
        out.append((cc.getDependentCoordinateName(), [ind.get(k) for k in range(ind.getSize())], cc))
    return out


def build_poses(model):
    """Return (coordinate names, swept names, pose matrix over swept coords, labels, defaults dict, couplers)."""
    cset = model.getCoordinateSet()
    names = [cset.get(i).getName() for i in range(cset.getSize())]
    couplers = _couplers(model)
    dependent = {d for d, _, _ in couplers}
    swept = [n for n in names if n not in dependent and not cset.get(n).getDefaultLocked()]
    lo = np.array([cset.get(n).getRangeMin() for n in swept])
    hi = np.array([cset.get(n).getRangeMax() for n in swept])
    defaults = {n: cset.get(n).getDefaultValue() for n in names}

    rng = np.random.default_rng(SEED)
    poses = [rng.uniform(lo, hi, size=(N_RANDOM, len(swept)))]
    labels = ["random"] * N_RANDOM
    base = np.array([defaults[n] for n in swept])
    for k, n in enumerate(swept):
        block = np.tile(base, (N_SWEEP, 1))
        block[:, k] = np.linspace(lo[k], hi[k], N_SWEEP)
        poses.append(block)
        labels += [f"sweep:{n}"] * N_SWEEP
    return names, swept, np.vstack(poses), labels, defaults, couplers


def reference(osim_path):
    model = osim.Model(osim_path)
    state = model.initSystem()
    names, swept, poses, labels, defaults, couplers = build_poses(model)
    cset = model.getCoordinateSet()
    bset = model.getBodySet()
    bodies = [bset.get(i).getName() for i in range(bset.getSize())]

    full = np.zeros((len(poses), len(names)))
    pos = np.zeros((len(poses), len(bodies), 3))
    rot = np.zeros((len(poses), len(bodies), 3, 3))
    qerr = np.zeros(len(poses))
    col = {n: i for i, n in enumerate(names)}
    for p, row in enumerate(poses):
        values = dict(defaults)
        values.update(zip(swept, row))
        for d, ind, cc in couplers:          # coupled coordinates follow their constraint function
            x = osim.Vector(len(ind), 0.0)
            for k, n in enumerate(ind):
                x.set(k, values[n])
            values[d] = cc.getFunction().calcValue(x)
        for n in names:
            c = cset.get(n)
            if c.getLocked(state):
                # locked coordinates stay where initSystem put them (their default); setValue would refuse
                if abs(c.getValue(state) - values[n]) > 0:
                    raise RuntimeError(f"locked coordinate {n} is not at its default value")
                continue
            c.setValue(state, values[n], False)
        model.realizePosition(state)
        for n in names:
            full[p, col[n]] = cset.get(n).getValue(state)
        qe = state.getQErr()
        qerr[p] = max((abs(qe.get(k)) for k in range(qe.size())), default=0.0)
        for b, bn in enumerate(bodies):
            T = bset.get(bn).getTransformInGround(state)
            pp, R = T.p(), T.R()
            pos[p, b] = [pp.get(k) for k in range(3)]
            rot[p, b] = [[R.get(i, j) for j in range(3)] for i in range(3)]

    # the values OpenSim actually holds must be the ones we asked for
    asked = np.array([[dict(defaults, **dict(zip(swept, row))).get(n) for n in names] for row in poses])
    indep = [col[n] for n in swept]
    if np.max(np.abs(full[:, indep] - asked[:, indep])) > 1e-12:
        raise RuntimeError("OpenSim did not hold the requested coordinate values")

    return dict(coord_names=np.array(names), swept=np.array(swept), coords=full, labels=np.array(labels),
                body_names=np.array(bodies), pos=pos, rot=rot, qerr=qerr,
                dependent=np.array([d for d, _, _ in couplers]),
                locked=np.array([n for n in names if cset.get(n).getDefaultLocked()]),
                opensim_version=np.array(osim.__version__), seed=np.array(SEED))


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("osim")
    ap.add_argument("out")
    a = ap.parse_args(argv)
    r = reference(a.osim)
    np.savez_compressed(a.out, **r)
    print(f"{a.osim}: {len(r['coords'])} poses, {len(r['swept'])} swept, {len(r['body_names'])} bodies, "
          f"dependent {[str(x) for x in r['dependent']]}, locked {[str(x) for x in r['locked']]}, max QErr {r['qerr'].max():.2e}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
