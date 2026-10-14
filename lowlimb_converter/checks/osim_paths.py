"""Station 3, OpenSim side: muscle lengths and moment arms in every Station 2 pose.

Writes one .npz that the MuJoCo side reads (so the MuJoCo side can run in MuJoCo 2.3.7, which has no OpenSim).
Method and gates: station3/GATES.md. Usage:
    python -m lowlimb_converter.checks.osim_paths model.osim out.npz
    or: python station3/osim_paths.py ...

Moved here from station3/osim_paths.py on 2026-10-11 so the installable tool can use it; that file is now a
wrapper with the same command line.

Moment arm (2026-10-12, Station 3 amendment): "analytic" (default) is OpenSim's `computeMomentArm`;
"length-derivative" is −dL/dq of OpenSim's muscle length by central difference (h = 1e-6, coupled coordinates
following their constraint), the same definition as the MuJoCo side. For a path that wraps over a cylinder,
`computeMomentArm` is not the derivative of OpenSim's own length (Rajagopal 2016: up to 18 mm off), so the
length-derivative is used there; the analytic value is still stored (`ma_analytic`) and reported. With
length-derivative, the M0 self-check compares h with 10h (as MuJoCo's), stored in `ma_fd_check`.
    python -m lowlimb_converter.checks.osim_paths model.osim out.npz --moment-arm length-derivative
"""
import argparse
import os
import sys

import numpy as np
import opensim as osim

from .osim_reference import build_poses  # same poses as Station 2

H = 1e-6           # central-difference step for the M0 self-check
N_CHECK = 50       # random poses used by the M0 self-check


def _tree(model):
    """child body -> (parent body, joint) for every joint."""
    parent = {}
    js = model.getJointSet()
    for i in range(js.getSize()):
        j = js.get(i)
        child = j.getChildFrame().findBaseFrame().getName()
        par = j.getParentFrame().findBaseFrame().getName()
        parent[child] = (par, j)
    return parent


def _chain(parent, body):
    out = [body]
    while out[-1] in parent:
        out.append(parent[out[-1]][0])
    return out


def spanned_coordinates(model, muscle, free, couplers):
    """Free coordinates of every joint on the tree path between the bodies of the muscle's path points."""
    parent = _tree(model)
    pps = muscle.getGeometryPath().getPathPointSet()
    bodies = {pps.get(k).getParentFrame().findBaseFrame().getName() for k in range(pps.getSize())}
    chains = [_chain(parent, b) for b in bodies]
    common = set(chains[0]).intersection(*chains[1:])
    joints = set()
    for ch in chains:
        for b in ch:
            if b in common:
                break
            joints.add(parent[b][1].getName())
    coords = set()
    js = model.getJointSet()
    for jn in joints:
        j = js.get(jn)
        for k in range(j.numCoordinates()):
            coords.add(j.get_coordinates(k).getName())
    # a coupled coordinate counts as its independent coordinate(s)
    for dep, ind, _ in couplers:
        if dep in coords:
            coords.discard(dep)
            coords.update(ind)
    return [c for c in free if c in coords]     # in CoordinateSet order


def _set(model, state, cset, names, swept, defaults, couplers, row):
    values = dict(defaults)
    values.update(zip(swept, row))
    for d, ind, cc in couplers:
        x = osim.Vector(len(ind), 0.0)
        for k, n in enumerate(ind):
            x.set(k, values[n])
        values[d] = cc.getFunction().calcValue(x)
    for n in names:
        c = cset.get(n)
        if not c.getLocked(state):
            c.setValue(state, values[n], False)
    model.realizePosition(state)


MOMENT_ARMS = ("analytic", "length-derivative")


def has_wrapped_paths(osim_path):
    """True if any force's geometry path wraps over a wrap object (then use the length-derivative moment arm)."""
    from lxml import etree
    return bool(etree.parse(osim_path).getroot().xpath(".//ForceSet//PathWrap"))


def _lengths_fd(model, state, cset, names, swept, defaults, couplers, row, coord_col, muscles_of, ms, h):
    """−dL/dq by central difference for every muscle in muscles_of[coordinate], one coordinate at a time."""
    out = {}
    for c, idx in muscles_of.items():
        r = row.copy()
        r[coord_col[c]] += h
        _set(model, state, cset, names, swept, defaults, couplers, r)
        lp = [ms.get(i).getLength(state) for i in idx]
        r[coord_col[c]] -= 2 * h
        _set(model, state, cset, names, swept, defaults, couplers, r)
        lm = [ms.get(i).getLength(state) for i in idx]
        for i, a, b in zip(idx, lp, lm):
            out[(i, c)] = -(a - b) / (2 * h)
    return out


def reference(osim_path, moment_arm="analytic"):
    if moment_arm not in MOMENT_ARMS:
        raise ValueError(f"moment_arm must be one of {MOMENT_ARMS}")
    model = osim.Model(osim_path)
    state = model.initSystem()
    names, swept, poses, labels, defaults, couplers = build_poses(model)
    cset = model.getCoordinateSet()
    ms = model.getMuscles()
    muscles = [ms.get(i).getName() for i in range(ms.getSize())]
    pairs = []                                       # (muscle index, coordinate name)
    for i, mn in enumerate(muscles):
        for c in spanned_coordinates(model, ms.get(i), swept, couplers):
            pairs.append((i, c))

    P = len(poses)
    length = np.zeros((P, len(muscles)))
    ma = np.zeros((P, len(pairs)))
    ma_analytic = np.zeros((P, len(pairs)))
    qerr = np.zeros(P)
    col = {n: k for k, n in enumerate(swept)}
    muscles_of = {}
    for i, c in pairs:
        muscles_of.setdefault(c, []).append(i)
    for p, row in enumerate(poses):
        _set(model, state, cset, names, swept, defaults, couplers, row)
        qe = state.getQErr()
        qerr[p] = max((abs(qe.get(k)) for k in range(qe.size())), default=0.0)
        for i in range(len(muscles)):
            length[p, i] = ms.get(i).getLength(state)
        for k, (i, c) in enumerate(pairs):
            ma_analytic[p, k] = ms.get(i).computeMomentArm(state, cset.get(c))
        if moment_arm == "analytic":
            ma[p] = ma_analytic[p]
        else:
            fd = _lengths_fd(model, state, cset, names, swept, defaults, couplers, row, col, muscles_of, ms, H)
            ma[p] = [fd[(i, c)] for i, c in pairs]

    # M0 self-check, first N_CHECK random poses: analytic -> -dL/dq with h; length-derivative -> the same with 10 h
    check = np.zeros((N_CHECK, len(pairs)))
    h_check = H if moment_arm == "analytic" else 10 * H
    for p in range(N_CHECK):
        fd = _lengths_fd(model, state, cset, names, swept, defaults, couplers, poses[p], col, muscles_of, ms, h_check)
        check[p] = [fd[(i, c)] for i, c in pairs]

    return dict(coord_names=np.array(names), swept=np.array(swept), coords=_full_coords(poses, names, swept,
                defaults, couplers), labels=np.array(labels), muscles=np.array(muscles),
                pair_muscle=np.array([i for i, _ in pairs]), pair_coord=np.array([c for _, c in pairs]),
                length=length, ma=ma, qerr=qerr, ma_fd_check=check, n_check=np.array(N_CHECK), h=np.array(H),
                opensim_version=np.array(osim.__version__), moment_arm=np.array(moment_arm), ma_analytic=ma_analytic)


def _full_coords(poses, names, swept, defaults, couplers):
    full = np.zeros((len(poses), len(names)))
    for p, row in enumerate(poses):
        values = dict(defaults)
        values.update(zip(swept, row))
        for d, ind, cc in couplers:
            x = osim.Vector(len(ind), 0.0)
            for k, n in enumerate(ind):
                x.set(k, values[n])
            values[d] = cc.getFunction().calcValue(x)
        full[p] = [values[n] for n in names]
    return full


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("osim")
    ap.add_argument("out")
    ap.add_argument("--moment-arm", choices=MOMENT_ARMS, default="analytic",
                    help="OpenSim's computeMomentArm (default), or -dL/dq of OpenSim's length (wrapped paths)")
    a = ap.parse_args(argv)
    r = reference(a.osim, a.moment_arm)
    np.savez_compressed(a.out, **r)
    print(f"{os.path.basename(a.osim)}: {len(r['labels'])} poses, {len(r['muscles'])} muscles, "
          f"{len(r['pair_coord'])} muscle-coordinate pairs, max QErr {r['qerr'].max():.2e}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
