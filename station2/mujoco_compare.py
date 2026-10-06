"""Station 2, MuJoCo side: put the converted model in each OpenSim pose and compare every body.

Needs only MuJoCo and NumPy (works with MuJoCo 2.3.7 / NumPy 1.x and MuJoCo 3.x / NumPy 2.x), reads the .npz
written by osim_reference.py. Checks gate K0 (the check is valid) and reports K1 (position) and K2 (orientation).
Method and gates: station2/GATES.md. Usage:
    python station2/mujoco_compare.py model.xml reference.npz out.json
Exit 0 if K0, K1 and K2 all pass, 1 if any fails.
"""
import argparse
import json
import os
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

K0_RESIDUAL = 1e-9
K1_MM = 1.0
K2_DEG = 0.5


def load_without_meshes(xml_path):
    """Compile the model with every mesh asset and mesh geom removed (they don't affect where bodies are)."""
    root = ET.parse(xml_path).getroot()
    removed = set()
    for parent in root.iter():
        for child in list(parent):
            if child.tag == "geom" and (child.get("type") == "mesh" or child.get("mesh") is not None):
                removed.add(child.get("name"))
                parent.remove(child)
            elif child.tag == "mesh" and parent.tag == "asset":
                parent.remove(child)
    for contact in root.iter("contact"):
        for pair in list(contact):
            if pair.get("geom1") in removed or pair.get("geom2") in removed:
                contact.remove(pair)
    return mujoco.MjModel.from_xml_string(ET.tostring(root, encoding="unicode")), len(removed)


def _name(m, objtype, i):
    return mujoco.mj_id2name(m, objtype, i)


def _eq_active(m):
    return m.eq_active0 if hasattr(m, "eq_active0") else m.eq_active   # renamed in MuJoCo 3


def joint_plan(m, coord_names):
    """Split MuJoCo joints into those set from a coordinate and those solved from a joint equality."""
    J = mujoco.mjtObj.mjOBJ_JOINT
    eq_joint = int(mujoco.mjtEq.mjEQ_JOINT)
    problems = []
    driven = {}                                        # dependent joint id -> (independent joint id or -1, coefs)
    active = _eq_active(m)
    for e in range(m.neq):
        if int(m.eq_type[e]) != eq_joint:
            problems.append(f"equality {e} has type {int(m.eq_type[e])}, not a joint equality")
            continue
        if not active[e]:
            problems.append(f"joint equality {_name(m, mujoco.mjtObj.mjOBJ_EQUALITY, e)} is inactive")
            continue
        j1, j2 = int(m.eq_obj1id[e]), int(m.eq_obj2id[e])
        if j1 in driven:
            problems.append(f"joint {_name(m, J, j1)} is driven by two equalities")
        driven[j1] = (j2, np.array(m.eq_data[e, :5], dtype=float))
    for j in range(m.njnt):
        if int(m.jnt_type[j]) not in (int(mujoco.mjtJoint.mjJNT_HINGE), int(mujoco.mjtJoint.mjJNT_SLIDE)):
            problems.append(f"joint {_name(m, J, j)} is not a hinge or slide")
    coord_set = set(coord_names)
    from_coord = {}
    for j in range(m.njnt):
        n = _name(m, J, j)
        if j in driven:
            continue
        if n in coord_set:
            from_coord[j] = n
        else:
            problems.append(f"joint {n} is neither an OpenSim coordinate nor driven by a joint equality")
    return from_coord, driven, problems


def set_pose(m, d, from_coord, driven, values):
    """qpos for one pose: coordinates first, then dependent joints in dependency order."""
    qpos = m.qpos0.copy()
    for j, n in from_coord.items():
        qpos[m.jnt_qposadr[j]] = values[n]
    done = set(from_coord)
    todo = dict(driven)
    while todo:
        progressed = False
        for j1, (j2, c) in list(todo.items()):
            if j2 >= 0 and j2 not in done:
                continue
            x = qpos[m.jnt_qposadr[j2]] - m.qpos0[m.jnt_qposadr[j2]] if j2 >= 0 else 0.0
            qpos[m.jnt_qposadr[j1]] = m.qpos0[m.jnt_qposadr[j1]] + c[0] + c[1] * x + c[2] * x**2 + c[3] * x**3 + c[4] * x**4
            done.add(j1)
            del todo[j1]
            progressed = True
        if not progressed:
            raise RuntimeError("joint equalities form a cycle")
    d.qpos[:] = qpos
    mujoco.mj_forward(m, d)


def equality_residual(m, d):
    rows = np.asarray(d.efc_type)[: d.nefc] == int(mujoco.mjtConstraint.mjCNSTR_EQUALITY)
    r = np.asarray(d.efc_pos)[: d.nefc][rows]
    return float(np.max(np.abs(r))) if r.size else 0.0, int(rows.sum())


def angle_deg(Ra, Rb):
    """Angle of Ra^T Rb, robust near 0 (uses the skew part as well as the trace)."""
    D = Ra.T @ Rb
    s = 0.5 * np.linalg.norm([D[2, 1] - D[1, 2], D[0, 2] - D[2, 0], D[1, 0] - D[0, 1]])
    c = 0.5 * (np.trace(D) - 1.0)
    return np.degrees(np.arctan2(s, c))


def compare(xml_path, ref, keep_arrays=True):
    m, n_mesh = load_without_meshes(xml_path)
    d = mujoco.MjData(m)
    B = mujoco.mjtObj.mjOBJ_BODY
    coord_names = [str(x) for x in ref["coord_names"]]
    body_names = [str(x) for x in ref["body_names"]]
    swept = [str(x) for x in ref["swept"]]
    labels = [str(x) for x in ref["labels"]]
    k0 = []

    gid = mujoco.mj_name2id(m, B, "ground")
    if gid < 0:
        k0.append("no MuJoCo body named ground")
    compared = [b for b in body_names if b != "ground"]
    bid = {b: mujoco.mj_name2id(m, B, b) for b in compared}
    k0 += [f"OpenSim body {b} has no MuJoCo body" for b, i in bid.items() if i < 0]
    from_coord, driven, problems = joint_plan(m, coord_names)
    k0 += problems
    if len(labels) != 1000 + 41 * len(swept) or labels.count("random") != 1000:
        k0.append(f"pose count {len(labels)} != 1000 + 41 x {len(swept)}")
    osim_qerr = float(np.max(ref["qerr"]))
    if osim_qerr > K0_RESIDUAL:
        k0.append(f"OpenSim QErr {osim_qerr:.2e} > {K0_RESIDUAL}")
    if k0:
        return {"model": os.path.basename(xml_path), "K0": "FAIL", "K0_problems": k0}

    P, nb = len(labels), len(compared)
    osim_col = {n: i for i, n in enumerate(coord_names)}
    ob = [body_names.index(b) for b in compared]
    perr = np.zeros((P, nb))
    rerr = np.zeros((P, nb))
    max_res, n_eq_rows = 0.0, 0
    for p in range(P):
        values = {n: ref["coords"][p, osim_col[n]] for n in coord_names}
        set_pose(m, d, from_coord, driven, values)
        res, n_eq_rows = equality_residual(m, d)
        max_res = max(max_res, res)
        Rg = d.xmat[gid].reshape(3, 3)
        pg = d.xpos[gid]
        for k, b in enumerate(compared):
            pm = Rg.T @ (d.xpos[bid[b]] - pg)
            Rm = Rg.T @ d.xmat[bid[b]].reshape(3, 3)
            perr[p, k] = 1000.0 * np.linalg.norm(pm - ref["pos"][p, ob[k]])
            rerr[p, k] = angle_deg(ref["rot"][p, ob[k]], Rm)
    if n_eq_rows != m.neq:
        k0.append(f"{n_eq_rows} equality rows in the solver, model has {m.neq} equalities")
    if max_res > K0_RESIDUAL:
        k0.append(f"MuJoCo equality residual {max_res:.2e} > {K0_RESIDUAL}")

    def worst(err):
        p, k = np.unravel_index(np.argmax(err), err.shape)
        return {"value": float(err[p, k]), "body": compared[k], "pose": int(p), "pose_label": labels[p]}

    per_body = {b: {"pos_mean_mm": float(perr[:, k].mean()), "pos_rms_mm": float(np.sqrt((perr[:, k] ** 2).mean())),
                    "pos_max_mm": float(perr[:, k].max()), "rot_mean_deg": float(rerr[:, k].mean()),
                    "rot_rms_deg": float(np.sqrt((rerr[:, k] ** 2).mean())), "rot_max_deg": float(rerr[:, k].max())}
                for k, b in enumerate(compared)}
    per_sweep = {}
    for n in swept:
        idx = [i for i, l in enumerate(labels) if l == f"sweep:{n}"]
        per_sweep[n] = {"pos_max_mm": float(perr[idx].max()), "rot_max_deg": float(rerr[idx].max())}
    rnd = [i for i, l in enumerate(labels) if l == "random"]
    out = {
        "model": os.path.basename(xml_path), "mujoco_version": mujoco.__version__, "numpy_version": np.__version__,
        "opensim_version": str(ref["opensim_version"]), "seed": int(ref["seed"]),
        "poses": P, "random_poses": len(rnd), "swept_coordinates": swept,
        "locked_coordinates": [str(x) for x in ref["locked"]], "coupled_coordinates": [str(x) for x in ref["dependent"]],
        "bodies_compared": nb, "mesh_geoms_removed": n_mesh,
        "joints_from_coordinates": len(from_coord), "joints_from_equalities": len(driven),
        "max_equality_residual": max_res, "max_osim_qerr": osim_qerr,
        "K0": "PASS" if not k0 else "FAIL", "K0_problems": k0,
        "pos_worst_mm": worst(perr), "rot_worst_deg": worst(rerr),
        "pos_rms_all_mm": float(np.sqrt((perr ** 2).mean())), "rot_rms_all_deg": float(np.sqrt((rerr ** 2).mean())),
        "pos_rms_random_mm": float(np.sqrt((perr[rnd] ** 2).mean())),
        "per_body": per_body, "per_sweep": per_sweep,
    }
    out["K1"] = "NOT EVALUATED" if k0 else ("PASS" if perr.max() <= K1_MM else "FAIL")
    out["K2"] = "NOT EVALUATED" if k0 else ("PASS" if rerr.max() <= K2_DEG else "FAIL")
    out["bodies_failing_K1"] = [b for b in compared if per_body[b]["pos_max_mm"] > K1_MM]
    out["bodies_failing_K2"] = [b for b in compared if per_body[b]["rot_max_deg"] > K2_DEG]
    if keep_arrays:                       # per-pose errors for tests; not written to JSON
        out["_perr"], out["_rerr"] = perr, rerr
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("xml")
    ap.add_argument("ref")
    ap.add_argument("out")
    a = ap.parse_args(argv)
    ref = dict(np.load(a.ref))
    r = compare(a.xml, ref, keep_arrays=False)
    with open(a.out, "w") as f:
        json.dump(r, f, indent=1)
    if r["K0"] != "PASS":
        print(f"{a.xml}: K0 FAIL (K1, K2 not evaluated): " + "; ".join(r["K0_problems"]))
        return 1
    print(f"{a.xml}: K0 PASS (residual {r['max_equality_residual']:.1e}) | "
          f"K1 {r['K1']} worst {r['pos_worst_mm']['value']:.4f} mm ({r['pos_worst_mm']['body']}) | "
          f"K2 {r['K2']} worst {r['rot_worst_deg']['value']:.4f} deg ({r['rot_worst_deg']['body']})")
    return 0 if r["K1"] == r["K2"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
