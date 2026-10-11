"""Station 3: prove the muscle-path check can fail, and cross-check its moment arms a second way.

Uses gait10dof18musc: the Station 1a cvt3 file committed in station1/ (a fixed fixture) and its OpenSim model.

- Muscle-coordinate pairs match the anatomy (which joints each muscle crosses).
- Moving one MuJoCo path site 5 mm changes that muscle's errors and no other muscle's.
- Renaming a tendon fails M0.
- Analytic moment arms from MuJoCo's tendon Jacobian, with the chain rule through every dependent joint's
  equality polynomial, agree with the harness's finite differences in every pose.

Usage: python station3/tests/test_paths_harness.py <gait10dof18musc.osim> <scratch_dir>
Exit 0 if every check passes.
"""
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, REPO)
from lowlimb_converter.checks import mujoco_compare as mc   # noqa: E402
from lowlimb_converter.checks import mujoco_paths as mp     # noqa: E402
from lowlimb_converter.checks import osim_paths as op       # noqa: E402

XML = os.path.join(REPO, "station1", "gait10dof18musc", "gait10dof18musc_cvt3.xml")
fails = 0


def check(ok, msg):
    global fails
    print(("  ok   " if ok else "  FAIL ") + msg)
    fails += 0 if ok else 1


def mutated(text, old, new, scratch, tag):
    assert text.count(old) == 1, old
    p = os.path.join(scratch, f"mut_{tag}.xml")
    with open(p, "w") as f:
        f.write(text.replace(old, new))
    return p


def tendon_jacobian(m, d):
    """Dense tendon Jacobian (ntendon x nv) for MuJoCo 2.x (ten_moment) and 3.x (ten_J, dense or sparse)."""
    for src in (m, d):      # sparsity pattern: in mjModel (recent MuJoCo 3) or mjData (earlier sparse versions)
        if hasattr(src, "ten_J_rownnz"):
            dense = np.zeros((m.ntendon, m.nv))
            mujoco.mju_sparse2dense(dense, d.ten_J, src.ten_J_rownnz, src.ten_J_rowadr, src.ten_J_colind)
            return dense
    J = d.ten_J if hasattr(d, "ten_J") else d.ten_moment
    return np.array(J).reshape(m.ntendon, m.nv)


def main():
    osim_path, scratch = sys.argv[1], sys.argv[2]
    os.makedirs(scratch, exist_ok=True)
    ref = op.reference(osim_path)
    base = mp.evaluate(XML, ref, keep_arrays=True)
    check(base["M0"] == "PASS", f"unmutated model passes M0 ({base['M0_problems']})")

    print("Pairs follow the anatomy")
    pairs = {}
    for q in base["pairs"]:
        pairs.setdefault(q["muscle"], []).append(q["coordinate"])
    expect = {"soleus_r": ["ankle_angle_r"], "tib_ant_r": ["ankle_angle_r"], "glut_max_r": ["hip_flexion_r"],
              "gastroc_r": ["knee_angle_r", "ankle_angle_r"], "rect_fem_r": ["hip_flexion_r", "knee_angle_r"],
              "hamstrings_r": ["hip_flexion_r", "knee_angle_r"], "vasti_r": ["knee_angle_r"],
              "bifemsh_r": ["knee_angle_r"], "iliopsoas_r": ["hip_flexion_r"]}
    for mn, cs in expect.items():
        check(sorted(pairs.get(mn, [])) == sorted(cs), f"{mn}: {pairs.get(mn)}")

    print("M1: soleus origin moved 5 mm")
    text = open(XML).read()
    r = mp.evaluate(mutated(text, '<site name="soleus_r_soleus_r-P1" pos="-0.0024 -0.1533 0.0071"/>',
                            '<site name="soleus_r_soleus_r-P1" pos="-0.0024 -0.1483 0.0071"/>', scratch, "m1"), ref)
    changed = [mn for mn in r["per_muscle"] if r["per_muscle"][mn] != base["per_muscle"][mn]]
    check(changed == ["soleus_r"], f"only soleus_r's length errors change ({changed})")
    shift = r["per_muscle"]["soleus_r"]["length_mean_signed_mm"] - base["per_muscle"]["soleus_r"]["length_mean_signed_mm"]
    check(3.0 < abs(shift) <= 5.0, f"soleus_r length shifts by {shift:.3f} mm (site moved 5 mm, mostly along the muscle)")
    other = [q for q in r["pairs"] if q["muscle"] != "soleus_r"]
    check(all(q == b for q, b in zip(other, [b for b in base["pairs"] if b["muscle"] != "soleus_r"])),
          "no other pair's moment-arm error changes")

    print("M2: tendon renamed")
    renamed = text.replace('tendon="soleus_r_tendon"', 'tendon="soleus_r_tendonX"')   # the actuator follows it
    r = mp.evaluate(mutated(renamed, '<spatial name="soleus_r_tendon">', '<spatial name="soleus_r_tendonX">', scratch,
                            "m2"), ref)
    check(r["M0"] == "FAIL" and any("soleus_r" in p for p in r["M0_problems"]), "M0 fails and names soleus_r")

    print("Cross-check: analytic moment arms (tendon Jacobian + chain rule through dependent joints) vs harness")
    m, _ = mc.load_without_meshes(XML)
    d = mujoco.MjData(m)
    names = [str(x) for x in ref["coord_names"]]
    from_coord, driven, _ = mc.joint_plan(m, names)
    jid_of = {n: j for j, n in from_coord.items()}
    muscles = [str(x) for x in ref["muscles"]]
    tid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_TENDON, f"{mn}_tendon") for mn in muscles]
    worst = 0.0
    for p in range(len(ref["labels"])):
        values = {n: ref["coords"][p, names.index(n)] for n in names}
        qpos = mc.solve_qpos(m, from_coord, driven, values)
        d.qpos[:] = qpos
        mujoco.mj_fwdPosition(m, d)
        J = tendon_jacobian(m, d)
        for k, (i, c) in enumerate(zip(ref["pair_muscle"], ref["pair_coord"])):
            # dq_j/dq_c for every joint j: 1 for the coordinate's joint, chain rule for dependents
            dq = {jid_of[str(c)]: 1.0}
            todo = dict(driven)
            while todo:
                for j1, (j2, coef) in list(todo.items()):
                    if j2 >= 0 and j2 not in dq and j2 in todo:
                        continue
                    x = qpos[m.jnt_qposadr[j2]] - m.qpos0[m.jnt_qposadr[j2]] if j2 >= 0 else 0.0
                    slope = coef[1] + 2 * coef[2] * x + 3 * coef[3] * x ** 2 + 4 * coef[4] * x ** 3
                    dq[j1] = slope * dq.get(j2, 0.0) if j2 >= 0 else 0.0
                    del todo[j1]
            analytic = -sum(J[tid[i], m.jnt_dofadr[j]] * v for j, v in dq.items())
            worst = max(worst, abs(analytic - base["_ma"][p, k]))
    check(worst < 1e-6, f"every pose and pair: largest difference {worst:.2e} m")

    print("ALL CHECKS PASS" if fails == 0 else f"{fails} CHECK(S) FAILED")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
