"""Station 2: prove the kinematic check can fail, and cross-check it a second way.

Uses gait10dof18musc (Station 1a cvt3) and its OpenSim model. Every mutation must change the result in exactly the
expected way; the cross-check recomputes knee-sweep errors without the ground-frame transform.

Usage: python station2/tests/test_harness.py <gait10dof18musc.osim> <scratch_dir> <Rajagopal2016.osim>
Exit 0 if every check passes.
"""
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import mujoco                                     # noqa: E402
import opensim as osim                            # noqa: E402
import mujoco_compare as mc                       # noqa: E402
import osim_reference as orf                      # noqa: E402

REPO = os.path.dirname(os.path.dirname(HERE))
XML = os.path.join(REPO, "station1", "gait10dof18musc", "gait10dof18musc_cvt3.xml")
fails = 0


def check(ok, msg):
    global fails
    print(("  ok   " if ok else "  FAIL ") + msg)
    fails += 0 if ok else 1


def mutated(text, old, new, scratch, tag):
    assert text.count(old) == 1, f"mutation anchor not unique: {old}"
    p = os.path.join(scratch, f"mut_{tag}.xml")
    with open(p, "w") as f:
        f.write(text.replace(old, new))
    return p


def main():
    osim_path, scratch = sys.argv[1], sys.argv[2]
    os.makedirs(scratch, exist_ok=True)
    ref = orf.reference(osim_path)
    text = open(XML).read()
    base = mc.compare(XML, ref)
    print("unmutated")
    check(base["K0"] == "PASS", "K0 passes")
    torso0 = base["per_body"]["torso"]
    check(torso0["pos_max_mm"] < 1e-9 and torso0["rot_max_deg"] < 1e-9, "torso matches exactly before mutation")

    print("M1: torso moved 2 mm along its parent's x")
    r = mc.compare(mutated(text, 'name="torso" pos="-0.1007 0.0815 0"', 'name="torso" pos="-0.0987 0.0815 0"',
                           scratch, "m1"), ref)
    t = r["per_body"]["torso"]
    check(abs(t["pos_max_mm"] - 2.0) < 1e-6 and abs(t["pos_mean_mm"] - 2.0) < 1e-6, f"torso error 2 mm in every pose ({t['pos_max_mm']:.9f})")
    check(r["K1"] == "FAIL" and r["bodies_failing_K1"] == ["torso"], f"K1 fails on torso only ({r['bodies_failing_K1']})")
    check(r["K2"] == "PASS", "K2 unaffected")

    print("M2: torso rotated 1 deg about its z axis")
    q = np.zeros(4)
    mujoco.mju_axisAngle2Quat(q, np.array([0.0, 0.0, 1.0]), np.radians(1.0))
    r = mc.compare(mutated(text, 'name="torso" pos="-0.1007 0.0815 0"',
                           'name="torso" pos="-0.1007 0.0815 0" quat="%.17g %.17g %.17g %.17g"' % tuple(q),
                           scratch, "m2"), ref)
    t = r["per_body"]["torso"]
    check(abs(t["rot_max_deg"] - 1.0) < 1e-9 and abs(t["rot_mean_deg"] - 1.0) < 1e-9, f"torso error 1 deg in every pose ({t['rot_max_deg']:.12f})")
    check(t["pos_max_mm"] < 1e-9, "torso origin unaffected")
    check(r["K2"] == "FAIL" and r["bodies_failing_K2"] == ["torso"] and r["K1"] == base["K1"], "K2 fails on torso only")

    print("M3: ground body's rotation changed, pelvis left alone (proves the ground-frame transform is used)")
    r = mc.compare(mutated(text, '<body name="ground" quat="0.707035 0.707179 0 0">', '<body name="ground">',
                           scratch, "m3"), ref)
    check(r["K1"] == "FAIL" and r["K2"] == "FAIL" and abs(r["rot_worst_deg"]["value"] - 90.0) < 0.1,
          f"everything off by ~90 deg ({r['rot_worst_deg']['value']:.4f})")

    print("M4: a MuJoCo body renamed")
    r = mc.compare(mutated(text, '<body name="torso"', '<body name="torso_x"', scratch, "m4"), ref)
    check(r["K0"] == "FAIL" and any("torso" in p for p in r["K0_problems"]), "K0 fails, names torso")

    print("M5: dependent joints solved with the quartic term dropped (MuJoCo residual must catch it)")
    orig = mc.set_pose

    def bad_set_pose(m, d, from_coord, driven, values):
        orig(m, d, from_coord, {j: (j2, np.r_[c[:4], 0.0]) for j, (j2, c) in driven.items()}, values)
    mc.set_pose = bad_set_pose
    try:
        r = mc.compare(XML, ref)
    finally:
        mc.set_pose = orig
    check(r["K0"] == "FAIL" and any("residual" in p for p in r["K0_problems"]) and r["K1"] == "NOT EVALUATED",
          f"K0 fails on residual ({r['K0_problems']})")

    print("M6: a joint not tied to any coordinate or equality")
    r = mc.compare(mutated(text, '<joint name="lumbar_extension"', '<joint name="lumbar_extension_x"', scratch, "m6"), ref)
    check(r["K0"] == "FAIL" and any("lumbar_extension_x" in p for p in r["K0_problems"]), "K0 fails, names the joint")

    print("M7: wrong pose count")
    bad = dict(ref)
    bad["labels"] = ref["labels"][:-1]
    bad["coords"], bad["pos"], bad["rot"] = ref["coords"][:-1], ref["pos"][:-1], ref["rot"][:-1]
    r = mc.compare(XML, bad)
    check(r["K0"] == "FAIL" and any("pose count" in p for p in r["K0_problems"]), "K0 fails on pose count")

    print("Cross-check: knee_angle_r sweep, tibia_r relative to femur_r, no ground transform, no harness code")
    # femur_r matches OpenSim exactly (unmutated run), so tibia-in-femur error must equal the harness's
    # tibia-in-ground error, pose by pose.
    check(base["per_body"]["femur_r"]["pos_max_mm"] < 1e-9 and base["per_body"]["femur_r"]["rot_max_deg"] < 1e-9,
          "femur_r exact (precondition)")
    m, _ = mc.load_without_meshes(XML)
    d = mujoco.MjData(m)
    om = osim.Model(osim_path)
    s = om.initSystem()
    J, B = mujoco.mjtObj.mjOBJ_JOINT, mujoco.mjtObj.mjOBJ_BODY
    jid = {mujoco.mj_id2name(m, J, j): j for j in range(m.njnt)}
    eq = {int(m.eq_obj1id[e]): (int(m.eq_obj2id[e]), m.eq_data[e, :5].copy()) for e in range(m.neq)}
    check(all(j2 >= 0 and m.qpos0[m.jnt_qposadr[j2]] == 0 and m.qpos0[m.jnt_qposadr[j1]] == 0
              for j1, (j2, _) in eq.items()), "gait10: every equality has a second joint and qpos0 = 0 (simple form valid)")
    names = [str(x) for x in ref["coord_names"]]
    labels = [str(x) for x in ref["labels"]]
    idx = [i for i, l in enumerate(labels) if l == "sweep:knee_angle_r"]
    tib, fem = mujoco.mj_name2id(m, B, "tibia_r"), mujoco.mj_name2id(m, B, "femur_r")
    k_tib = [str(x) for x in ref["body_names"] if str(x) != "ground"].index("tibia_r")
    diffs, indep = [], []
    for p in idx:
        for n in names:
            om.getCoordinateSet().get(n).setValue(s, ref["coords"][p, names.index(n)], False)
        om.realizePosition(s)
        v = om.getBodySet().get("tibia_r").findStationLocationInAnotherFrame(s, osim.Vec3(0, 0, 0),
                                                                             om.getBodySet().get("femur_r"))
        po = np.array([v.get(k) for k in range(3)])
        d.qpos[:] = 0
        for n in names:
            d.qpos[m.jnt_qposadr[jid[n]]] = ref["coords"][p, names.index(n)]
        for j1, (j2, c) in eq.items():
            d.qpos[m.jnt_qposadr[j1]] = np.polyval(c[::-1], d.qpos[m.jnt_qposadr[j2]])
        mujoco.mj_kinematics(m, d)
        pm = d.xmat[fem].reshape(3, 3).T @ (d.xpos[tib] - d.xpos[fem])
        e = 1000 * np.linalg.norm(pm - po)
        indep.append(e)
        diffs.append(abs(e - base["_perr"][p, k_tib]))
    check(max(diffs) < 1e-9, f"{len(idx)} poses: independent vs harness error differ by at most {max(diffs):.2e} mm "
          f"(independent range {min(indep):.4f}..{max(indep):.4f} mm)")

    print("OpenSim side: a coupled coordinate set wrong must show up in QErr (Rajagopal-type models)")
    if len(sys.argv) > 3:
        raj = osim.Model(sys.argv[3])
        rs = raj.initSystem()
        raj.realizePosition(rs)
        q0 = max(abs(rs.getQErr().get(k)) for k in range(rs.getQErr().size()))
        c = raj.getCoordinateSet().get("knee_angle_r_beta")
        c.setValue(rs, c.getValue(rs) + 0.01, False)
        raj.realizePosition(rs)
        q1 = max(abs(rs.getQErr().get(k)) for k in range(rs.getQErr().size()))
        check(q0 <= 1e-9 < q1, f"QErr {q0:.1e} at default, {q1:.1e} with knee_angle_r_beta off by 0.01 rad")
    else:
        check(False, "Rajagopal2016.osim not given; QErr check not run")

    print("ALL CHECKS PASS" if fails == 0 else f"{fails} CHECK(S) FAILED")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
