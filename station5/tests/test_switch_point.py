"""Change 5.3 (conditional path points that switch toward a neighbour, through a helper joint): checks.

Usage: python station5/tests/test_switch_point.py <src_dir> <gait2354.osim> <geometry_dir> <scratch_dir>
  src_dir: the converter to test (this repo's src/, or an older copy to show the checks fail without change 5.3)

- dependencyJointAng (Steps 2-3 pose the model with it), on small hand-written models: a plain coupling gives the
  quartic; a chain (coordinate -> helper -> slide) is followed whatever the order of the equalities; an equality
  driven by a joint that isn't given is left out (as upstream); a joint driven twice is an error (as upstream).
- scale_helper: the chained function is unchanged (1e-9 relative) and the helper's slope is at most 0.1.
- Converting gait2354 (XML step): iliacus and psoas on both sides use the switch; each has a helper joint on its
  own body under the world body (template minimum mass 0.001 kg, one geom that collides with nothing), driven by its
  hip flexion, with a slope of at most 0.1 over the coordinate's range; the point's slide joints follow the helper;
  the keyframe satisfies every joint equality; dependencyJointAng poses every driven joint exactly as Station 3's
  solver does (random poses).
- In motion (MuJoCo simulation, gravity off, right hip flexion driven between -11 and 46 degrees at 1 Hz by a joint
  torque, 3 s): iliacus_r and psoas_r stay within 0.5 mm of their length at the simulated pose with every
  equality solved exactly, and nothing diverges.
Exit 0 if every check passes.
"""
import os
import subprocess
import sys

import mujoco
import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
RUNNER = r'''
import sys
sys.path.insert(0, sys.argv[4])
from myoconverter.O2MPipeline import O2MPipeline
O2MPipeline(sys.argv[1], sys.argv[2], sys.argv[3], convert_steps=[1], muscle_list=None, osim_data_overwrite=True,
            conversion=True, validation=False, speedy=False, generate_pdf=False, add_ground_geom=True,
            treat_as_normal_path_point=False)
'''
fails = 0


def check(ok, msg):
    global fails
    print(("  ok   " if ok else "  FAIL ") + msg)
    fails += 0 if ok else 1


def small_model(equalities):
    """Hinges a, b, h and slide s on separate bodies, with the given joint equalities (joint1, joint2, coefs)."""
    eq = "".join(f'<joint joint1="{j1}" joint2="{j2}" polycoef="{" ".join(map(str, c))}"/>' for j1, j2, c in equalities)
    xml = f"""<mujoco><worldbody>
      <body name="A"><joint name="a" type="hinge" axis="0 0 1"/><geom size="0.1"/></body>
      <body name="B"><joint name="b" type="hinge" axis="0 0 1"/><geom size="0.1"/></body>
      <body name="H"><joint name="h" type="hinge" axis="0 0 1"/><geom size="0.1"/></body>
      <body name="S"><joint name="s" type="slide" axis="1 0 0"/><geom size="0.1"/></body>
      </worldbody><equality>{eq}</equality></mujoco>"""
    return mujoco.MjModel.from_xml_string(xml)


def main():
    src, osim_path, geometry, scratch = (os.path.abspath(p) for p in sys.argv[1:5])
    os.makedirs(scratch, exist_ok=True)
    sys.path.insert(0, src)
    from myoconverter.optimization.utils.UtilsMujoco import dependencyJointAng
    from myoconverter.xml import utils as xu
    P = np.polynomial.Polynomial

    print("dependencyJointAng")
    c1, c2 = [0.1, 0.5, -0.2, 0.05, 0.01], [-0.3, 2.0, 0.4, -0.1, 0.02]
    m = small_model([("b", "a", c1)])
    dep, ang, free = dependencyJointAng(m, np.array([0]), np.array([0.7]))
    check(list(dep) == [1] and list(free) == [0] and abs(ang[0] - P(c1)(0.7)) < 1e-12, "plain coupling: b = p(a)")
    for order in ((("s", "h", c2), ("h", "a", c1)), (("h", "a", c1), ("s", "h", c2))):
        m = small_model(list(order))
        dep, ang, free = dependencyJointAng(m, np.array([0]), np.array([0.7]))
        got = dict(zip([int(j) for j in dep], ang))
        check(set(got) == {2, 3} and abs(got[2] - P(c1)(0.7)) < 1e-12 and abs(got[3] - P(c2)(P(c1)(0.7))) < 1e-12,
              f"chain a -> h -> s followed (equalities in order {[o[0] for o in order]})")
    m = small_model([("b", "a", c1), ("s", "h", c2)])
    dep, ang, free = dependencyJointAng(m, np.array([0]), np.array([0.7]))
    check([int(j) for j in dep] == [1], "an equality driven by a joint not given is left out")
    m = small_model([("b", "a", c1), ("b", "h", c2)])
    try:
        dependencyJointAng(m, np.array([0, 2]), np.array([0.7, 0.1]))
        check(False, "a joint driven twice raises RuntimeError")
    except RuntimeError:
        check(True, "a joint driven twice raises RuntimeError")

    print("scale_helper")
    has = hasattr(xu, "scale_helper")
    check(has, "converter has scale_helper")
    if has:
        rng = np.random.default_rng(0)
        worst_rel, worst_slope = 0.0, 0.0
        for _ in range(20):
            p1, p2 = P(rng.normal(0, 1, 5)), P(rng.normal(0, 1, 5))
            q1, q2, k = xu.scale_helper(p1, p2, [-1.5, 2.0])
            xs = np.linspace(-1.5, 2.0, 5001)
            y, y2 = p2(p1(xs)), q2(q1(xs))
            worst_rel = max(worst_rel, np.max(np.abs(y2 - y)) / max(1.0, np.max(np.abs(y))))
            worst_slope = max(worst_slope, np.max(np.abs(q1.deriv()(xs))))
        check(worst_rel < 1e-9, f"same function after scaling (largest relative change {worst_rel:.1e})")
        check(worst_slope <= 0.1 * (1 + 1e-6), f"helper slope at most 0.1 ({worst_slope:.6f})")

    print("The converted model (gait2354, XML step)")
    out = os.path.join(scratch, "converted")
    r = subprocess.run([sys.executable, "-c", RUNNER, osim_path, geometry, out, src], capture_output=True, text=True)
    check(r.returncode == 0, "XML step runs")
    if r.returncode != 0:
        print(r.stderr[-1500:])
        return 1
    sys.path.insert(0, REPO)
    from lowlimb_converter.checks import mujoco_compare as mc
    m, _ = mc.load_without_meshes(os.path.join(out, "gait2354_cvt1.xml"))
    J = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    B = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
    eqs = {int(m.eq_obj1id[e]): (int(m.eq_obj2id[e]), np.array(m.eq_data[e, :5])) for e in range(m.neq)}
    for side in ("r", "l"):
        hip = J(f"hip_flexion_{side}")
        lo, hi = m.jnt_range[hip]
        for muscle in ("iliacus", "psoas"):
            point = f"{muscle}_{side}_{muscle}_{side}-P3"
            helper = f"{point}_helper"
            hj, hb = J(helper), B(helper)
            check(hj >= 0 and hb >= 0 and m.body_parentid[hb] == 0 and m.jnt_bodyid[hj] == hb,
                  f"{helper}: own body under the world body")
            if hj < 0:
                continue
            check(abs(m.body_mass[hb] - 0.001) < 1e-12 and m.body_geomnum[hb] == 1
                  and m.geom_contype[m.body_geomadr[hb]] == 0 and m.geom_conaffinity[m.body_geomadr[hb]] == 0,
                  f"{helper}: mass {m.body_mass[hb]} kg, one geom that collides with nothing")
            drv, coef = eqs.get(hj, (-1, None))
            slope = np.max(np.abs(P(coef).deriv()(np.linspace(lo, hi, 2001)))) if drv == hip else np.inf
            check(drv == hip and slope <= 0.1 * (1 + 1e-6), f"{helper}: driven by hip_flexion_{side}, "
                  f"slope at most 0.1 ({slope:.6f})")
            slides = [j for j in range(m.njnt) if mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j)
                      .startswith(point + "_") and j != hj]
            check(len(slides) > 0 and all(eqs.get(j, (-1,))[0] == hj for j in slides),
                  f"{point}: {len(slides)} slide joint(s), all following the helper")
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    res, n = mc.equality_residual(m, d)
    check(res < 1e-9, f"keyframe satisfies all {n} joint equalities (largest residual {res:.1e})")

    names = [mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(m.njnt)]
    from_coord, driven, problems = mc.joint_plan(m, [n for j, n in enumerate(names) if j not in eqs])
    rng = np.random.default_rng(1)
    free = np.array(sorted(from_coord))
    worst = 0.0
    for _ in range(50):
        vals = {from_coord[j]: rng.uniform(*m.jnt_range[j]) if m.jnt_limited[j] else rng.normal() for j in free}
        exact = mc.solve_qpos(m, from_coord, driven, vals)
        dep, ang, _ = dependencyJointAng(m, free, np.array([vals[from_coord[j]] for j in free]))
        got = dict(zip([int(j) for j in dep], ang))
        if set(got) != set(driven):
            worst = np.inf
            break
        worst = max(worst, max(abs(got[j] - exact[m.jnt_qposadr[j]]) for j in driven))
    check(worst < 1e-12, f"dependencyJointAng poses all {len(driven)} driven joints as Station 3's solver "
          f"(largest difference {worst:.1e})")

    print("In motion")
    m.opt.gravity[:] = 0
    hip = J("hip_flexion_r")
    acts = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, a) for a in ("iliacus_r", "psoas_r")]
    mujoco.mj_resetDataKeyframe(m, d, 0)
    exact_d = mujoco.MjData(m)
    worst, qmin, qmax = 0.0, 9.0, -9.0
    for k in range(int(3.0 / m.opt.timestep)):
        target = 0.3 + 0.5 * np.sin(2 * np.pi * d.time)
        d.qfrc_applied[:] = 0
        d.qfrc_applied[m.jnt_dofadr[hip]] = 300 * (target - d.qpos[m.jnt_qposadr[hip]]) - 10 * d.qvel[m.jnt_dofadr[hip]]
        mujoco.mj_step(m, d)
        if not np.all(np.isfinite(d.qpos)):
            break
        if d.time > 0.3 and k % 10 == 0:
            q = d.qpos[m.jnt_qposadr[hip]]
            qmin, qmax = min(qmin, q), max(qmax, q)
            exact_d.qpos[:] = mc.solve_qpos(m, from_coord, driven, {n: d.qpos[m.jnt_qposadr[j]]
                                                                   for j, n in from_coord.items()})
            mujoco.mj_forward(m, exact_d)
            worst = max(worst, np.max(np.abs(d.actuator_length[acts] - exact_d.actuator_length[acts])))
    check(np.all(np.isfinite(d.qpos)), "simulation stays finite")
    check(qmax - qmin > 0.8, f"hip moved over {np.degrees(qmin):.0f}..{np.degrees(qmax):.0f} deg")
    check(worst < 0.0005, f"iliacus_r and psoas_r within {1000 * worst:.3f} mm of their exact-pose lengths")

    print("ALL CHECKS PASS" if fails == 0 else f"{fails} CHECK(S) FAILED")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
