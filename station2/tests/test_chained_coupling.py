"""Change 2.4 (chained quartic couplings): the fit, the converted model, and the model in motion.

Usage: python station2/tests/test_chained_coupling.py <src_dir> <Rajagopal2016.osim> <geometry_dir> <scratch_dir>
  src_dir: the converter to test (this repo's src/, or an older copy to show the checks fail without change 2.4)

- fit_spline_chained on Rajagopal 2016's patella rotation spline: largest error well under the single quartic's,
  checked against OpenSim's spline on a grid ten times finer than the fit's; same result when run twice.
- Converting Rajagopal 2016 (XML step): each patella rotation is coupled to its knee through a helper joint on its own
  body under the world body; the helper adds the template's minimum body mass (0.001 kg) and nothing else to the
  model; the keyframe satisfies every joint equality.
- In motion (MuJoCo simulation, gravity off, muscles at zero activation, the right knee driven through 11-80 degrees
  by a joint torque, 4 s):
  the patella angle follows OpenSim's spline of the simulated knee angle within 0.25 deg at every step (half of
  Station 2's 0.5 deg limit), and nothing diverges. The joints the couplings drive have no damping (change 6.10:
  with the template's 0.05 and the patella's own inertia, change 6.3, it lagged by 1.9 deg).
Exit 0 if every check passes.
"""
import os
import subprocess
import sys

import mujoco
import numpy as np
from lxml import etree

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


def patella_spline(osim_path, side):
    t = etree.parse(osim_path)
    ta = t.getroot().xpath(f".//CustomJoint[@name='patellofemoral_{side}']//TransformAxis[@name='rotation1']")[0]
    sp = ta.find("SimmSpline")
    return (np.array([float(v) for v in sp.findtext("x").split()]),
            np.array([float(v) for v in sp.findtext("y").split()]))


def main():
    src, osim_path, geometry, scratch = (os.path.abspath(p) for p in sys.argv[1:5])
    os.makedirs(scratch, exist_ok=True)
    import opensim
    x_kn, y_kn = patella_spline(osim_path, "r")
    spline = opensim.SimmSpline()
    for a, b in zip(x_kn, y_kn):
        spline.addPoint(float(a), float(b))
    f = lambda q: spline.calcValue(opensim.Vector(1, float(q)))
    domain = np.array([0.0, 2.0944])      # knee_angle_r's range in Rajagopal 2016

    sys.path.insert(0, src)
    from myoconverter.xml import utils as xu
    has_chain = hasattr(xu, "fit_spline_chained")
    print("The fit")
    check(has_chain, "converter has fit_spline_chained")
    if has_chain:
        single, _, _, single_err = xu.fit_spline_on_range("SimmSpline", x_kn, y_kn, domain)
        p1, p2, h_range, err = xu.fit_spline_chained("SimmSpline", x_kn, y_kn, domain, single)
        fine = np.linspace(domain[0], domain[1], 20001)
        true = np.array([f(q) for q in fine])
        err_fine = np.degrees(np.max(np.abs(p2(p1(fine)) - true)))
        check(err_fine < 0.05 and err_fine < np.degrees(single_err) / 10,
              f"largest error {err_fine:.4f} deg on a 10x finer grid (single quartic {np.degrees(single_err):.4f} deg)")
        check(abs(err_fine - np.degrees(err)) < 0.002, f"agrees with the fit's own figure ({np.degrees(err):.4f} deg)")
        q1, q2, _, err2 = xu.fit_spline_chained("SimmSpline", x_kn, y_kn, domain, single)
        check(np.array_equal(q1.coef, p1.coef) and np.array_equal(q2.coef, p2.coef) and err2 == err,
              "same coefficients when run twice")
        check(np.all(np.isfinite(h_range)) and h_range[0] < h_range[1], f"helper range {h_range}")

    print("The converted model")
    out = os.path.join(scratch, "converted")
    r = subprocess.run([sys.executable, "-c", RUNNER, osim_path, geometry, out, src], capture_output=True, text=True)
    check(r.returncode == 0, "XML step runs")
    if r.returncode != 0:
        print(r.stderr[-1500:])
        return 1
    sys.path.insert(0, REPO)
    from osim2mujoco.checks import mujoco_compare as mc
    xml = os.path.join(out, "Rajagopal2016_cvt1.xml")
    m, _ = mc.load_without_meshes(xml)
    J = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    B = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
    for side in ("r", "l"):
        helper = f"patellofemoral_{side}_rotation1_helper"
        hj, hb = J(helper), B(helper)
        check(hj >= 0 and hb >= 0 and m.body_parentid[hb] == 0 and m.jnt_bodyid[hj] == hb,
              f"{helper}: own body under the world body")
        if hj < 0:
            continue
        drives = {(int(m.eq_obj1id[e]), int(m.eq_obj2id[e])) for e in range(m.neq)}
        check((hj, J(f"knee_angle_{side}")) in drives and (J(f"patellofemoral_{side}_rotation1"), hj) in drives,
              f"knee_angle_{side} -> helper -> patella rotation")
        check(abs(m.body_mass[hb] - 0.001) < 1e-12 and m.body_geomnum[hb] == 1
              and m.geom_contype[m.body_geomadr[hb]] == 0 and m.geom_conaffinity[m.body_geomadr[hb]] == 0,
              f"helper body mass {m.body_mass[hb]} kg, one geom that collides with nothing")
    d = mujoco.MjData(m)
    mujoco.mj_resetDataKeyframe(m, d, 0)
    mujoco.mj_forward(m, d)
    res, n = mc.equality_residual(m, d)
    check(res < 1e-9, f"keyframe satisfies all {n} joint equalities (largest residual {res:.1e})")

    for jn in ("patellofemoral_r_rotation1_helper", "patellofemoral_r_rotation1"):
        check(m.dof_damping[m.jnt_dofadr[J(jn)]] == 0.0, f"{jn}: no damping (a coupling drives it, change 6.10)")
    print("In motion")
    m.opt.gravity[:] = 0
    # muscles at zero activation, as this test was written: since change 1c.3 Rajagopal's muscles have OpenSim's
    # min_control 0.01, and 1% activation of the quadriceps pulls the patella off its coupling
    m.actuator_ctrlrange[:, 0] = np.minimum(m.actuator_ctrlrange[:, 0], 0.0)
    knee, pat = J("knee_angle_r"), J("patellofemoral_r_rotation1")
    mujoco.mj_resetDataKeyframe(m, d, 0)
    worst, kmin, kmax = 0.0, 9.0, -9.0
    for _ in range(int(4.0 / m.opt.timestep)):
        target = 0.8 + 0.6 * np.sin(np.pi * d.time)
        d.qfrc_applied[:] = 0
        d.qfrc_applied[m.jnt_dofadr[knee]] = 200 * (target - d.qpos[m.jnt_qposadr[knee]]) - 5 * d.qvel[m.jnt_dofadr[knee]]
        mujoco.mj_step(m, d)
        if not np.all(np.isfinite(d.qpos)):
            break
        if d.time > 0.5:
            k = d.qpos[m.jnt_qposadr[knee]]
            kmin, kmax = min(kmin, k), max(kmax, k)
            worst = max(worst, abs(np.degrees(d.qpos[m.jnt_qposadr[pat]] - f(k))))
    check(np.all(np.isfinite(d.qpos)), "simulation stays finite")
    check(kmax - kmin > 1.0, f"knee moved over {np.degrees(kmin):.0f}..{np.degrees(kmax):.0f} deg")
    check(worst < 0.25, f"patella follows OpenSim's spline of the simulated knee within {worst:.3f} deg")

    print("ALL CHECKS PASS" if fails == 0 else f"{fails} CHECK(S) FAILED")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
