"""Changes 1c.1 (gravity) and 1c.2 (CoordinateActuator strength): the converted model must match OpenSim.

Builds a test model from gait10dof18musc with a non-default gravity (0, -9.7, 0.3) and two CoordinateActuators
(optimal_force 25 on knee_angle_r, 0.5 on pelvis_tx), converts it (XML step), and checks in the compiled model:
- gravity equals OpenSim's rotated into MuJoCo's world (+90 deg about x: (gx, gy, gz) -> (gx, -gz, gy));
- at control 1, each motor applies optimal_force on its joint (N*m or N), and nothing on other joints.

Usage: python station1/tests/test_gravity_actuators.py <src_dir> <gait10dof18musc.osim> <geometry_dir> <scratch_dir>
  src_dir: the converter to test (this repo's src/, or an older copy to show the test fails without the fixes)
Exit 0 if every check passes.
"""
import os
import subprocess
import sys

import mujoco
import numpy as np
from lxml import etree

RUNNER = r'''
import sys
sys.path.insert(0, sys.argv[4])
from myoconverter.O2MPipeline import O2MPipeline
O2MPipeline(sys.argv[1], sys.argv[2], sys.argv[3], convert_steps=[1], muscle_list=None, osim_data_overwrite=True,
            conversion=True, validation=False, speedy=False, generate_pdf=False, add_ground_geom=True,
            treat_as_normal_path_point=False)
'''
ACTUATORS = {"act_knee": ("knee_angle_r", 25.0), "act_pelvis_tx": ("pelvis_tx", 0.5)}
GRAVITY = (0.0, -9.7, 0.3)
fails = 0


def check(ok, msg):
    global fails
    print(("  ok   " if ok else "  FAIL ") + msg)
    fails += 0 if ok else 1


def build(osim_in, osim_out):
    t = etree.parse(osim_in)
    model = t.getroot().find("Model")
    model.find("gravity").text = " ".join(str(v) for v in GRAVITY)
    forces = model.find("ForceSet/objects")
    for name, (coord, f) in ACTUATORS.items():
        ca = etree.SubElement(forces, "CoordinateActuator", name=name)
        etree.SubElement(ca, "min_control").text = "-1"
        etree.SubElement(ca, "max_control").text = "1"
        etree.SubElement(ca, "coordinate").text = coord
        etree.SubElement(ca, "optimal_force").text = str(f)
    t.write(osim_out)


def main():
    src, osim_in, geometry, scratch = (os.path.abspath(p) for p in sys.argv[1:5])
    os.makedirs(scratch, exist_ok=True)
    osim_path = os.path.join(scratch, "gait10_1c.osim")
    build(osim_in, osim_path)
    out = os.path.join(scratch, "converted")
    r = subprocess.run([sys.executable, "-c", RUNNER, osim_path, geometry, out, src], capture_output=True, text=True)
    check(r.returncode == 0, "XML step runs")
    if r.returncode != 0:
        print(r.stderr[-1500:])
        return 1
    sys.path.insert(0, os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))),
                                    "station2"))
    import mujoco_compare as mc
    m, _ = mc.load_without_meshes(os.path.join(out, "gait10_1c_cvt1.xml"))
    gx, gy, gz = GRAVITY
    expect = np.array([gx, -gz, gy])
    check(np.allclose(m.opt.gravity, expect, atol=1e-12), f"gravity {m.opt.gravity} == {expect}")
    d = mujoco.MjData(m)

    def actuator_torque(ctrl_index):
        # muscles at rest still pull passively, so qfrc_actuator at control 0 is not zero: measure it and subtract
        d.qpos[:] = m.qpos0
        d.qvel[:] = 0
        d.ctrl[:] = 0
        if d.act.size:
            d.act[:] = 0
        if ctrl_index is not None:
            d.ctrl[ctrl_index] = 1.0
        mujoco.mj_forward(m, d)
        return d.qfrc_actuator.copy()

    tau0 = actuator_torque(None)
    for name, (coord, f) in ACTUATORS.items():
        a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
        check(a >= 0, f"{name} converted")
        if a < 0:
            continue
        dof = m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, coord)]
        tau = actuator_torque(a) - tau0
        others = np.delete(tau, dof)
        check(abs(tau[dof] - f) < 1e-9 and np.max(np.abs(others)) < 1e-9,
              f"{name}: {tau[dof]:.6g} on {coord} at control 1 (OpenSim: {f}), {np.max(np.abs(others)):.1e} elsewhere")
    print("ALL CHECKS PASS" if fails == 0 else f"{fails} CHECK(S) FAILED")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
