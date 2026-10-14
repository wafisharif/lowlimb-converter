"""Change 1c.3 (MyoConverter limitation 3): properties a component omits come from the file's <defaults>, as in OpenSim.

Builds a test model from gait10dof18musc with a <defaults> element (a CoordinateActuator default with optimal_force
300 and max_control 2; a Millard2012EquilibriumMuscle default with max_isometric_force 1234 and min_control 0.05),
one CoordinateActuator that omits optimal_force and max_control, and one muscle (soleus_r) whose max_isometric_force
and min_control are removed; everything else keeps its own explicit values. Then:
- OpenSim itself reads the omitted values from <defaults> (so this is the behaviour to match);
- in the converted model (XML step), the actuator's strength (torque at control 1), its control range, the muscle's
  peak force and its control range are OpenSim's values, and a property given explicitly still wins over <defaults>
  (glut_max_r keeps its own max_isometric_force);
- the unmodified gait10dof18musc (no <defaults>) converts to a byte-identical file with and without the change
  (checked by the Station 2 runs; here: apply_document_defaults returns 0 and changes nothing).

Usage: python station1/tests/test_document_defaults.py <src_dir> <gait10dof18musc.osim> <geometry_dir> <scratch_dir>
  src_dir: the converter to test (this repo's src/, or an older copy to show the test fails without the change)
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
ACT, COORD = "act_knee_defaults", "knee_angle_r"
MUSCLE, EXPLICIT = "soleus_r", "glut_max_r"
fails = 0


def check(ok, msg):
    global fails
    print(("  ok   " if ok else "  FAIL ") + msg)
    fails += 0 if ok else 1


def build(osim_in, osim_out):
    t = etree.parse(osim_in)
    model = t.getroot().find("Model")
    defaults = etree.Element("defaults")
    model.insert(0, defaults)
    ca = etree.SubElement(defaults, "CoordinateActuator", name="default")
    etree.SubElement(ca, "min_control").text = "-2"
    etree.SubElement(ca, "max_control").text = "2"
    etree.SubElement(ca, "optimal_force").text = "300"
    mu = etree.SubElement(defaults, "Millard2012EquilibriumMuscle", name="default")
    etree.SubElement(mu, "min_control").text = "0.05"
    etree.SubElement(mu, "max_isometric_force").text = "1234"
    forces = model.find("ForceSet/objects")
    act = etree.SubElement(forces, "CoordinateActuator", name=ACT)
    etree.SubElement(act, "coordinate").text = COORD
    sol = forces.find(f"Millard2012EquilibriumMuscle[@name='{MUSCLE}']")
    for tag in ("max_isometric_force", "min_control"):
        el = sol.find(tag)
        if el is not None:
            sol.remove(el)
    t.write(osim_out)


def main():
    src, osim_in, geometry, scratch = (os.path.abspath(p) for p in sys.argv[1:5])
    os.makedirs(scratch, exist_ok=True)
    osim_path = os.path.join(scratch, "gait10_defaults.osim")
    build(osim_in, osim_path)

    import opensim
    om = opensim.Model(osim_path)
    oa = opensim.CoordinateActuator.safeDownCast(om.getForceSet().get(ACT))
    osol = om.getMuscles().get(MUSCLE)
    oexp = om.getMuscles().get(EXPLICIT)
    print("What OpenSim reads")
    check(oa.getOptimalForce() == 300 and oa.getMaxControl() == 2 and oa.getMinControl() == -2,
          f"{ACT}: optimal_force {oa.getOptimalForce()}, controls [{oa.getMinControl()}, {oa.getMaxControl()}] "
          f"(from <defaults>)")
    check(osol.getMaxIsometricForce() == 1234 and osol.getMinControl() == 0.05,
          f"{MUSCLE}: max_isometric_force {osol.getMaxIsometricForce()}, min_control {osol.getMinControl()} "
          f"(from <defaults>)")

    print("The converted model")
    out = os.path.join(scratch, "converted")
    r = subprocess.run([sys.executable, "-c", RUNNER, osim_path, geometry, out, src], capture_output=True, text=True)
    check(r.returncode == 0, "XML step runs")
    if r.returncode != 0:
        print(r.stderr[-1500:])
        print(f"{fails} CHECK(S) FAILED")
        return 1
    sys.path.insert(0, REPO)
    from osim2mujoco.checks import mujoco_compare as mc
    m, _ = mc.load_without_meshes(os.path.join(out, "gait10_defaults_cvt1.xml"))
    A = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n)
    a = A(ACT)
    check(a >= 0, f"{ACT} converted")
    if a >= 0:
        d = mujoco.MjData(m)
        dof = m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, COORD)]

        def torque(ctrl):
            d.qpos[:] = m.qpos0
            d.qvel[:] = 0
            d.ctrl[:] = 0
            if d.act.size:
                d.act[:] = 0
            d.ctrl[a] = ctrl
            mujoco.mj_forward(m, d)
            return d.qfrc_actuator[dof]
        tau = torque(1.0) - torque(0.0)
        check(abs(tau - 300) < 1e-9, f"{ACT}: {tau:.6g} N*m at control 1 (OpenSim: optimal_force 300)")
        check(np.allclose(m.actuator_ctrlrange[a], [-2, 2]) and m.actuator_ctrllimited[a] == 1,
              f"{ACT}: control range {m.actuator_ctrlrange[a]} (OpenSim [-2, 2])")
    s, e = A(MUSCLE), A(EXPLICIT)
    peak = lambda i: float(m.actuator_gainprm[i, 2])
    check(s >= 0 and abs(peak(s) - 1234) < 1e-9, f"{MUSCLE}: peak force {peak(s) if s >= 0 else None} "
          f"(OpenSim 1234)")
    check(s >= 0 and abs(m.actuator_ctrlrange[s][0] - 0.05) < 1e-12,
          f"{MUSCLE}: lowest control {m.actuator_ctrlrange[s][0] if s >= 0 else None} (OpenSim min_control 0.05)")
    check(e >= 0 and abs(peak(e) - oexp.getMaxIsometricForce()) < 1e-9,
          f"{EXPLICIT}: peak force {peak(e) if e >= 0 else None} = its own explicit value "
          f"{oexp.getMaxIsometricForce()}, not <defaults>")

    print("A model without <defaults>")
    sys.path.insert(0, src)
    try:
        from myoconverter.xml.config import apply_document_defaults
        root = etree.parse(osim_in).getroot().find("Model")
        before = etree.tostring(root)
        n = apply_document_defaults(root)
        check(n == 0 and etree.tostring(root) == before, "gait10dof18musc: nothing filled in, XML unchanged")
    except ImportError:
        check(False, "converter has apply_document_defaults")
    print("ALL CHECKS PASS" if fails == 0 else f"{fails} CHECK(S) FAILED")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
