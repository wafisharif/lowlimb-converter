"""Station 1 structural gates: OpenSim original vs MuJoCo cvt3.
Gates (written 2026-10-05, before gait2354 was run):
  JOINT GATE   : MuJoCo joints not driven by a joint-equality constraint == OpenSim coordinates (by name)
  MUSCLE GATE  : OpenSim muscle names == MuJoCo actuator names
  KEYFRAME GATE: model loads, keyframe 0 violates no equality constraint (max |violation| < 1e-6)
"""
import sys, glob, opensim as osim, mujoco
import numpy as np
osim_path, out_dir = sys.argv[1], sys.argv[2]
m = osim.Model(osim_path); m.initSystem()
coords = [m.getCoordinateSet().get(i).getName() for i in range(m.getCoordinateSet().getSize())]
muscles = [m.getMuscles().get(i).getName() for i in range(m.getMuscles().getSize())]
print("OpenSim  bodies", m.getBodySet().getSize(), "joints", m.getJointSet().getSize(),
      "coords", len(coords), "muscles", len(muscles))
xml = glob.glob(f"{out_dir}/*_cvt3.xml")[0]
mj = mujoco.MjModel.from_xml_path(xml); d = mujoco.MjData(mj)
mujoco.mj_resetDataKeyframe(mj, d, 0); mujoco.mj_forward(mj, d)
print("MuJoCo   bodies", mj.nbody - 1, "joints", mj.njnt, "dofs", mj.nv, "actuators", mj.nu, "equality", mj.neq)
dep = {mj.eq_obj1id[i] for i in range(mj.neq) if mj.eq_type[i] == mujoco.mjtEq.mjEQ_JOINT}
indep = [mujoco.mj_id2name(mj, mujoco.mjtObj.mjOBJ_JOINT, j) for j in range(mj.njnt) if j not in dep]
acts = [mujoco.mj_id2name(mj, mujoco.mjtObj.mjOBJ_ACTUATOR, j) for j in range(mj.nu)]
viol = float(np.abs(d.efc_pos[:d.ne]).max()) if d.ne else 0.0
print("joint-follower constraints", len(dep))
print("OpenSim coords     ", len(coords), sorted(coords))
print("MuJoCo independent ", len(indep), sorted(indep))
print("missing in MuJoCo  ", sorted(set(coords) - set(indep)))
print("extra in MuJoCo    ", sorted(set(indep) - set(coords)))
print("JOINT GATE:   ", "PASS" if sorted(coords) == sorted(indep) else "FAIL")
print("MUSCLE GATE:  ", "PASS" if sorted(muscles) == sorted(acts) else f"FAIL missing={sorted(set(muscles)-set(acts))} extra={sorted(set(acts)-set(muscles))}")
print("KEYFRAME GATE:", "PASS" if viol < 1e-6 else "FAIL", f"(max |eq violation| = {viol:.3g})")
print("Loaded OK:", xml)
