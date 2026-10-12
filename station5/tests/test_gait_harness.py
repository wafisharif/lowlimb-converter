"""Station 5: prove the walking check can fail, and cross-check its inverse dynamics independently.

Usage: python station5/tests/test_gait_harness.py <gait10dof18musc.osim> <opensim_models_clone_dir> <xml> <scratch_dir>
  xml: a converted gait10dof18musc (the _cvt3.xml of a full-pipeline run with the current converter)

- OpenSim side = OpenSim's own InverseDynamicsTool: the generalized forces the harness computes are identical to the
  tool's output for the same model, coordinates, ground reaction forces and filter.
- Independent cross-check of the MuJoCo side: an OpenSim model whose knee translations are exactly the MuJoCo
  model's quartics, and a MuJoCo model with OpenSim's physics restored where the converter adds to it (joint damping
  off, the massless virtual bodies of moving path points really massless, OpenSim's toe inertia). Then the two must
  give the same joint moments to within 1e-4 N*m/kg (any error in the chain rule, the ground-reaction-force frames,
  the projection onto coordinates or the units would show up far above that).
- Mutations: 10% more mass on tibia_r changes the joint moments of tibia_r's ancestors only (pelvis, right hip, right
  knee; not the right ankle, the left leg or the lumbar joint); a renamed muscle and a renamed joint fail W0.
Exit 0 if every check passes.
"""
import os
import sys

import mujoco
import numpy as np
import opensim as osim
from lxml import etree

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, REPO)
from lowlimb_converter.checks import mujoco_compare as mc   # noqa: E402
from lowlimb_converter.checks import mujoco_gait as mg      # noqa: E402
from lowlimb_converter.checks import osim_gait as og        # noqa: E402

PIPE = "Pipelines/Gait10dof18musc"
IK = "OutputReference/IK/subject01_walk_IK.mot"
GRF = "ExperimentalData/subject01_walk_grf.xml"
OSIM_BODIES = {"ground", "pelvis", "femur_r", "tibia_r", "talus_r", "calcn_r", "toes_r", "femur_l", "tibia_l",
               "talus_l", "calcn_l", "toes_l", "torso"}
fails = 0


def check(ok, msg):
    global fails
    print(("  ok   " if ok else "  FAIL ") + msg)
    fails += 0 if ok else 1


def evaluate_with(xml, ref, edit, keep_arrays=False):
    """mujoco_gait.evaluate on the model as loaded, then edited in memory by edit(m)."""
    orig = mc.load_without_meshes

    def patched(path):
        m, n = orig(path)
        edit(m)
        return m, n
    mc.load_without_meshes = patched
    try:
        return mg.evaluate(xml, ref, keep_arrays=keep_arrays)
    finally:
        mc.load_without_meshes = orig


def main():
    osim_path, osm, xml, scratch = (os.path.abspath(p) for p in sys.argv[1:5])
    os.makedirs(scratch, exist_ok=True)
    ik, grf = os.path.join(osm, PIPE, IK), os.path.join(osm, PIPE, GRF)
    ref = og.reference(osim_path, ik, grf)
    names = [str(x) for x in ref["coord_names"]]

    print("OpenSim side vs OpenSim's InverseDynamicsTool")
    tool = osim.InverseDynamicsTool()
    tool_model = osim.Model(osim_path)        # keep a reference: the tool doesn't own the model
    tool.setModel(tool_model)
    tool.setCoordinatesFileName(ik)
    tool.setLowpassCutoffFrequency(og.CUTOFF_HZ)
    tool.setStartTime(float(ref["times"][0]))
    tool.setEndTime(float(ref["times"][-1]))
    tool.setExternalLoadsFileName(grf)
    excluded = osim.ArrayStr()
    excluded.append("Muscles")
    tool.setExcludedForces(excluded)
    tool.setResultsDir(scratch)
    tool.setOutputGenForceFileName("id_tool.sto")
    tool.run()
    table = osim.TimeSeriesTable(os.path.join(scratch, "id_tool.sto"))
    labels = list(table.getColumnLabels())
    t_tool = np.array(table.getIndependentColumn())
    check(len(t_tool) == len(ref["times"]) and np.max(np.abs(t_tool - ref["times"])) < 1e-6,
          f"same {len(t_tool)} frames")
    worst = 0.0
    for j, n in enumerate(names):
        col = [lb for lb in labels if lb in (f"{n}_moment", f"{n}_force")]
        if len(col) != 1:
            check(False, f"tool output has a column for {n}")
            continue
        v = np.array(table.getDependentColumn(col[0]).to_numpy())
        worst = max(worst, float(np.max(np.abs(v - ref["tau"][:, j]))))
    check(worst < 1e-6, f"generalized forces equal the tool's (largest difference {worst:.1e} N*m or N; the tool "
                        f"writes 8 decimals)")

    print("Independent cross-check: exact knee polynomials on both sides, OpenSim's physics in MuJoCo")
    root = etree.parse(xml).getroot()
    poly = {e.get("joint1"): [float(v) for v in e.get("polycoef").split()] for e in root.find("equality")
            if e.tag == "joint"}
    tree = etree.parse(osim_path)
    replaced = 0
    for cj in tree.getroot().iter("CustomJoint"):
        for ta in cj.iter("TransformAxis"):
            name = f"{cj.get('name')}_{ta.get('name')}"
            spline = ta.find("SimmSpline")
            if spline is not None and name in poly:
                ta.remove(spline)
                pf = etree.SubElement(ta, "PolynomialFunction", name="function")
                etree.SubElement(pf, "coefficients").text = " ".join(repr(c) for c in poly[name][::-1])
                replaced += 1
    exact_osim = os.path.join(scratch, "gait10_quartic_knees.osim")
    tree.write(exact_osim)
    check(replaced == 4, f"{replaced} knee splines replaced by the MuJoCo model's quartics (4 expected)")
    ref_exact = og.reference(exact_osim, ik, grf)
    om = osim.Model(osim_path)
    om.initSystem()
    bodies = om.getBodySet()
    toes = {b: [bodies.get(b).getInertia().getMoments().get(k) for k in range(3)] for b in ("toes_r", "toes_l")}

    def opensim_physics(m):
        m.dof_damping[:] = 0
        for b in range(m.nbody):
            nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)
            if nm not in OSIM_BODIES:
                m.body_mass[b] = 0
                m.body_inertia[b] = 0
            if nm in toes:
                m.body_inertia[b] = toes[nm]
                m.body_iquat[b] = [1, 0, 0, 0]
    r = evaluate_with(xml, ref_exact, opensim_physics)
    worst_exact = max(v["max"] for v in r["per_coordinate"].values())
    check(r["W0"] == "PASS" and worst_exact < 1e-4,
          f"joint moments agree within {worst_exact:.1e} N*m/kg (or N/kg) in every frame and coordinate")
    base = mg.evaluate(xml, ref, keep_arrays=True)
    worst_as_is = max(v["max"] for v in base["per_coordinate"].values())
    check(worst_as_is > 100 * worst_exact, f"the converted model as it is differs by up to {worst_as_is:.1e} "
                                           f"(so the cross-check is sensitive)")

    print("Mutation: tibia_r 10% heavier")
    tib = None

    def heavier(m):
        nonlocal tib
        tib = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "tibia_r")
        m.body_mass[tib] *= 1.1
        m.body_inertia[tib] *= 1.1
    r = evaluate_with(xml, ref, heavier, keep_arrays=True)
    changed = {n: float(np.max(np.abs(r["_tau"][:, j] - base["_tau"][:, j]))) for j, n in enumerate(names)}
    expect = {"pelvis_tilt", "pelvis_tx", "pelvis_ty", "hip_flexion_r", "knee_angle_r"}
    moved = {n for n, v in changed.items() if v > 1e-6}
    check(moved == expect, f"changed: {sorted(moved)}")
    check(all(changed[n] < 1e-9 for n in names if n not in expect),
          f"right ankle, left leg and lumbar unchanged (largest change "
          f"{max(changed[n] for n in names if n not in expect):.1e} N*m)")

    print("Mutations that must fail W0")
    text = open(xml).read()
    bad = os.path.join(scratch, "renamed_muscle.xml")
    open(bad, "w").write(text.replace('name="soleus_r"', 'name="soleus_rX"', 1))
    r = mg.evaluate(bad, ref)
    check(r["W0"] == "FAIL" and any("soleus_r" in p for p in r["W0_problems"]), "renamed muscle fails W0")
    bad = os.path.join(scratch, "renamed_joint.xml")
    open(bad, "w").write(text.replace('name="ankle_angle_l"', 'name="ankle_angle_lX"').replace(
        'joint="ankle_angle_l"', 'joint="ankle_angle_lX"').replace('joint1="ankle_angle_l"', 'joint1="ankle_angle_lX"')
        .replace('joint2="ankle_angle_l"', 'joint2="ankle_angle_lX"'))
    r = mg.evaluate(bad, ref)
    check(r["W0"] == "FAIL" and any("ankle_angle_l" in p for p in r["W0_problems"]), "renamed joint fails W0")

    print("ALL CHECKS PASS" if fails == 0 else f"{fails} CHECK(S) FAILED")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
