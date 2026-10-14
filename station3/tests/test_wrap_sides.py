"""Station 3, change 3.5: a wrap's side site is kept only where it brings the muscle path closer to OpenSim's.

On Rajagopal 2016's XML-step model (cvt1; Step 1 only, a few minutes) with the converter in src_dir:
- choose_side_sites() drops the side sites of iliacus and psoas (both sides), whose paths then match OpenSim's length
  within 0.1 mm RMS (and 1 mm max) over the sampled poses, where with their side sites they are more than 5 mm RMS off
  (the path goes the long way around the pelvic-brim cylinder);
- it keeps gluteus maximus 1's side sites (without them its path is further from OpenSim's);
- it touches nothing else: every other element of the file is unchanged, and a model without side sites
  (gait10dof18musc) is not rewritten at all;
- independently of the module's own sampling: in the 2189 Station 2 poses (osim2mujoco.checks.osim_paths
  reference), iliacus_r and psoas_r without side sites are within 0.1 mm of OpenSim everywhere.

Usage: python station3/tests/test_wrap_sides.py <src_dir> <opensim_models_dir> <rajagopal_geometry_dir>
         <gait10dof18musc_dir> <scratch_dir>
"""
import os
import shutil
import subprocess
import sys

import numpy as np

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


def convert(src, osim, geometry, out):
    subprocess.run([sys.executable, "-c", RUNNER, osim, geometry, out, src], capture_output=True, text=True)
    name = os.path.splitext(os.path.basename(osim))[0]
    xml = os.path.join(out, f"{name}_cvt1.xml")
    return xml if os.path.isfile(xml) else None


def main():
    src, osm, rgeom, g10, scratch = (os.path.abspath(p) for p in sys.argv[1:6])
    sys.path.insert(0, src)
    here = os.path.dirname(os.path.abspath(__file__))
    sys.path.insert(0, os.path.dirname(os.path.dirname(here)))
    import myoconverter.xml.config  # noqa: F401
    import mujoco
    import opensim
    from lxml import etree
    from myoconverter.optimization.utils import wrap_sides
    from myoconverter.optimization.utils.force_refit import _joint_plan, _solve_qpos
    from osim2mujoco.checks import mujoco_compare as mc, osim_paths

    if os.path.exists(scratch):
        shutil.rmtree(scratch)
    os.makedirs(scratch)
    raj = os.path.join(osm, "Models", "Rajagopal", "Rajagopal2016.osim")
    xml = convert(src, raj, rgeom, os.path.join(scratch, "raj"))
    check(xml is not None, "Rajagopal 2016's XML step converts")
    if xml is None:
        return 1
    original = etree.tostring(etree.parse(xml).getroot())
    om = opensim.Model(raj)
    muscles = [om.getMuscles().get(i).getName() for i in range(om.getMuscles().getSize())]
    reports = {r["muscle"]: r for r in wrap_sides.choose_side_sites(xml, om, muscles)}
    for side in ("r", "l"):
        for mus in (f"iliacus_{side}", f"psoas_{side}"):
            r = reports.get(mus)
            check(r is not None and r["dropped"] and r["rms_before_mm"] > 5 and r["rms_after_mm"] < 0.1,
                  f"{mus}: side site dropped, length RMS {r['rms_before_mm']:.2f} -> {r['rms_after_mm']:.4f} mm"
                  if r else f"{mus}: considered")
        r = reports.get(f"glmax1_{side}")
        check(r is not None and not r["dropped"], f"glmax1_{side}: side sites kept ("
              + (f"{r['rms_before_mm']:.3f} mm" if r else "not considered") + ")")

    after = etree.parse(xml).getroot()
    before = etree.fromstring(original)
    for g in before.iter("geom"):
        if g.getparent().tag == "spatial" and g.getparent().get("name").split("_tendon")[0] in \
                {m for m, r in reports.items() if r["dropped"]}:
            g.attrib.pop("sidesite", None)
    check(etree.tostring(before) == etree.tostring(after), "nothing else in the file changed")

    # the Station 2 poses, OpenSim's lengths from the Station 3 harness (its analytic moment arms aren't used here)
    ref = osim_paths.reference(raj)
    check(ref is not None, "OpenSim reference over the Station 2 poses")
    if ref is not None:
        m, _ = mc.load_without_meshes(xml)
        d = mujoco.MjData(m)
        plan = _joint_plan(m)
        names, all_muscles = list(ref["coord_names"]), list(ref["muscles"])
        for mus in ("iliacus_r", "psoas_r"):
            a = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, mus)
            L = np.zeros(len(ref["coords"]))
            for p in range(len(L)):
                d.qpos[:] = _solve_qpos(m, plan, dict(zip(names, ref["coords"][p])))
                mujoco.mj_fwdPosition(m, d)
                L[p] = d.actuator_length[a]
            err = 1000 * np.abs(L - ref["length"][:, all_muscles.index(mus)])
            check(err.max() < 0.1, f"{mus}: within {err.max():.4f} mm of OpenSim in all {len(L)} Station 2 poses")

    g_osim = os.path.join(g10, "gait10dof18musc.osim")
    g_xml = convert(src, g_osim, os.path.join(g10, "Geometry"), os.path.join(scratch, "gait10"))
    if g_xml:
        stamp = os.stat(g_xml).st_mtime_ns
        gm = opensim.Model(g_osim)
        r = wrap_sides.choose_side_sites(g_xml, gm, [gm.getMuscles().get(i).getName()
                                                     for i in range(gm.getMuscles().getSize())])
        check(r == [] and os.stat(g_xml).st_mtime_ns == stamp, "gait10dof18musc (no side sites): file not rewritten")
    else:
        check(False, "gait10dof18musc's XML step converts")
    print("ALL CHECKS PASS" if fails == 0 else f"{fails} CHECK(S) FAILED")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
