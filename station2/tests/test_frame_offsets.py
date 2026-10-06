"""Station 2, change 2.1: joints whose frames are offset and rotated must convert to the right body placement.

None of our three models exercises the general case (Rajagopal's offset frames have equal parent and child
orientations, so the order of composition doesn't matter there). This test builds one from gait10dof18musc by
giving four joints arbitrary frames, converts it, and runs the Station 2 comparison:

- ankle_r (PinJoint): different parent and child orientations, plus a child translation
- knee_r (CustomJoint, spline-driven translations): equal parent and child orientations plus a child translation
  (the Rajagopal walker_knee case)
- subtalar_r (WeldJoint): different orientations plus a child translation (body placement only, no joint)
- back (PinJoint): different orientations, no translation

All values have at most 4 significant figures, so the converter's '%.4g' output formatting only rounds the
composed results. The unmodified model is converted and checked too, because gait10's knee already has a
0.88 mm error from the quartic fit to its spline (Station 2 run1), which these edits don't touch.
Pass: for every body, max position error <= unmodified + 0.1 mm and max orientation error <= unmodified + 0.01 deg,
over every Station 2 pose.

Usage: python station2/tests/test_frame_offsets.py <src_dir> <gait10dof18musc.osim> <geometry_dir> <scratch_dir>
  src_dir: the converter to test (this repo's src/, or an older copy to show the test fails without the fix)
"""
import os
import subprocess
import sys

import numpy as np
from lxml import etree

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))
import mujoco_compare as mc                       # noqa: E402
import osim_reference as orf                      # noqa: E402

EDITS = {  # joint: (parent orientation, child translation, child orientation)
    "ankle_r": ("0.3 -0.2 0.5", "0.01 -0.02 0.015", "-0.4 0.6 0.1"),
    "knee_r": ("0.2 0.1 -0.3", "0.005 0.01 -0.008", "0.2 0.1 -0.3"),
    "subtalar_r": ("-0.25 0.4 0.15", "-0.012 0.006 0.009", "0.35 -0.1 0.7"),
    "back": ("0.15 0 0.2", "0 0 0", "-0.3 0.25 0"),
}
RUNNER = r'''
import sys
sys.path.insert(0, sys.argv[4])
from myoconverter.O2MPipeline import O2MPipeline
O2MPipeline(sys.argv[1], sys.argv[2], sys.argv[3], convert_steps=[1], muscle_list=None, osim_data_overwrite=True,
            conversion=True, validation=False, speedy=False, generate_pdf=False, add_ground_geom=True,
            treat_as_normal_path_point=False)
'''


def build(osim_in, osim_out):
    t = etree.parse(osim_in)
    for jn, (p_ori, c_tr, c_ori) in EDITS.items():
        j = t.xpath(f"//JointSet/objects/*[@name='{jn}']")[0]
        pf = j.xpath(f"frames/PhysicalOffsetFrame[@name='{j.findtext('socket_parent_frame')}']")[0]
        cf = j.xpath(f"frames/PhysicalOffsetFrame[@name='{j.findtext('socket_child_frame')}']")[0]
        pf.find("orientation").text = p_ori
        cf.find("translation").text = c_tr
        cf.find("orientation").text = c_ori
    t.write(osim_out)


def convert_and_compare(src, osim_path, geometry, out):
    r = subprocess.run([sys.executable, "-c", RUNNER, osim_path, geometry, out, src], capture_output=True, text=True)
    if r.returncode != 0:
        print(r.stdout[-2000:], r.stderr[-2000:])
        raise RuntimeError("conversion failed")
    name = os.path.splitext(os.path.basename(osim_path))[0]
    res = mc.compare(os.path.join(out, f"{name}_cvt1.xml"), orf.reference(osim_path))
    if res["K0"] != "PASS":
        raise RuntimeError(f"K0: {res['K0_problems']}")
    return res


def main():
    src, osim_in, geometry, scratch = (os.path.abspath(p) for p in sys.argv[1:5])
    os.makedirs(scratch, exist_ok=True)
    osim_path = os.path.join(scratch, "gait10_offsets.osim")
    build(osim_in, osim_path)
    plain = convert_and_compare(src, osim_in, geometry, os.path.join(scratch, "plain"))
    res = convert_and_compare(src, osim_path, geometry, os.path.join(scratch, "offsets"))
    ok = True
    for b, v in res["per_body"].items():
        u = plain["per_body"][b]
        dp, dr = v["pos_max_mm"] - u["pos_max_mm"], v["rot_max_deg"] - u["rot_max_deg"]
        good = dp <= 0.1 and dr <= 0.01
        ok &= good
        print(f"  {'ok  ' if good else 'FAIL'} {b:9s} pos max {v['pos_max_mm']:9.5f} mm (unmodified {u['pos_max_mm']:.5f})"
              f"   rot max {v['rot_max_deg']:8.5f} deg (unmodified {u['rot_max_deg']:.5f})")
    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
