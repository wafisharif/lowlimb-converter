"""Station 6, change 6.2: PlanarJoint, FreeJoint, BallJoint and GimbalJoint convert exactly.

For each joint type, builds a small model with OpenSim's API: ground -> (joint) -> body "a" -> PinJoint -> body "b",
the joint under test with rotated and offset parent and child frames (so axis conventions and frame composition
matter), and gives every coordinate a range. Then converts it with the converter in src_dir and runs Station 2's
kinematic check (lowlimb_converter.checks: the same poses, method and K0-K2 limits): every body within 1e-6 mm and
1e-6 deg of OpenSim in every pose (exact up to round-off). The converter before change 6.2 refuses each model
(no parser for the joint type).

Usage: python station6/tests/test_joint_types.py <src_dir> <scratch_dir>
"""
import os
import shutil
import subprocess
import sys

import opensim as osim

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, REPO)
from lowlimb_converter.checks import mujoco_compare as mc   # noqa: E402
from lowlimb_converter.checks import osim_reference as orf  # noqa: E402

RUNNER = r'''
import sys
sys.path.insert(0, sys.argv[4])
from myoconverter.O2MPipeline import O2MPipeline
O2MPipeline(sys.argv[1], sys.argv[2], sys.argv[3], convert_steps=[1], muscle_list=None, osim_data_overwrite=True,
            conversion=True, validation=False, speedy=False, generate_pdf=False, add_ground_geom=False,
            treat_as_normal_path_point=False)
'''
fails = 0


def check(ok, msg):
    global fails
    print(("  ok   " if ok else "  FAIL ") + msg)
    fails += 0 if ok else 1


def build(kind, path):
    m = osim.Model()
    m.setName(f"test_{kind}")
    a = osim.Body("a", 2.0, osim.Vec3(0.01, 0.1, -0.02), osim.Inertia(0.02, 0.03, 0.025, 0.001, 0.0, 0.002))
    b = osim.Body("b", 1.0, osim.Vec3(0.0, -0.15, 0.01), osim.Inertia(0.01, 0.004, 0.012))
    m.addBody(a)
    m.addBody(b)
    p_loc, p_ori = osim.Vec3(0.1, 0.9, -0.05), osim.Vec3(0.3, -0.2, 0.4)
    c_loc, c_ori = osim.Vec3(0.02, 0.2, 0.03), osim.Vec3(-0.1, 0.25, 0.15)
    joint = getattr(osim, kind)(f"{kind}_ab", m.getGround(), p_loc, p_ori, a, c_loc, c_ori)
    m.addJoint(joint)
    pin = osim.PinJoint("pin", a, osim.Vec3(0, -0.2, 0), osim.Vec3(0.1, 0, 0), b, osim.Vec3(0, 0.1, 0),
                        osim.Vec3(0, 0, 0.2))
    m.addJoint(pin)
    for k in range(joint.numCoordinates()):
        c = joint.upd_coordinates(k)
        if c.getMotionType() == 2:
            c.setRangeMin(-0.3)
            c.setRangeMax(0.4)
        else:
            c.setRangeMin(-1.2)
            c.setRangeMax(1.0)
    pin.upd_coordinates(0).setRangeMin(-1.0)
    pin.upd_coordinates(0).setRangeMax(1.5)
    m.finalizeConnections()
    m.initSystem()
    m.printToXML(path)
    return joint.numCoordinates()


def main():
    src, scratch = (os.path.abspath(p) for p in sys.argv[1:3])
    if os.path.exists(scratch):
        shutil.rmtree(scratch)
    os.makedirs(os.path.join(scratch, "Geometry"))
    for kind in ("PlanarJoint", "FreeJoint", "BallJoint", "GimbalJoint"):
        print(kind)
        osim_path = os.path.join(scratch, f"test_{kind}.osim")
        n = build(kind, osim_path)
        out = os.path.join(scratch, kind)
        r = subprocess.run([sys.executable, "-c", RUNNER, osim_path, os.path.join(scratch, "Geometry"), out, src],
                           capture_output=True, text=True)
        xml = os.path.join(out, f"test_{kind}_cvt1.xml")
        check(r.returncode == 0 and os.path.isfile(xml), f"converts ({n} coordinates)")
        if r.returncode != 0:
            print("    " + (r.stdout + r.stderr).strip().splitlines()[-1][:200])
            continue
        res = mc.compare(xml, orf.reference(osim_path), keep_arrays=False)
        ok = res["K0"] == "PASS" and res["pos_worst_mm"]["value"] < 1e-6 and res["rot_worst_deg"]["value"] < 1e-6
        check(ok, f"Station 2 K0 {res['K0']}"
              + (f": worst {res['pos_worst_mm']['value']:.1e} mm, {res['rot_worst_deg']['value']:.1e} deg in "
                 f"{res['poses']} poses" if res["K0"] == "PASS" else f" {res.get('K0_problems')}"))
    print("ALL CHECKS PASS" if fails == 0 else f"{fails} CHECK(S) FAILED")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
