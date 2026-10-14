"""Station 2, change 2.5: CustomJoint axes given by a PolynomialFunction convert exactly.

RajagopalLaiUhlrich2023's walker knees are the only real case we have; this test builds a controlled one from
gait10dof18musc: the right knee's translation1 and translation2 splines are replaced by OpenSim PolynomialFunctions
(degree 3 and 4, least-squares fits to the splines' knots), so the OpenSim model's knee *is* a polynomial. Then:

- conversion succeeds (the converter before 2.5 stopped with NotImplementedError: translation1 is the first axis
  processed for knee_angle_r, and only SimmSpline / LinearFunction could be "designated");
- each polynomial becomes the MuJoCo joint equality's polycoef exactly (same coefficients, 1e-15 relative);
- in every Station 2 pose, the right shank and foot (tibia_r, talus_r, calcn_r, toes_r) are within 1e-6 mm and
  1e-6 deg of OpenSim (the unmodified model, with its spline knee, is 0.22 mm off at the toes);
- a degree-5 polynomial is refused with NotImplementedError naming the degree, and the support report predicts that
  failure at that axis; for the degree 3-4 model it predicts "converts" and marks both axes converted.

Usage: python station2/tests/test_polynomial_axes.py <src_dir> <gait10dof18musc.osim> <geometry_dir> <scratch_dir>
  src_dir: the converter to test (this repo's src/, or an older copy to show the checks fail without change 2.5)
Exit 0 if every check passes.
"""
import os
import subprocess
import sys

import mujoco
import numpy as np
from lxml import etree

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, REPO)
from osim2mujoco.checks import mujoco_compare as mc   # noqa: E402
from osim2mujoco.checks import osim_reference as orf  # noqa: E402
from osim2mujoco.support import build_report          # noqa: E402

RUNNER = r'''
import sys
sys.path.insert(0, sys.argv[4])
from myoconverter.O2MPipeline import O2MPipeline
O2MPipeline(sys.argv[1], sys.argv[2], sys.argv[3], convert_steps=[1], muscle_list=None, osim_data_overwrite=True,
            conversion=True, validation=False, speedy=False, generate_pdf=False, add_ground_geom=True,
            treat_as_normal_path_point=False)
'''
AXES = {"translation1": 3, "translation2": 4}      # axis of knee_r: polynomial degree
SHANK = ("tibia_r", "talus_r", "calcn_r", "toes_r")
fails = 0


def check(ok, msg):
    global fails
    print(("  ok   " if ok else "  FAIL ") + msg)
    fails += 0 if ok else 1


def build(osim_in, osim_out, degrees):
    """Replace knee_r's spline axes by PolynomialFunctions; return {axis: coefficients, highest order first}."""
    t = etree.parse(osim_in)
    knee = t.xpath("//JointSet/objects/CustomJoint[@name='knee_r']")[0]
    coefs = {}
    for axis, deg in degrees.items():
        ta = knee.xpath(f"SpatialTransform/TransformAxis[@name='{axis}']")[0]
        spline = ta.find("SimmSpline")
        x = np.array([float(v) for v in spline.findtext("x").split()])
        y = np.array([float(v) for v in spline.findtext("y").split()])
        c = np.polyfit(x, y, deg)
        ta.remove(spline)
        poly = etree.SubElement(ta, "PolynomialFunction", name="function")
        etree.SubElement(poly, "coefficients").text = " ".join(repr(float(v)) for v in c)
        coefs[axis] = c
    t.write(osim_out)
    return coefs


def convert(src, osim_path, geometry, out):
    return subprocess.run([sys.executable, "-c", RUNNER, osim_path, geometry, out, src], capture_output=True,
                          text=True)


def main():
    src, osim_in, geometry, scratch = (os.path.abspath(p) for p in sys.argv[1:5])
    os.makedirs(scratch, exist_ok=True)

    print("Degree 3 and 4 polynomials")
    poly_osim = os.path.join(scratch, "gait10_polyknee.osim")
    coefs = build(osim_in, poly_osim, AXES)
    rep = build_report(poly_osim)
    items = {i["path"]: i for i in rep["items"] if i["kind"] == "TransformAxis"}
    check(rep["outcome"] == "converts" and all(items[f"knee_r/{a}"]["status"] == "converted" for a in AXES),
          f"support report predicts '{rep['outcome']}', both polynomial axes converted")
    out = os.path.join(scratch, "poly")
    r = convert(src, poly_osim, geometry, out)
    check(r.returncode == 0, "XML step runs")
    if r.returncode != 0:
        print("  " + (r.stdout + r.stderr).strip().splitlines()[-1][:300])
    else:
        xml = os.path.join(out, "gait10_polyknee_cvt1.xml")
        m, _ = mc.load_without_meshes(xml)
        for axis, c in coefs.items():
            j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, f"knee_r_{axis}")
            e = [k for k in range(m.neq) if int(m.eq_obj1id[k]) == j]
            want = np.zeros(5)
            want[:len(c)] = c[::-1]
            got = np.array(m.eq_data[e[0], :5]) if len(e) == 1 else np.full(5, np.nan)
            check(np.allclose(got, want, rtol=1e-15, atol=0), f"knee_r_{axis}: polycoef equals OpenSim's "
                  f"degree-{len(c) - 1} polynomial ({np.max(np.abs(got - want)):.1e})")
        res = mc.compare(xml, orf.reference(poly_osim))
        check(res["K0"] == "PASS", f"Station 2 K0 {res['K0']}")
        if res["K0"] == "PASS":
            pos = max(res["per_body"][b]["pos_max_mm"] for b in SHANK)
            rot = max(res["per_body"][b]["rot_max_deg"] for b in SHANK)
            check(pos < 1e-6 and rot < 1e-6, f"right shank and foot within {pos:.1e} mm and {rot:.1e} deg of OpenSim "
                  f"in all {res['poses']} poses")

    print("Degree 5 polynomial")
    bad_osim = os.path.join(scratch, "gait10_polyknee5.osim")
    build(osim_in, bad_osim, {"translation1": 5})
    rep = build_report(bad_osim)
    first = rep.get("first_failure") or {}
    check(rep["outcome"] == "fails" and "knee_r/translation1" in str(first),
          f"support report predicts '{rep['outcome']}' at {first}")
    r = convert(src, bad_osim, geometry, os.path.join(scratch, "poly5"))
    text = r.stdout + r.stderr
    check(r.returncode != 0 and "NotImplementedError" in text and "degree 5" in text,
          "XML step refuses it with NotImplementedError naming the degree")

    print("ALL CHECKS PASS" if fails == 0 else f"{fails} CHECK(S) FAILED")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
