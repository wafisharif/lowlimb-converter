"""Tests of the installable tool (the `lowlimb-convert` command and lowlimb_converter.convert / .validate).

Run after `pip install .` (or with the repo root on sys.path):
    python tests/test_tool.py <upstream_myoconverter_clone> <scratch_dir> [--full]

- the output-folder rules: a non-empty folder is refused, a folder claimed by another conversion is refused, and the
  claim is released when a conversion ends, also when it fails;
- the support pre-check rejects a model the converter can't handle (RajagopalLaiUhlrich2023-style PolynomialFunction
  knee, made from gait10dof18musc) before anything runs, with the reason;
- `lowlimb-convert check` and its exit codes;
- `lowlimb-convert convert --steps 1` on gait10dof18musc, then `validate --levels kinematics`: the report must pass
  and its numbers must equal the Station 2 check run directly (same code path, same result);
- with --full: the full conversion with --validate (about 4 minutes), whose report must be valid at every level.
Exit 0 if every check passes.
"""
import json
import os
import shutil
import subprocess
import sys

from lxml import etree

REPO = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, REPO)
sys.path.insert(0, os.path.join(REPO, "src"))
from lowlimb_converter import convert as cv  # noqa: E402

fails = 0


def check(ok, msg):
    global fails
    print(("  ok   " if ok else "  FAIL ") + msg)
    fails += 0 if ok else 1


def cli(*args):
    """Run the command as a user would: the installed console script if there is one, else `python -m`."""
    exe = shutil.which("lowlimb-convert")
    if exe:                                   # the installed package, exactly as a user runs it
        return subprocess.run([exe] + list(args), capture_output=True, text=True)
    env = dict(os.environ, PYTHONPATH=os.pathsep.join([REPO, os.path.join(REPO, "src")]))
    return subprocess.run([sys.executable, "-m", "lowlimb_converter.cli"] + list(args), capture_output=True,
                          text=True, env=env)


def polynomial_knee(osim_in, osim_out):
    """gait10dof18musc with the right knee's first translation driven by a PolynomialFunction (unsupported)."""
    t = etree.parse(osim_in)
    for ta in t.getroot().iter("TransformAxis"):
        if ta.get("name") == "translation1" and ta.getparent().getparent().get("name") == "knee_r":
            for fn in list(ta):
                if fn.tag in ("SimmSpline", "NaturalCubicSpline", "LinearFunction", "Constant", "MultiplierFunction"):
                    ta.remove(fn)
            poly = etree.SubElement(ta, "PolynomialFunction", name="function")
            etree.SubElement(poly, "coefficients").text = "0.001 0.002 0"
            break
    t.write(osim_out)


def main():
    up, scratch = os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2])
    full = "--full" in sys.argv[3:]
    if os.path.exists(scratch):
        shutil.rmtree(scratch)
    os.makedirs(scratch)
    model_dir = os.path.join(up, "models", "osim", "Gait10dof18musc")
    osim = os.path.join(model_dir, "gait10dof18musc.osim")

    print("Output folder rules")
    busy = os.path.join(scratch, "busy")
    os.makedirs(busy)
    open(os.path.join(busy, "old_cvt3.xml"), "w").close()
    try:
        cv.check_output_folder(busy)
        check(False, "non-empty folder refused")
    except cv.ConversionError as e:
        check("not empty" in str(e), f"non-empty folder refused ({e})")
    claimed = os.path.join(scratch, "claimed")
    marker = cv.claim_output_folder(claimed)
    try:
        cv.claim_output_folder(claimed)
        check(False, "second claim refused")
    except cv.ConversionError as e:
        check("another conversion" in str(e), "second claim of the same folder refused")
    try:
        cv.check_output_folder(claimed)
        check(False, "claimed folder refused by the early check")
    except cv.ConversionError as e:
        check("another conversion" in str(e), "claimed folder refused by the early check, with the right reason")
    os.remove(marker)
    check(os.listdir(claimed) == [], "claim released")

    print("Pre-check rejects an unsupported model before anything runs")
    bad = os.path.join(scratch, "poly_knee.osim")
    polynomial_knee(osim, bad)
    out_bad = os.path.join(scratch, "out_bad")
    r = cli("convert", bad, out_bad, "--geometry", os.path.join(model_dir, "Geometry"), "--steps", "1")
    check(r.returncode == 1 and "can't be converted" in r.stderr and "PolynomialFunction" in r.stderr,
          f"convert exits 1 and names the reason ({r.stderr.strip()[:140]})")
    check(not os.path.exists(out_bad), "no output folder created")

    print("check command")
    r = cli("check", osim, "--geometry", os.path.join(model_dir, "Geometry"))
    check(r.returncode == 0 and "converts" in r.stdout, f"gait10dof18musc: exit 0, converts ({r.stdout.strip()[:80]})")
    r = cli("check", bad)
    check(r.returncode == 1 and "fails" in r.stdout, "unsupported model: exit 1, fails")
    r = cli("check", os.path.join(scratch, "missing.osim"))
    check(r.returncode == 2, "missing file: exit 2")

    print("convert --steps 1, then validate --levels kinematics")
    out1 = os.path.join(scratch, "out_step1")
    r = cli("convert", osim, out1, "--steps", "1")
    cvt1 = os.path.join(out1, "gait10dof18musc_cvt1.xml")
    check(r.returncode == 0 and os.path.isfile(cvt1), "converts (Geometry folder found next to the model)")
    check(not os.path.exists(os.path.join(out1, cv.MARKER)), "claim released after the conversion")
    r = cli("convert", osim, out1, "--steps", "1")
    check(r.returncode == 1 and "not empty" in r.stderr, "converting into the same folder again is refused")
    rep_dir = os.path.join(scratch, "report_step1")
    r = cli("validate", osim, cvt1, rep_dir, "--levels", "kinematics")
    check(r.returncode == 0, f"validate exits 0 ({r.stdout.strip()[-80:]})")
    rep = json.load(open(os.path.join(rep_dir, "report.json")))
    k = rep["levels"]["kinematics"]
    check(k["status"] == "pass" and k["worst_position_mm"]["value"] < 1.0,
          f"kinematics pass, worst {k['worst_position_mm']['value']:.4f} mm")
    from lowlimb_converter.checks import mujoco_compare, osim_reference
    direct = mujoco_compare.compare(cvt1, osim_reference.reference(osim), keep_arrays=False)
    detailed = json.load(open(os.path.join(rep_dir, "kinematics.json")))
    check(json.loads(json.dumps(direct)) == detailed, "report's detailed result equals the Station 2 check run directly")
    check(os.path.isfile(os.path.join(rep_dir, "report.md")), "report.md written")

    if full:
        print("convert (all steps) --validate")
        out3 = os.path.join(scratch, "out_full")
        r = cli("convert", osim, out3, "--validate")
        check(r.returncode == 0, f"exit 0 ({r.stdout.strip()[-80:]})")
        rep = json.load(open(os.path.join(out3, "validation", "report.json")))
        check(rep["ok"] and set(rep["levels"]) == {"kinematics", "paths", "forces"},
              f"all three levels valid ({ {k: v['status'] for k, v in rep['levels'].items()} })")
        f = rep["levels"]["forces"]["by_activation"]
        check(f["a1"]["rms_pct_mean"] < 5 and f["a0"]["rms_pct_mean"] < 5,
              f"mean static force error < 5% Fmax (a=1 {f['a1']['rms_pct_mean']:.2f}%, a=0 {f['a0']['rms_pct_mean']:.2f}%)")

    print("ALL CHECKS PASS" if fails == 0 else f"{fails} CHECK(S) FAILED")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
