"""Station 6, change 6.6: contact geometry and Hunt-Crossley contact on OpenSim's WalkerModel.

- The model converts (the converter before Station 6 can't even read the file) and every contact pair passes gate
  L6's geometry check (signed distance equal to OpenSim's, sign convention right).
- The converter's calibration ran (contact_calibration.json) and the converted pairs carry its parameters; the whole,
  unlocked model is stable with them, and better-fitting candidates that weren't were rejected (change 6.12).
- The converter's calibration and gate L7's harness are separate code; both simulate the rigid OpenSim model the same
  way: the root trajectory of the 0.02 m calibration drop agrees within 1e-9 between them.
- The calibration only used the calibration drops (it never sees the held-out scenarios).

Usage: python station6/tests/test_contacts.py <src_dir> <old_src_dir> <opensim_models_dir> <scratch_dir>
"""
import json
import os
import shutil
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, REPO)
from osim2mujoco.checks import contact_dynamics, contact_geometry   # noqa: E402

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


def main():
    src, old, osm, scratch = (os.path.abspath(p) for p in sys.argv[1:5])
    if os.path.exists(scratch):
        shutil.rmtree(scratch)
    os.makedirs(scratch)
    walker = os.path.join(osm, "Models/WalkerModel/WalkerModel.osim")
    geometry = os.path.join(osm, "Geometry")
    out = os.path.join(scratch, "new")
    r = subprocess.run([sys.executable, "-c", RUNNER, walker, geometry, out, src], capture_output=True, text=True)
    xml = os.path.join(out, "WalkerModel_cvt1.xml")
    check(r.returncode == 0 and os.path.isfile(xml), "WalkerModel converts")
    r_old = subprocess.run([sys.executable, "-c", RUNNER, walker, geometry, os.path.join(scratch, "old"), old],
                           capture_output=True, text=True)
    check(r_old.returncode != 0, "the converter before Station 6 can't convert it ("
          + ((r_old.stdout + r_old.stderr).strip().splitlines() or ["?"])[-1][:80] + ")")
    if r.returncode != 0:
        print(r.stderr[-2000:])
        return 1

    g = contact_geometry.evaluate(walker, xml)
    check(g["L6"] == "PASS", f"L6 {g['L6']}: {len(g['pairs'])} pairs, max error "
          f"{max(v['max_abs_error_m'] for v in g['pairs'].values()):.1e} m, signs "
          f"{'ok' if all(v['sign_convention'] for v in g['pairs'].values()) else 'WRONG'}")

    cal_path = os.path.join(out, "contact_calibration.json")
    check(os.path.isfile(cal_path), "calibration report written")
    if os.path.isfile(cal_path):
        cal = json.load(open(cal_path))
        check(cal["drops_m"] == [0.02, 0.10], f"calibration used only the calibration drops {cal['drops_m']}")
        import mujoco
        m = mujoco.MjModel.from_xml_path(xml)
        for grp in cal["groups"].values():
            for name in grp["pairs"]:
                k = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_PAIR, name)
                ok = np.allclose(m.pair_solref[k], [grp["timeconst"], grp["dampratio"]]) and \
                    np.isclose(m.pair_friction[k, 0], grp["friction"])
                check(ok, f"pair {name} carries the calibrated parameters")

    sys.path.insert(0, src)
    from myoconverter.xml.contact_calibration import _opensim_drops, _full_mujoco, _stable
    if os.path.isfile(cal_path):
        pp = {p: (g["timeconst"], g["dampratio"], g["friction"]) for g in cal["groups"].values() for p in g["pairs"]}
        check(_stable(_full_mujoco(xml), pp), "the whole (unlocked) model is stable with the calibrated pairs "
              f"(change 6.12; {cal.get('rejected_unstable')} better-fitting candidate(s) were not)")
        check(cal.get("rejected_unstable", 0) >= 1, "the best fit to the drops alone would have thrown the free "
              "model into the air (why 6.12 exists)")
    rig = contact_dynamics.RigidOpenSim(walker)
    times, ref, q, v = rig.simulate(contact_dynamics.SCENARIOS["calibration"]["cal_h0.02"])
    pairs = [(a.getName(), b.getName()) for a, b in rig.pairs]
    conv = _opensim_drops(walker, pairs)
    cols = [conv["coordinates"].index(n) for n in rig.coords]
    diff = float(np.max(np.abs(conv["runs"][0][1][:, cols] - ref)))
    check(diff < 1e-9, f"calibration and harness simulate the 0.02 m drop alike (max difference {diff:.1e})")
    print("ALL CHECKS PASS" if fails == 0 else f"{fails} CHECK(S) FAILED")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
