"""Station 6, changes 6.7-6.9, 6.11 and 6.13: what stopped OpenSim's own models in the gated runs 1 and 2.

- 6.7 meshes: SoccerKickingModel has ASCII STL meshes (MuJoCo reads binary STL only). It converts; each ASCII STL is
  written as binary STL with the same triangles (vertex for vertex). An OBJ mesh (its soccer ball) is copied as OBJ and
  MuJoCo loads it.
- 6.8 a half-space on a body that can move: ToyLandingModel's platform half-space (on a body whose joint is locked)
  becomes a box (MuJoCo allows planes only on static bodies). It converts, and gate L6's check passes on every pair
  (the box's face is the half-space's surface: same signed distance as OpenSim's within 1e-6 m).
- 6.9 coupled coordinates in the contact calibration's rigid model: DynamicJumperModel (knee couplers) converts and its
  calibration ran (OpenSim couldn't assemble the rigid model when coupled coordinates were locked against their
  coupler); the L7 harness's rigid OpenSim model assembles too, and both simulate the 0.02 m drop alike (1e-9).
- 6.11 (found by Station 6 run2): Leg39 names fibula_R.vtp, the file is fibula_r.vtp (OpenSim finds it on
  case-insensitive file systems); it converts, reading that file.
- 6.13 (found by Station 6 run4: ToyLandingModel's calibration failed in 1 of 12 conversions): the calibration's
  OpenSim drops run in a fresh process. Checked with OpenSim blocked in the calling process: the drops still run, and
  equal the same drops simulated in-process bit for bit (the converter before needs OpenSim in the calling process);
  a drop that can't run (a missing file) is tried twice and raises RuntimeError. And ToyLandingModel's conversion
  calibrates its contacts.
The converter before these changes fails each of these files (old_src_dir: Station 6 run1's converter shows the errors
each change fixes; an older one, e.g. 49140ff's, fails earlier).

Usage: python station6/tests/test_more_files.py <src_dir> <old_src_dir> <opensim_models_dir> <scratch_dir>
"""
import importlib.util
import json
import os
import shutil
import subprocess
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, REPO)
from lowlimb_converter.checks import contact_dynamics, contact_geometry   # noqa: E402

# station6/coverage.py (its geometry folders and conversion runner), loaded by path (a "coverage" package may exist)
_spec = importlib.util.spec_from_file_location("station6_coverage", os.path.join(REPO, "station6", "coverage.py"))
cov = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(cov)

fails = 0


def check(ok, msg):
    global fails
    print(("  ok   " if ok else "  FAIL ") + msg)
    fails += 0 if ok else 1


def convert(src, osm, rel, scratch, label):
    osim = os.path.join(osm, rel)
    geometry = cov.geometry_folder(osm, osim, os.path.join(scratch, "geometry_work"))
    out = os.path.join(scratch, label)
    r = subprocess.run([sys.executable, "-c", cov.RUNNER, osim, geometry, out, src], capture_output=True, text=True)
    xml = os.path.join(out, os.path.basename(rel)[:-5] + "_cvt1.xml")
    ok = r.returncode == 0 and os.path.isfile(xml)
    last = next((ln for ln in reversed((r.stdout + r.stderr).strip().splitlines()) if "Error" in ln), "")
    return (xml if ok else None), geometry, out, last


ISOLATED = r"""
import json, pickle, sys
src, path, block = sys.argv[1], sys.argv[2], sys.argv[3] == "1"
pairs = [tuple(p) for p in json.loads(sys.argv[4])]
sys.path.insert(0, src)
import numpy as np
import myoconverter.xml.config  # noqa: F401
from myoconverter.xml import contact_calibration as cc
attempts = []
cc.logger.add(lambda m: attempts.append(1) if "attempt" in m else None, level="WARNING")
if block:
    sys.modules["opensim"] = None             # OpenSim can't be imported in this process
try:
    r = cc._opensim_drops(path, pairs)
except BaseException as e:
    print(json.dumps({"error": type(e).__name__ + ": " + str(e).splitlines()[-1][:80], "attempts": len(attempts) + 1}))
    sys.exit(0)
if block:
    print(json.dumps({"runs": len(r["runs"])}))
    sys.exit(0)
ref = r
"""


def isolated_drops(src, path, pairs, block):
    """change 6.13's check, in a child process: _opensim_drops with OpenSim importable or blocked there. Unblocked, it
    also compares with the in-process simulation (_opensim_drops_here), bit for bit."""
    code = ISOLATED
    if not block:
        code += ("here = cc._opensim_drops_here(path, pairs)\n"
                 "same = all(np.array_equal(a[1], b[1]) and a[0] == b[0] for a, b in zip(ref['runs'], here['runs']))"
                 " and ref['pose'] == here['pose'] and ref['touch'] == here['touch']\n"
                 "print(json.dumps({'same': bool(same)}))\n")
    r = subprocess.run([sys.executable, "-c", code, src, path, "1" if block else "0", json.dumps(pairs)],
                       capture_output=True, text=True)
    lines = [ln for ln in r.stdout.splitlines() if ln.startswith("{")]
    if not lines:
        return {"error": (r.stderr.strip().splitlines() or ["no output"])[-1][:120]}
    return json.loads(lines[-1])


def ascii_triangles(path):
    v = [[float(x) for x in ln.split()[1:]] for ln in open(path, errors="replace") if ln.split()[:1] == ["vertex"]]
    return np.array(v).reshape(-1, 3, 3)


def binary_triangles(path):
    data = open(path, "rb").read()
    n = int(np.frombuffer(data[80:84], dtype="<u4")[0])
    rec = np.frombuffer(data[84:84 + 50 * n], dtype=[("n", "<f4", 3), ("v", "<f4", (3, 3)), ("a", "<u2")])
    return rec["v"].astype(np.float64)


def main():
    src, old, osm, scratch = (os.path.abspath(p) for p in sys.argv[1:5])
    if os.path.exists(scratch):
        shutil.rmtree(scratch)
    os.makedirs(scratch)
    sys.path.insert(0, src)
    import myoconverter.xml.config  # noqa: F401  (the converter's modules are imported through config)
    from myoconverter.xml.bodies.utils import is_ascii_stl
    import mujoco

    print("6.7 ASCII STL and OBJ meshes (SoccerKickingModel)")
    rel = "Models/SoccerKick/SoccerKickingModel.osim"
    xml, geometry, out, _ = convert(src, osm, rel, scratch, "soccer")
    check(xml is not None, "it converts")
    _, _, _, err = convert(old, osm, rel, scratch, "soccer_old")
    check(bool(err), f"the converter before fails: {err[:100]}")
    if xml:
        m = mujoco.MjModel.from_xml_path(xml)    # loads every mesh (ASCII ones rewritten, OBJ ones copied)
        check(m.nmesh > 0, f"the converted model loads with its {m.nmesh} meshes")
        for f in sorted(os.listdir(geometry)):
            path = os.path.join(geometry, f)
            if f.lower().endswith(".stl") and is_ascii_stl(path):
                conv = os.path.join(out, "Geometry", f[:-4] + ".stl")
                if not os.path.isfile(conv):
                    continue                     # not used by the model
                a, b = ascii_triangles(path), binary_triangles(conv)
                check(a.shape == b.shape and np.allclose(a, b, rtol=0, atol=1e-6 * max(1.0, np.abs(a).max())),
                      f"{f}: ASCII -> binary STL, {len(a)} triangles, same vertices (float32)")
    # OBJ: SoccerKick's OBJ meshes hang on offset frames inside bodies, which the converter doesn't read (visual
    # only), so the copy is checked directly: copy_mesh_file on its soccer ball, and MuJoCo loads the result
    from myoconverter.xml.bodies.utils import copy_mesh_file
    objdir = os.path.join(scratch, "obj_check")
    os.makedirs(objdir)
    name, fname = copy_mesh_file("soccer_ball.obj", os.path.join(osm, "Models", "SoccerKick", "Geometry"), objdir)
    spec = mujoco.MjSpec()
    spec.add_mesh(name="ball", file=os.path.join(objdir, fname))
    spec.worldbody.add_body().add_geom(type=mujoco.mjtGeom.mjGEOM_MESH, meshname="ball")
    faces = int(spec.compile().mesh_facenum[0])
    check(fname == "soccer_ball.obj" and faces > 0, f"an OBJ mesh is copied as OBJ and MuJoCo loads it ({faces} faces)")

    print("6.8 a half-space on a body that can move (ToyLandingModel)")
    rel = "Models/ToyLanding/ToyLandingModel.osim"
    xml, _, _, _ = convert(src, osm, rel, scratch, "toy")
    check(xml is not None, "it converts")
    _, _, _, err = convert(old, osm, rel, scratch, "toy_old")
    check(bool(err), f"the converter before fails: {err[:100]}")
    if xml:
        m = mujoco.MjModel.from_xml_path(xml)
        g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "platform_surface")
        check(g >= 0 and m.geom_type[g] == mujoco.mjtGeom.mjGEOM_BOX, "platform_surface is a box")
        cal = json.load(open(os.path.join(scratch, "toy", "contact_calibration.json")))
        check("failed" not in cal and "groups" in cal, "its contacts are calibrated (6.13)")
        r = contact_geometry.evaluate(os.path.join(osm, rel), xml)
        check(r["L6"] == "PASS", f"L6 {r['L6']}: {len(r['pairs'])} pairs, max error "
              f"{max(v['max_abs_error_m'] for v in r['pairs'].values()):.1e} m")

    print("6.9 coupled coordinates in the rigid drop (DynamicJumperModel)")
    rel = "Models/Jumper/DynamicJumperModel.osim"
    xml, _, out, _ = convert(src, osm, rel, scratch, "jumper")
    check(xml is not None, "it converts")
    _, _, _, err = convert(old, osm, rel, scratch, "jumper_old")
    check(bool(err), f"the converter before fails: {err[:100]}")
    if xml:
        cal = json.load(open(os.path.join(out, "contact_calibration.json")))
        check("failed" not in cal and "groups" in cal,
              f"calibration ran: root position RMS {cal.get('root_position_rms_mm', float('nan')):.2f} mm")
        from myoconverter.xml.contact_calibration import _opensim_drops
        path = os.path.join(osm, rel)
        rig = contact_dynamics.RigidOpenSim(path)
        times, ref, q, v = rig.simulate(contact_dynamics.SCENARIOS["calibration"]["cal_h0.02"])
        conv = _opensim_drops(path, [(a.getName(), b.getName()) for a, b in rig.pairs])
        cols = [conv["coordinates"].index(n) for n in rig.coords]
        diff = float(np.max(np.abs(conv["runs"][0][1][:, cols] - ref)))
        check(diff < 1e-9, f"calibration and harness simulate the 0.02 m drop alike (max difference {diff:.1e})")
    print("6.13 the calibration's OpenSim drops in a fresh process (ToyLandingModel)")
    path = os.path.join(osm, "Models/ToyLanding/ToyLandingModel.osim")
    rig = contact_dynamics.RigidOpenSim(path)
    pairs = [(a.getName(), b.getName()) for a, b in rig.pairs]
    del rig
    r = isolated_drops(src, path, pairs, block=False)
    check(r.get("same") is True, f"the drops equal the same drops simulated in-process, bit for bit: {r}")
    r = isolated_drops(src, path, pairs, block=True)
    check(r.get("runs") == 2, f"they run with OpenSim blocked in the calling process: {r}")
    r = isolated_drops(old, path, pairs, block=True)
    check("error" in r, f"the converter before fails with OpenSim blocked in the calling process: {r}")
    r = isolated_drops(src, os.path.join(scratch, "missing.osim"), pairs, block=True)
    check("RuntimeError" in r.get("error", "") and r.get("attempts") == 2,
          f"a drop that can't run is tried twice and raises RuntimeError: {r}")

    print("6.11 a mesh file named with a different case (Leg39)")
    rel = "Models/Leg39/leg39.osim"
    xml, _, out, _ = convert(src, osm, rel, scratch, "leg39")
    check(xml is not None, "it converts")
    _, _, _, err = convert(old, osm, rel, scratch, "leg39_old")
    check(bool(err), f"the converter before fails: {err[:100]}")
    if xml:
        check(os.path.isfile(os.path.join(out, "Geometry", "fibula_R.stl")),
              "fibula_R.vtp (the file is fibula_r.vtp) was read and converted")

    print("ALL CHECKS PASS" if fails == 0 else f"{fails} CHECK(S) FAILED")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
