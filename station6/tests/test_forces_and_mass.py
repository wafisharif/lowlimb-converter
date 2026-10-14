"""Station 6, changes 6.3-6.5 on small cases: mass properties, CoordinateLimitForce, PathActuator and PathSpring.

- 6.3: gait10dof18musc's XML step: every OpenSim body's mass, centre of mass and inertia equal (gate L3's check);
  every body the converter adds (path-point and helper bodies) has the same mass properties as with the template's
  compiler bounds (compiled with and without the change, compared); the converter before 6.3 fails L3 at the toes.
- 6.4: onset_springs() reproduces OpenSim's limit force exactly beyond the transition; the ankle-spring variant of the
  gait10 model (opensim-models: subject01_metabolics_spring.osim) passes gate L4's check; before 6.4 the CLF is
  skipped and the check fails.
- 6.5: the exoskeleton variants (subject01_metabolics_path_actuator / _path_spring) convert and pass gate L5's check;
  before 6.5 they fail to convert.

Usage: python station6/tests/test_forces_and_mass.py <src_dir> <old_src_dir> <upstream_dir> <opensim_models_dir>
         <scratch_dir>
  old_src_dir: the converter before Station 6 (e.g. git archive 49140ff src), to show each check fails there
"""
import os
import shutil
import subprocess
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(os.path.dirname(HERE))
sys.path.insert(0, REPO)
from osim2mujoco.checks import limit_forces, mass_properties, path_forces   # noqa: E402

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
    r = subprocess.run([sys.executable, "-c", RUNNER, osim, geometry, out, src], capture_output=True, text=True)
    xml = os.path.join(out, os.path.splitext(os.path.basename(osim))[0] + "_cvt1.xml")
    return xml if r.returncode == 0 and os.path.isfile(xml) else None


def compiled(xml, bounds=None):
    spec = mujoco.MjSpec.from_file(xml)
    if bounds is not None:
        spec.compiler.boundmass, spec.compiler.boundinertia = bounds, bounds
    return spec.compile()


def main():
    src, old, up, osm, scratch = (os.path.abspath(p) for p in sys.argv[1:6])
    if os.path.exists(scratch):
        shutil.rmtree(scratch)
    os.makedirs(scratch)
    g10 = os.path.join(up, "models/osim/Gait10dof18musc/gait10dof18musc.osim")
    g10_geom = os.path.join(up, "models/osim/Gait10dof18musc/Geometry")
    shared_geom = os.path.join(osm, "Geometry")

    print("6.3 mass properties")
    new_xml, old_xml = convert(src, g10, g10_geom, os.path.join(scratch, "g10_new")), \
        convert(old, g10, g10_geom, os.path.join(scratch, "g10_old"))
    r_new, r_old = mass_properties.evaluate(g10, new_xml), mass_properties.evaluate(g10, old_xml)
    check(r_new["L3"] == "PASS", f"L3 {r_new['L3']} with the change ({len(r_new['bodies'])} bodies)")
    check(r_old["L3"] == "FAIL" and set(r_old["failing"]) == {"toes_l", "toes_r"},
          f"L3 {r_old['L3']} before it, failing {r_old['failing']}")
    a, b = compiled(new_xml), compiled(old_xml)
    added = [i for i in range(a.nbody) if mujoco.mj_id2name(a, mujoco.mjtObj.mjOBJ_BODY, i) not in r_new["bodies"]
             and i > 0 and mujoco.mj_id2name(a, mujoco.mjtObj.mjOBJ_BODY, i) != "ground"]
    same = all(np.isclose(a.body_mass[i], b.body_mass[i], rtol=1e-12, atol=0) and
               np.allclose(a.body_inertia[i], b.body_inertia[i], rtol=1e-12, atol=0) for i in added)
    check(len(added) > 0 and same, f"the {len(added)} bodies the converter adds keep their mass properties")

    print("6.4 CoordinateLimitForce")
    sys.path.insert(0, src)
    import myoconverter.xml.config  # noqa: F401  (the converter's modules are imported through config, as it does)
    from myoconverter.xml.forces.CoordinateLimitForce import onset_springs, smooth_step
    k_lim, w = 10.0 * 180 / np.pi, np.radians(2)
    springs = onset_springs(k_lim, w)
    x = np.linspace(w, 5 * w, 101)
    approx = sum(k * np.maximum(x - h, 0) for k, h in springs)
    check(np.allclose(approx, k_lim * x, rtol=1e-12, atol=1e-9), "dead-band springs exact beyond the transition")
    xi = np.linspace(0, w, 1001)
    err = np.max(np.abs(sum(k * np.maximum(xi - h, 0) for k, h in springs) - k_lim * xi * smooth_step(xi / w)))
    t = np.linspace(0, 1, 100001)
    g2 = np.max(np.abs(120 * t ** 2 - 300 * t ** 3 + 180 * t ** 4))   # (t S(t))'' (linear interpolation's error bound)
    bound = g2 / 8 * k_lim * w / 8 ** 2
    check(err <= bound, f"inside the transition within max|g''|/8 k w / n^2 = {bound:.3g} N m ({err:.3g} N m)")
    spring = os.path.join(osm, "Models/Gait10dof18musc/subject01_metabolics_spring.osim")
    for label, s in (("with the change", src), ("before it", old)):
        xml = convert(s, spring, shared_geom, os.path.join(scratch, f"spring_{label[:4]}"))
        r = limit_forces.evaluate(spring, xml) if xml else {"L4": "no conversion"}
        f = r.get("forces", {}).get("AnkleLimitSpringDamper", {})
        want = "PASS" if s == src else "FAIL"
        check(r["L4"] == want, f"L4 {r['L4']} {label}"
              + (f" (max error {f['max_error_pct_peak']:.3f}% of peak)" if f else ""))

    print("6.5 PathActuator, PathSpring")
    for v in ("subject01_metabolics_path_actuator", "subject01_metabolics_path_spring"):
        path = os.path.join(osm, f"Models/Gait10dof18musc/{v}.osim")
        xml = convert(src, path, shared_geom, os.path.join(scratch, v))
        r = path_forces.evaluate(path, xml) if xml else {"L5": "no conversion", "forces": {}}
        detail = "; ".join(f"{n}: length RMS {x['length_rms_mm']:.3f} mm, worst force "
                           f"{max(y['rms_pct_peak'] for y in x['generalized_force'].values()):.2f}%"
                           for n, x in r["forces"].items())
        check(r["L5"] == "PASS", f"{v}: L5 {r['L5']} ({detail})")
        check(convert(old, path, shared_geom, os.path.join(scratch, v + "_old")) is None,
              f"{v}: the converter before 6.5 can't convert it")
    print("ALL CHECKS PASS" if fails == 0 else f"{fails} CHECK(S) FAILED")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
