"""Station 6, change 6.1: the converter reads every OpenSim 4 file OpenSim itself reads.

Builds variants of gait10dof18musc that OpenSim loads without complaint and checks what the converter does:
- "::" element names: a disabled HuntCrossleyForce (with an empty HuntCrossleyForce::ContactParametersSet, so it needs
  no contact geometry) added to the ForceSet; the file must convert, and give the same cvt1 as the unmodified model
  (the force doesn't apply);
- omitted coordinate properties: knee_angle_r without <range>, <clamped> and <locked>: OpenSim uses its class
  defaults (checked here against the installed OpenSim: range -10..10, not clamped, not locked), and the converted joint
  must get the same (unlimited, range -10..10, not locked);
- placeholder and upper-case meshes: one body's mesh_file set to "Unassigned" (no geom, logged "skipping"), another's
  mesh file copied to an upper-case .STL name (converted like .stl);
- a joint whose parent socket names the parent body directly (/bodyset/femur_r instead of an offset frame): the child
  body is converted, and Station 2's kinematic check passes for the whole model.
The converter before change 6.1 fails each conversion (pass an older src to see it).

Usage: python station6/tests/test_reading.py <src_dir> <gait10dof18musc.osim> <geometry_dir> <scratch_dir>
"""
import os
import shutil
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
    name = os.path.splitext(os.path.basename(osim))[0]
    xml = os.path.join(out, f"{name}_cvt1.xml")
    log = os.path.join(out, f"{name}_conversion.log")
    return (xml if r.returncode == 0 and os.path.isfile(xml) else None,
            (open(log).read() if os.path.isfile(log) else "") + r.stdout + r.stderr)


HC = """<HuntCrossleyForce name="unused_contact">
  <appliesForce>false</appliesForce>
  <HuntCrossleyForce::ContactParametersSet name="contact_parameters">
    <objects />
    <groups />
  </HuntCrossleyForce::ContactParametersSet>
  <transition_velocity>0.2</transition_velocity>
</HuntCrossleyForce>
"""


def main():
    src, osim_in, geometry, scratch = (os.path.abspath(p) for p in sys.argv[1:5])
    if os.path.exists(scratch):
        shutil.rmtree(scratch)
    os.makedirs(scratch)
    base_xml, _ = convert(src, osim_in, geometry, os.path.join(scratch, "base"))
    check(base_xml is not None, "the unmodified model converts")

    print("'::' element names")
    text = open(osim_in).read()
    i = text.rindex("</objects>", 0, text.index("</ForceSet>"))   # the ForceSet's own list
    variant = os.path.join(scratch, "gait10_nested.osim")
    with open(variant, "w") as f:
        f.write(text[:i] + HC + text[i:])
    import opensim
    om = opensim.Model(variant)
    om.initSystem()
    check(om.getForceSet().contains("unused_contact"), "OpenSim reads the file and builds the model")
    xml, log = convert(src, variant, geometry, os.path.join(scratch, "nested"))
    check(xml is not None, "it converts")
    if xml and base_xml:
        same = open(xml).read().replace("gait10_nested", "gait10dof18musc") == open(base_xml).read()
        check(same, "cvt1 equals the unmodified model's (the added force doesn't apply)")

    print("omitted coordinate properties")
    c = opensim.Coordinate()
    check(c.getRangeMin() == -10 and c.getRangeMax() == 10 and not c.getDefaultClamped() and not c.getDefaultLocked(),
          "OpenSim's Coordinate class defaults: range -10..10, not clamped, not locked (as the converter assumes)")
    sys.path.insert(0, src)
    try:
        from myoconverter.xml.joints.utils import COORDINATE_DEFAULTS
        check(COORDINATE_DEFAULTS == {"range": "-10 10", "clamped": "false", "locked": "false"},
              f"converter's defaults {COORDINATE_DEFAULTS}")
    except ImportError:
        check(False, "converter has COORDINATE_DEFAULTS")
    t = etree.parse(osim_in)
    coord = t.xpath("//Coordinate[@name='knee_angle_r']")[0]
    for tag in ("range", "clamped", "locked"):
        el = coord.find(tag)
        if el is not None:
            coord.remove(el)
    variant = os.path.join(scratch, "gait10_coord.osim")
    t.write(variant)
    om = opensim.Model(variant)
    oc = om.getCoordinateSet().get("knee_angle_r")
    check(oc.getRangeMin() == -10 and not oc.getDefaultClamped(), "OpenSim reads it with the class defaults")
    xml, log = convert(src, variant, geometry, os.path.join(scratch, "coord"))
    check(xml is not None, "it converts")
    if xml:
        m, _ = mc.load_without_meshes(xml)
        j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, "knee_angle_r")
        check(np.allclose(m.jnt_range[j], [-10, 10]) and not m.jnt_limited[j],
              f"knee_angle_r: range {m.jnt_range[j]}, limited {bool(m.jnt_limited[j])}")

    print("placeholder and upper-case meshes")
    t = etree.parse(osim_in)
    meshes = t.xpath("//BodySet//attached_geometry/Mesh")
    placeholder, upper = meshes[0], meshes[1]
    placeholder.find("mesh_file").text = "Unassigned"
    original = upper.find("mesh_file").text
    geo2 = os.path.join(scratch, "geometry_upper")
    shutil.copytree(geometry, geo2)
    upper_name = os.path.splitext(original)[0] + "_copy.STL"
    if original.lower().endswith(".vtp"):
        import pyvista
        pyvista.read(os.path.join(geometry, original)).extract_surface().triangulate().save(
            os.path.join(geo2, upper_name))
    else:
        shutil.copyfile(os.path.join(geometry, original), os.path.join(geo2, upper_name))
    upper.find("mesh_file").text = upper_name
    variant = os.path.join(scratch, "gait10_meshes.osim")
    t.write(variant)
    xml, log = convert(src, variant, geo2, os.path.join(scratch, "meshes"))
    check(xml is not None, "it converts")
    if xml:
        x = etree.parse(xml).getroot()
        names = {g.get("name") for g in x.iter("geom")}
        check(placeholder.get("name") not in names and f"skipping {placeholder.get('name')}" in log,
              f"'Unassigned' mesh {placeholder.get('name')}: no geom, logged as skipped")
        check(upper.get("name") in names, f"upper-case .STL mesh {upper.get('name')}: converted")

    print("a joint whose parent socket names the body directly")
    t = etree.parse(osim_in)
    knee = t.xpath("//JointSet/objects/*[@name='knee_r']")[0]
    pf_name = knee.findtext("socket_parent_frame")
    pf = knee.find(f"frames/*[@name='{pf_name}']")
    zero = all(float(v) == 0 for v in (pf.findtext("translation") + " " + pf.findtext("orientation")).split())
    knee.find("socket_parent_frame").text = pf.findtext("socket_parent")
    knee.find("frames").remove(pf)
    variant = os.path.join(scratch, "gait10_direct.osim")
    t.write(variant)
    om = opensim.Model(variant)
    om.initSystem()
    check(om.getJointSet().contains("knee_r"), f"OpenSim reads it (knee_r's parent frame had zero offset: {zero})")
    xml, log = convert(src, variant, geometry, os.path.join(scratch, "direct"))
    check(xml is not None, "it converts")
    if xml:
        m, _ = mc.load_without_meshes(xml)
        check(all(mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, b) >= 0 for b in ("tibia_r", "calcn_r", "toes_r")),
              "tibia_r and the bodies below it are converted")
        res = mc.compare(xml, orf.reference(variant), keep_arrays=False)
        check(res["K0"] == "PASS" and res["K1"] == "PASS" and res["K2"] == "PASS",
              f"Station 2: K0 {res['K0']} K1 {res.get('K1')} K2 {res.get('K2')}"
              + (f", worst {res['pos_worst_mm']['value']:.4f} mm" if res["K0"] == "PASS" else ""))

    print("ALL CHECKS PASS" if fails == 0 else f"{fails} CHECK(S) FAILED")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
