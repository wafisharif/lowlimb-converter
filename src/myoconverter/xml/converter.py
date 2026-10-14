# Modified by lowlimb-converter (2026-10-12), from MyoConverter @ cadf380: bodies get their mass properties exactly as
# in OpenSim; the template's compiler bounds (boundmass, boundinertia 0.001) no longer inflate small bodies (Station 6,
# change 6.3). Contact pairs are calibrated against OpenSim drops of the model (change 6.6). Joints whose socket names
# a body (or an offset frame on one) directly are converted too (change 6.1).
# Modified by lowlimb-converter (2026-10-13): the path-point fits' cached OpenSim model is released before the contact
# calibration, and a calibration that OpenSim can't simulate leaves the pairs uncalibrated with an error logged
# (change 6.9).
# Modified by lowlimb-converter (2026-10-13): joints driven by a coupling get no damping (change 6.10).
""" This module contains the OpenSim-to-MuJoCo model (XML file) convertion main access points.

You can convert an OpenSim XML model file to MuJoCo by calling the :py:func:`convert` function, or by running this
module.

Example:

  in a command shell::

    python converter.py /path/to/opensim/model.xml /output/directory/for/mujoco/model/


  or, in a Python file::

    from myoconverter.xml.converter import convert
    convert(xml_file, output_folder, **kwargs)

"""

import argparse
from loguru import logger
from lxml import etree

from myoconverter.xml import config as cfg
from myoconverter.xml.utils import find_element_by_name, split_name, create_keyframe, vec2str


def convert(xml_file, output_folder, **kwargs):
  """ Convert an OpenSim XML model file to MuJoCo XMl model file.

  :param xml_file: Path to the OpenSim XML model file
  :param output_folder: Path to folder where converted model is saved
  :param kwargs: geometry_folder (str), add_ground_geom (bool), treat_as_normal_path_point (bool)
  :return: Path to the MuJoCo XML file
  """

  # First, initialise config (read and parse OpenSim and MuJoCo XML files, set variables etc.)
  cfg.initialise(xml_file, output_folder, **kwargs)

  logger.info(f"Commencing the conversion procedure!")

  # Parse ground. The ground may have attached geometries and wrapping objects
  _parse_ground()

  # Parse constraints. The constraints need to be parsed before joints, because we might need to update the polycoefs
  # of some joint constraints when parsing joints
  _parse_constraints()

  # Parse bodies and joints
  _parse_bodies_and_joints()

  # Parse forces
  _parse_forces()

  # Parse markers -- needed for later optimization procedures
  _parse_markers()

  # Set keyframe
  _set_keyframe()

  # lowlimb-converter (change 6.10): no damping on joints a coupling drives
  _undamped_coupled_joints()

  # Copy credits from the OpenSim model
  _copy_credits()

  # Finally, save the MuJoCo model into XML file
  logger.info(f"Writing the converted model into {cfg.OUTPUT_XML_FILE}")
  etree.ElementTree(cfg.MUJOCO).write(cfg.OUTPUT_XML_FILE, pretty_print=True)

  # lowlimb-converter (change 6.3): exact mass properties, then write again
  _explicit_inertials()
  etree.ElementTree(cfg.MUJOCO).write(cfg.OUTPUT_XML_FILE, pretty_print=True)

  # lowlimb-converter (change 6.6): fit the contact pairs' parameters to OpenSim drops of the model
  if cfg.CONTACT_GROUPS:
    _calibrate_contacts()
    etree.ElementTree(cfg.MUJOCO).write(cfg.OUTPUT_XML_FILE, pretty_print=True)

  logger.info("All good, conversion procedure is finished")

  # Return path to converted file
  return cfg.OUTPUT_XML_FILE

def _undamped_coupled_joints():
  """ lowlimb-converter (change 6.10): a joint that an active joint equality drives from another joint (a coupled
  coordinate, a CustomJoint's dependent axis, a patella) gets damping 0.

  The template gives every joint damping 0.05 N m s/rad (OpenSim has no joint damping). On a joint whose motion the
  coupling dictates, that damping only fights the coupling: once change 6.3 gave Rajagopal 2016's patellae their own
  small inertia (1.3e-5 kg m^2 instead of the template's 0.001), the patella lagged its knee by up to 1.9 deg in motion
  (station2/tests/test_chained_coupling.py; 0.07 deg without it). Independent joints keep the template's damping.
  """
  for eq in cfg.M_EQUALITY.findall("joint"):
    if eq.get("joint2") is None or eq.get("active", "true").strip().lower() == "false":
      continue
    joint = cfg.M_WORLDBODY.find(f".//joint[@name='{eq.get('joint1')}']")
    if joint is not None:
      joint.attrib["damping"] = "0"


def _calibrate_contacts():
  """ lowlimb-converter (change 6.6): calibrate every converted contact pair (xml/contact_calibration.py). """
  import json
  import os
  from myoconverter.xml.contact_calibration import calibrate
  from myoconverter.xml.path_points.utils import release_osim_model
  release_osim_model()      # change 6.9: the path-point fits' cached OpenSim model must not outlive them
  try:
    result = calibrate(cfg.OUTPUT_XML_FILE, cfg.OSIM_FILE, cfg.CONTACT_GROUPS, cfg.OUTPUT_FOLDER)
  except RuntimeError as e:
    # change 6.9: OpenSim couldn't simulate the rigid model; the pairs keep their starting parameters, and say so
    logger.error(f"Contact calibration failed, pairs keep MuJoCo's default solref and OpenSim's static friction: {e}")
    with open(os.path.join(cfg.OUTPUT_FOLDER, "contact_calibration.json"), "w") as f:
      json.dump({"failed": str(e)[:2000]}, f, indent=1)
    return
  if result is None:
    return
  for group in result.values():
    for name in group["pairs"]:
      pair = cfg.M_CONTACT.find(f"pair[@name='{name}']")
      pair.attrib["solref"] = f"{repr(group['timeconst'])} {repr(group['dampratio'])}"
      pair.attrib["friction"] = f"{repr(group['friction'])} {repr(group['friction'])} 0.005 0.0001 0.0001"


def _explicit_inertials():
  """ lowlimb-converter (change 6.3): every body's mass properties exactly as OpenSim's, others unchanged.

  The template's compiler sets boundmass = boundinertia = 0.001, so MuJoCo raised every body mass below 1 g and every
  principal inertia below 0.001 kg m^2 to those values, including OpenSim bodies whose own values are smaller (the
  gait models' toes: 0.0001-0.0002 kg m^2 became 0.001). The bounds exist for bodies the converter adds that have no
  OpenSim counterpart (moving/conditional path-point bodies, helper joints), and for OpenSim bodies without a usable
  inertia (MuJoCo infers theirs from geoms). Here the model just written is compiled once with the bounds; every body
  that has no explicit inertia in the XML (no <inertial>, or an <inertial> without fullinertia/diaginertia) gets the
  compiled values written out explicitly (same numbers, so those bodies are unchanged); then the bounds are removed,
  so bodies with an OpenSim inertia keep it exactly.
  """
  import mujoco
  compiler = cfg.MUJOCO.find("compiler")
  if compiler is None or ("boundmass" not in compiler.attrib and "boundinertia" not in compiler.attrib):
    return
  model = mujoco.MjModel.from_xml_path(cfg.OUTPUT_XML_FILE)
  changed = 0
  for body in cfg.M_WORLDBODY.iter("body"):
    inertial = body.find("inertial")
    if inertial is not None and ("fullinertia" in inertial.attrib or "diaginertia" in inertial.attrib):
      continue
    bid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body.attrib["name"])
    if bid < 0:
      logger.critical(f"Body {body.attrib['name']} not found in the compiled model")
      raise RuntimeError
    if inertial is None:
      inertial = etree.Element("inertial")
      body.insert(0, inertial)
    inertial.attrib.clear()
    inertial.attrib.update({"pos": vec2str(model.body_ipos[bid]), "quat": vec2str(model.body_iquat[bid]),
                            "mass": repr(float(model.body_mass[bid])), "diaginertia": vec2str(model.body_inertia[bid])})
    changed += 1
  del compiler.attrib["boundmass"]
  del compiler.attrib["boundinertia"]
  logger.info(f"Mass properties: {changed} bodies without an OpenSim inertia written out explicitly (as compiled "
              f"before), compiler bounds removed")


def _parse_bodies_and_joints():
  """ Parse OpenSim `BodySet` and `JointSet`.

  :return: None
  """

  # Start from ground and go through joints recursively (depth-first), and add bodies, geoms, joints to MuJoCo model
  logger.info("Starting to parse bodies, wrapping objects, and joints")
  _add_bodies_and_joints(f"/{cfg.O_GROUND.attrib['name']}", cfg.M_WORLDBODY, root_body=True)

def _socket_frame(joint, frames, name):
  """ lowlimb-converter (change 6.1): a joint's parent or child frame, by the name its socket gives.

  Usually a PhysicalOffsetFrame among the joint's own frames. OpenSim also accepts a path to a body or to ground
  ("/bodyset/rod1", "/ground"; the frame is the body's own), or to an offset frame among a body's components
  ("/bodyset/rod1/offset"); upstream found only the first kind and skipped or crashed on the others. Those are
  returned as an equivalent PhysicalOffsetFrame element (translation, orientation, socket_parent).
  """
  found = frames.find(f".//*[@name='{name}']") if frames is not None else None
  if found is not None or not name.startswith("/"):
    return found
  parts = [p for p in name.split("/") if p]
  translation, orientation, parent = "0 0 0", "0 0 0", name
  if len(parts) == 3 and parts[0] == "bodyset" or len(parts) == 2 and parts[0] == "ground":
    owner = cfg.O_GROUND if parts[0] == "ground" else \
      next((b for b in cfg.O_BODYSET if b.attrib.get("name") == parts[1]), None)
    frame = None if owner is None else owner.find(f"components/PhysicalOffsetFrame[@name='{parts[-1]}']")
    if frame is None:
      return None
    translation, orientation = frame.find("translation").text, frame.find("orientation").text
    parent = "/" + "/".join(parts[:-1])
  elif not (len(parts) == 2 and parts[0] == "bodyset" or parts == ["ground"]):
    return None
  element = etree.Element("PhysicalOffsetFrame", name=name)
  etree.SubElement(element, "socket_parent").text = parent
  etree.SubElement(element, "translation").text = translation
  etree.SubElement(element, "orientation").text = orientation
  return element


def _add_bodies_and_joints(parent_name, current_body, root_body=False):
  """ Add OpenSim `Body` and related `Joint`s to MuJoCo model in a recurrent fashion.

  :param parent_name: Name of parent `Body`
  :param current_body: Pointer to current body (XML element) in the MuJoCo XML file
  :param root_body: Boolean to indicate whether current body is the root of the kinematic chain or not
  :return: None
  """

  origin = parent_name   # lowlimb-converter: the loop below reassigns parent_name (upstream behaviour, kept)

  # Find ALL frames with 'parent_name' as socket parent. This will contain some incorrect frames as well,
  # we'll need to cut them by checking whether they are socket_parent_frames or socket_child_frames
  socket_parent = cfg.O_JOINTSET.xpath(f".//socket_parent[text()='{parent_name}']")

  for s in socket_parent:

    # Get parent until WeldJoint, CustomJoint, etc
    frames = s.getparent().getparent()
    joint = frames.getparent()

    # Get parent and child socket frames (can be PhysicalOffsetFrame, or PhysicalFrame? Let's use wildcard
    # in the find so we don't need to care about the tag)
    # lowlimb-converter (change 6.1): a socket may also name a body (or an offset frame on one) directly
    socket_parent_frame = _socket_frame(joint, frames, joint.find('socket_parent_frame').text)
    socket_child_frame = _socket_frame(joint, frames, joint.find('socket_child_frame').text)

    # Ignore child socket frames
    if socket_child_frame.find("socket_parent").text == parent_name:
      continue

    # Find child body of joint
    child_body = find_element_by_name(cfg.OPENSIM, split_name(socket_child_frame.find("socket_parent").text))

    # Parse body
    next_body = cfg.BODY_PARSER.parse(child_body,
                                      socket_parent_frame=socket_parent_frame,
                                      socket_child_frame=socket_child_frame,
                                      current_body=current_body,
                                      root_body=root_body)

    # Parse joint
    cfg.JOINT_PARSER.parse(joint,
                           socket_parent_frame=socket_parent_frame,
                           socket_child_frame=socket_child_frame,
                           pointer=next_body,
                           root_body=root_body)

    # Move to next joint
    parent_name = socket_child_frame.find('socket_parent').text
    _add_bodies_and_joints(parent_name, next_body)

  # lowlimb-converter (change 6.1): joints whose parent socket names this body (or an offset frame on it) directly,
  # without an offset frame of their own; upstream never visited them, so their child bodies were left out
  for joint in cfg.O_JOINTSET:
    if not isinstance(joint.tag, str) or joint.find("socket_parent_frame") is None:
      continue
    name = joint.find("socket_parent_frame").text
    frames = joint.find("frames")
    if (frames is not None and frames.find(f".//*[@name='{name}']") is not None) or not name.startswith("/"):
      continue
    socket_parent_frame = _socket_frame(joint, frames, name)
    if socket_parent_frame is None or socket_parent_frame.find("socket_parent").text != origin:
      continue
    socket_child_frame = _socket_frame(joint, frames, joint.find("socket_child_frame").text)
    child_body = find_element_by_name(cfg.OPENSIM, split_name(socket_child_frame.find("socket_parent").text))
    next_body = cfg.BODY_PARSER.parse(child_body, socket_parent_frame=socket_parent_frame,
                                      socket_child_frame=socket_child_frame, current_body=current_body,
                                      root_body=root_body)
    cfg.JOINT_PARSER.parse(joint, socket_parent_frame=socket_parent_frame, socket_child_frame=socket_child_frame,
                           pointer=next_body, root_body=root_body)
    _add_bodies_and_joints(socket_child_frame.find("socket_parent").text, next_body)

def _parse_ground():
  """ Parse OpenSim `Ground`.

  :return: None
  """

  logger.info("Parsing the ground")
  cfg.BODY_PARSER.parse(cfg.O_GROUND, add_ground_geom=cfg.ADD_GROUND_GEOM)

def _parse_constraints():
  """ Parse OpenSim `ConstraintSet`.

  :return: None
  """

  logger.info("Starting to parse constraints")
  cfg.CONSTRAINT_PARSER.parse_all(cfg.O_CONSTRAINTSET)

def _parse_forces():
  """ Parse OpenSim `ForceSet`.

  :return: None
  """

  logger.info("Starting to parse forces, including path points and wrap paths")
  cfg.FORCE_PARSER.parse_all(cfg.O_FORCESET)

def _parse_markers():
  """ Parse OpenSim `MarkerSet`.

  :return: None
  """

  logger.info("Starting to parse markers")
  cfg.MARKER_PARSER.parse_all(cfg.O_MARKERSET)

def _set_keyframe():
  """ Create a keyframe for the MuJoCo model.

  The keyframe defines a default pose, where all dependent joints have been properly initialised.

  :return: None
  """

  logger.info("Setting the default keyframe")
  create_keyframe(cfg.MUJOCO, cfg.M_WORLDBODY, cfg.M_EQUALITY)

def _copy_credits():
  """ Copy credits from the OpenSim model.

  :return: None
  """

  # Get credits
  credits = cfg.OPENSIM.find("credits")

  # Add as comment if they exist
  if credits is not None:
    comment = etree.Comment(f" Credits from the original OpenSim model. Note! If the following has any license information, it applies to the original OpenSim model. Credits: {credits.text} ")
    cfg.MUJOCO.insert(0, comment)

  # Add our credits as a comment too
  comment = etree.Comment(" This model has been converted from an OpenSim model. Model conversion by MyoConverter https://github.com/MyoHub/myoconverter. This model is licensed under Apache 2.0. ")
  cfg.MUJOCO.insert(0, comment)

if __name__ == "__main__":

  argparser = argparse.ArgumentParser(description='Convert an OpenSim model into a MuJoCo model.'
                                                    'Only Works with OpenSim v4 models.')
  """ ArgumentParser: Parse arguments when running module as a Python script. """
  argparser.add_argument('xml_file', type=str,
                         help='Path to an OpenSim model XML file')
  argparser.add_argument('output_folder', type=str,
                         help="Path to an output folder. The converted model will be saved here.")
  argparser.add_argument('--geometry_folder', type=str, default=None,
                         help='Path to the Geometry folder (by default uses folder of given OpenSim file)')
  argparser.add_argument('--add_ground_geom', default=False, action="store_true",
                         help="If true, a geom (of type plane) is added to the MuJoCo model as ground")
  argparser.add_argument('--treat_as_normal_path_point', default=False, action="store_true",
                         help="If true, MovingPathPoints and ConditionalPathPoints will be treated as normal "
                              "PathPoints")
  args = argparser.parse_args()

  # Do the conversion
  convert(**vars(args))
