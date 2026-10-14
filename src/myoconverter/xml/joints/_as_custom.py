""" lowlimb-converter (Station 6, change 6.2): joints that are special cases of OpenSim's CustomJoint.

OpenSim's PlanarJoint, FreeJoint, BallJoint and GimbalJoint are each a CustomJoint with a fixed SpatialTransform:
rotations about fixed axes in sequence (rotation1, rotation2, rotation3; body-fixed, as in every CustomJoint), then
translations along the parent frame's axes. They are converted by building that CustomJoint and converting it with the
CustomJoint parser, so they get exactly the same treatment as the CustomJoints already checked at Station 2.
station6/tests/test_joint_types.py builds one model per joint type with OpenSim's API and checks every body against
OpenSim in Station 2's poses.

(xml/parsers.py JointParser lists this file in ignore_files, so it is not taken for a parser.)
"""
from copy import deepcopy

from lxml import etree

from loguru import logger

from myoconverter.xml.joints.Joint import Joint

# coordinate index (in the joint's <coordinates>, as OpenSim orders them) driving each transform axis, or None for a
# constant zero; axes as in OpenSim's joint classes (Simbody mobilizers)
X, Y, Z = "1 0 0", "0 1 0", "0 0 1"
SPECS = {
  # PlanarJoint: coordinates (rz, tx, ty): rotation about Z, translation in the X-Y plane
  "PlanarJoint": [("rotation1", Z, 0), ("rotation2", X, None), ("rotation3", Y, None),
                  ("translation1", X, 1), ("translation2", Y, 2), ("translation3", Z, None)],
  # FreeJoint: coordinates (rx, ry, rz, tx, ty, tz): body-fixed X-Y-Z rotations, translations in the parent frame
  "FreeJoint": [("rotation1", X, 0), ("rotation2", Y, 1), ("rotation3", Z, 2),
                ("translation1", X, 3), ("translation2", Y, 4), ("translation3", Z, 5)],
  # BallJoint, GimbalJoint: coordinates (rx, ry, rz): body-fixed X-Y-Z rotations
  "BallJoint": [("rotation1", X, 0), ("rotation2", Y, 1), ("rotation3", Z, 2),
                ("translation1", X, None), ("translation2", Y, None), ("translation3", Z, None)],
  "GimbalJoint": [("rotation1", X, 0), ("rotation2", Y, 1), ("rotation3", Z, 2),
                  ("translation1", X, None), ("translation2", Y, None), ("translation3", Z, None)],
}


def as_custom_joint(xml):
  """ The CustomJoint equivalent of an OpenSim PlanarJoint / FreeJoint / BallJoint / GimbalJoint XML element. """
  spec = SPECS[xml.tag]
  coordinates = [c for c in (xml.find("coordinates") if xml.find("coordinates") is not None else [])
                 if isinstance(c.tag, str)]
  n = 1 + max(k for _, _, k in spec if k is not None)
  if len(coordinates) != n:
    logger.critical(f"{xml.tag} {xml.attrib.get('name')} has {len(coordinates)} coordinates, expected {n}")
    raise RuntimeError
  custom = deepcopy(xml)
  custom.tag = "CustomJoint"
  transform = etree.SubElement(custom, "SpatialTransform")
  for name, axis, k in spec:
    ta = etree.SubElement(transform, "TransformAxis", name=name)
    etree.SubElement(ta, "coordinates").text = coordinates[k].attrib["name"] if k is not None else None
    etree.SubElement(ta, "axis").text = axis
    if k is None:
      etree.SubElement(etree.SubElement(ta, "Constant", name="function"), "value").text = "0"
    else:
      etree.SubElement(etree.SubElement(ta, "LinearFunction", name="function"), "coefficients").text = "1 0"
  return custom


class CustomJointEquivalent(Joint):
  """ Base class: convert the joint as the CustomJoint it is. """

  def _parse(self, xml, socket_parent_frame, socket_child_frame, pointer):
    from myoconverter.xml.joints.CustomJoint import CustomJoint
    logger.info(f"{xml.tag} {xml.attrib.get('name')}: converted as the equivalent CustomJoint (change 6.2)")
    return CustomJoint()._parse(as_custom_joint(xml), socket_parent_frame, socket_child_frame, pointer)
