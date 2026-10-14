# Added by lowlimb-converter (2026-10-12): not in MyoConverter @ cadf380 (Station 6, change 6.6).
# Modified by lowlimb-converter (2026-10-13): a half-space on a body that can move (MuJoCo allows planes only on static
# bodies) becomes a large box whose face is the half-space's surface (Station 6, change 6.8).
""" Contains the `HuntCrossleyForce` parser (lowlimb-converter, change 6.6).

OpenSim's HuntCrossleyForce acts between contact geometries (the model's ContactGeometrySet) listed in its
ContactParameters: every listed pair of geometries on different bodies can touch. MuJoCo: each ContactSphere becomes a
sphere geom and each ContactHalfSpace a plane geom, on the same body, at the same place (the geom frame of a half-space
is turned so MuJoCo's plane normal, +z, is OpenSim's outward normal, -x: OpenSim's half-space is the region x > 0 of its
frame; on a body that can move, where MuJoCo allows no plane, a large box with that face, change 6.8), and every pair
becomes an explicit MuJoCo contact <pair> (the template's geoms never collide on their own).
Station 6 gate L6 checks that the signed distance of every pair matches OpenSim's geometry.

The contact force law differs: Hunt-Crossley is a nonlinear spring-damper (force ~ penetration^1.5) with Stribeck
friction; MuJoCo's soft contact is a linear, mass-normalised spring-damper with Coulomb friction. The pair parameters
(solref, friction) start from MuJoCo's defaults and the static friction; xml/contact_calibration.py then fits them to
OpenSim simulations of the model (gate L7).
"""

import numpy as np
from lxml import etree
from scipy.spatial.transform import Rotation

from loguru import logger

from myoconverter.xml.parsers import IParser
from myoconverter.xml.utils import str2bool, str2vec, vec2str, element_txt2num
from myoconverter.xml import config as cfg

SUPPORTED_GEOMETRY = ("ContactSphere", "ContactHalfSpace")
GEOM_GROUP = "3"                         # not used for mass (template: inertiagrouprange 0 1)
DEFAULT_SOLREF = (0.02, 1.0)             # MuJoCo's default, replaced by the calibration (change 6.6)
# MuJoCo plane: normal +z; OpenSim half-space: solid where x > 0, so its outward normal is -x
HALFSPACE_TO_PLANE = Rotation.from_euler("y", -90, degrees=True)
# change 6.8: a half-space on a moving body is a box this big (half sizes, m) with its +z face on the surface
HALFSPACE_BOX = (100.0, 100.0, 50.0)


def is_static(body):
  """ change 6.8: True for MuJoCo's world body and bodies welded to it (no joint on the way up). """
  while body is not None and body.tag == "body":
    if body.find("joint") is not None or body.find("freejoint") is not None:
      return False
    body = body.getparent()
  return True


def contact_geometries():
  """ OpenSim ContactGeometrySet objects by name. """
  objects = cfg.OPENSIM.find("ContactGeometrySet/objects")
  return {} if objects is None else {g.attrib["name"]: g for g in objects if isinstance(g.tag, str)}


def _frame_pose(socket):
  """ (MuJoCo body element, position, Rotation) of an OpenSim frame path: /ground, /bodyset/<body>, optionally
  followed by the name of a PhysicalOffsetFrame among that body's (or ground's) components. """
  parts = [p for p in socket.strip().split("/") if p]
  if parts and parts[0] == "ground":
    body, osim_owner, rest = cfg.M_GROUND, cfg.O_GROUND, parts[1:]
  elif len(parts) >= 2 and parts[0] == "bodyset":
    body = cfg.M_WORLDBODY.find(f".//body[@name='{parts[1]}']")
    osim_owner = next((b for b in cfg.O_BODYSET if b.attrib.get("name") == parts[1]), None)
    rest = parts[2:]
  else:
    body, osim_owner, rest = None, None, parts
  if body is None or osim_owner is None or len(rest) > 1:
    logger.critical(f"Contact geometry frame {socket} is not a body, ground, or an offset frame on one")
    raise NotImplementedError
  pos, rot = np.zeros(3), Rotation.identity()
  if rest:
    frame = osim_owner.find(f"components/PhysicalOffsetFrame[@name='{rest[0]}']")
    if frame is None:
      logger.critical(f"Contact geometry frame {socket}: offset frame {rest[0]} not found")
      raise NotImplementedError
    pos = str2vec(frame.find("translation").text)
    rot = Rotation.from_euler("XYZ", str2vec(frame.find("orientation").text))
  return body, pos, rot


def add_contact_geom(name, xml):
  """ Create the MuJoCo geom for OpenSim contact geometry `xml` (once); return its name. """
  if cfg.M_WORLDBODY.find(f".//geom[@name='{name}']") is not None:
    return name
  if xml.tag not in SUPPORTED_GEOMETRY:
    logger.critical(f"Contact geometry {name}: {xml.tag} is not supported (only {SUPPORTED_GEOMETRY})")
    raise NotImplementedError
  body, frame_pos, frame_rot = _frame_pose(xml.find("socket_frame").text)
  location = str2vec(xml.find("location").text) if xml.find("location") is not None else np.zeros(3)
  orientation = str2vec(xml.find("orientation").text) if xml.find("orientation") is not None else np.zeros(3)
  pos = frame_pos + frame_rot.apply(location)
  rot = frame_rot * Rotation.from_euler("XYZ", orientation)
  attrib = {"name": name, "pos": vec2str(pos), "group": GEOM_GROUP, "contype": "0", "conaffinity": "0"}
  if xml.tag == "ContactSphere":
    attrib.update({"type": "sphere", "size": repr(float(xml.find("radius").text)), "rgba": "0 1 1 0.5",
                   "quat": vec2str(np.roll(rot.as_quat(), 1))})
  elif is_static(body):
    attrib.update({"type": "plane", "size": "0 0 0.01", "rgba": "0.6 0.6 0.6 0.3",
                   "quat": vec2str(np.roll((rot * HALFSPACE_TO_PLANE).as_quat(), 1))})
  else:
    # change 6.8: MuJoCo allows planes only on static bodies; a box with its +z face on the half-space's surface (the
    # box's centre HALFSPACE_BOX[2] inside the solid side) has the same signed distance to anything near that face
    box_rot = rot * HALFSPACE_TO_PLANE
    attrib.update({"type": "box", "size": vec2str(np.array(HALFSPACE_BOX)), "rgba": "0.6 0.6 0.6 0.3",
                   "pos": vec2str(pos + box_rot.apply([0.0, 0.0, -HALFSPACE_BOX[2]])),
                   "quat": vec2str(np.roll(box_rot.as_quat(), 1))})
    logger.info(f"Contact half-space {name} is on a body that can move: converted as a "
                f"{2 * HALFSPACE_BOX[0]:g} x {2 * HALFSPACE_BOX[1]:g} x {2 * HALFSPACE_BOX[2]:g} m box (change 6.8)")
  etree.SubElement(body, "geom", attrib)
  return name


def geom_body(name):
  return cfg.M_WORLDBODY.find(f".//geom[@name='{name}']").getparent()


class HuntCrossleyForce(IParser):
  """ This class parses and converts the OpenSim `HuntCrossleyForce` XML element to MuJoCo. """

  def parse(self, xml):
    applies = xml.find("appliesForce")
    if applies is not None and applies.text is not None and not str2bool(applies.text.strip()):
      return
    force = xml.attrib["name"]
    geometries = contact_geometries()
    sets = xml.findall("HuntCrossleyForce__ContactParametersSet/objects/HuntCrossleyForce__ContactParameters")
    count = 0
    for k, params in enumerate(sets):
      names = (params.find("geometry").text or "").split()
      for n in names:
        if n not in geometries:
          logger.critical(f"HuntCrossleyForce {force}: contact geometry {n} not found")
          raise RuntimeError
      geoms = [add_contact_geom(n, geometries[n]) for n in names]
      static = element_txt2num(params, "static_friction", 0.0)
      material = tuple(element_txt2num(params, k, 0.0) for k in
                       ("stiffness", "dissipation", "static_friction", "dynamic_friction", "viscous_friction")) \
        + (element_txt2num(xml, "transition_velocity", 0.01),)
      group = next((g for g, v in cfg.CONTACT_GROUPS.items() if v["material"] == material), None)
      if group is None:
        group = force
        cfg.CONTACT_GROUPS[group] = {"material": material, "pairs": [], "geometry_pairs": [],
                                     "static_friction": material[2], "dynamic_friction": material[3]}
      for i in range(len(geoms)):
        for j in range(i + 1, len(geoms)):
          g1, g2 = geoms[i], geoms[j]
          if geom_body(g1) is geom_body(g2):
            continue
          if geometries[g1].tag == "ContactHalfSpace" and geometries[g2].tag == "ContactHalfSpace":
            continue
          # MuJoCo pairs plane-sphere with the plane first
          if geometries[g2].tag == "ContactHalfSpace":
            g1, g2 = g2, g1
          etree.SubElement(cfg.M_CONTACT, "pair", name=f"{force}_{g1}_{g2}", geom1=g1, geom2=g2,
                           condim="3", margin="0",
                           friction=f"{repr(max(static, 0.0))} {repr(max(static, 0.0))} 0.005 0.0001 0.0001",
                           solref=f"{DEFAULT_SOLREF[0]} {DEFAULT_SOLREF[1]}")
          cfg.CONTACT_GROUPS[group]["pairs"].append(f"{force}_{g1}_{g2}")
          cfg.CONTACT_GROUPS[group]["geometry_pairs"].append((g1, g2))
          count += 1
    logger.info(f"HuntCrossleyForce {force}: {count} contact pairs (change 6.6)")
