# Added by lowlimb-converter (2026-10-12): not in MyoConverter @ cadf380 (Station 6, change 6.5).
""" Contains the `PathActuator` parser (lowlimb-converter, change 6.5).

OpenSim's PathActuator pulls along a geometry path with tension = optimal_force * control (an idealised cable, e.g. an
exoskeleton's assistive cable). MuJoCo: the path becomes a spatial tendon (path points and wraps converted exactly as
for muscles), and a general actuator on it with gain -optimal_force: MuJoCo's actuator force acts along the tendon length
(positive lengthens it), OpenSim's tension shortens the path. Control range from min/max_control. Checked by
station6 gate L5 (generalized force per unit control against OpenSim, in Station 3's poses).
"""

from lxml import etree

from loguru import logger

from myoconverter.xml.parsers import IParser
from myoconverter.xml.utils import str2bool, element_txt2num
from myoconverter.xml import config as cfg


def add_path_tendon(xml, tendon_name, **attrib):
  """ Spatial tendon with the force's path points and path wraps (as Muscle.parse does for muscles). """
  tendon = etree.SubElement(cfg.M_TENDON, "spatial", name=tendon_name, **attrib)
  path = xml.find("GeometryPath")
  if path is None:
    logger.critical(f"{xml.tag} {xml.attrib['name']} has no GeometryPath")
    raise RuntimeError
  cfg.PATH_POINT_PARSER.parse_all(path.find("PathPointSet/objects"), tendon=tendon, force_name=xml.attrib["name"])
  cfg.PATH_WRAP_SET_PARSER.parse(path.find("PathWrapSet"), tendon=tendon, force_name=xml.attrib["name"])
  return tendon


class PathActuator(IParser):
  """ This class parses and converts the OpenSim `PathActuator` XML element to MuJoCo. """

  def parse(self, xml):
    applies = xml.find("appliesForce")
    if applies is not None and applies.text is not None and not str2bool(applies.text.strip()):
      return
    name = xml.attrib["name"]
    optimal_force = element_txt2num(xml, "optimal_force", 1.0)
    add_path_tendon(xml, f"{name}_tendon")
    lo, hi = element_txt2num(xml, "min_control", float("-inf")), element_txt2num(xml, "max_control", float("inf"))
    attrib = {"name": name, "tendon": f"{name}_tendon", "gainprm": f"{repr(-optimal_force)} 0 0 0 0 0 0 0 0 0",
              "biastype": "none", "gaintype": "fixed", "dyntype": "none"}
    if lo > float("-inf") and hi < float("inf"):
      attrib.update({"ctrllimited": "true", "ctrlrange": f"{repr(lo)} {repr(hi)}"})
    else:
      attrib["ctrllimited"] = "false"
    # placed before the first muscle, as CoordinateActuator does, so the actuator order is stable
    first_muscle = cfg.M_ACTUATOR.find("muscle")
    idx = len(cfg.M_ACTUATOR.getchildren()) if first_muscle is None else cfg.M_ACTUATOR.index(first_muscle)
    cfg.M_ACTUATOR.insert(idx, etree.Element("general", attrib=attrib))
    logger.info(f"PathActuator {name}: tendon + general actuator, gain {-optimal_force} (change 6.5)")
