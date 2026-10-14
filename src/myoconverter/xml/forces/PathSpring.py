# Added by lowlimb-converter (2026-10-12): not in MyoConverter @ cadf380 (Station 6, change 6.5).
""" Contains the `PathSpring` parser (lowlimb-converter, change 6.5).

OpenSim's PathSpring: tension = stiffness * max(length - resting_length, 0) * (1 + dissipation * lengthening speed),
along its geometry path (a passive cable, e.g. an exoskeleton spring). MuJoCo: a spatial tendon (path converted as for
muscles) with a dead-band spring, springlength "0 resting_length" and the same stiffness: no force while the tendon is
shorter than its resting length, -stiffness * (length - resting_length) beyond. That is exact at zero speed.
Approximation: the dissipation (tension times speed, zero when slack) has no MuJoCo equivalent and is not converted;
station6 gate L5 checks the static force and reports this.
"""

from loguru import logger

from myoconverter.xml.parsers import IParser
from myoconverter.xml.utils import str2bool, element_txt2num
from myoconverter.xml.forces.PathActuator import add_path_tendon


class PathSpring(IParser):
  """ This class parses and converts the OpenSim `PathSpring` XML element to MuJoCo. """

  def parse(self, xml):
    applies = xml.find("appliesForce")
    if applies is not None and applies.text is not None and not str2bool(applies.text.strip()):
      return
    name = xml.attrib["name"]
    rest = element_txt2num(xml, "resting_length")
    stiffness = element_txt2num(xml, "stiffness")
    dissipation = element_txt2num(xml, "dissipation", 0.0)
    add_path_tendon(xml, f"{name}_tendon", stiffness=repr(stiffness), springlength=f"0 {repr(rest)}")
    if dissipation != 0:
      logger.warning(f"PathSpring {name}: dissipation {dissipation} is not converted")
    logger.info(f"PathSpring {name}: tendon with a one-sided spring, stiffness {stiffness}, resting length {rest} "
                f"(change 6.5)")
