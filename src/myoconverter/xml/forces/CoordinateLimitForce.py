# Modified by lowlimb-converter (2026-10-12), from MyoConverter @ cadf380: CoordinateLimitForce is converted (upstream
# skipped it with a warning; its old, unused O2MConverter code is removed). The limit springs become MuJoCo fixed
# tendons with dead-band springs on the coordinate's joint (Station 6, change 6.4; see station6/GATES.md, gate L4).
""" Contains the `CoordinateLimitForce` parser.

@author: Aleksi Ikkala (upstream stub); conversion by lowlimb-converter
"""

import numpy as np
from lxml import etree

from loguru import logger

from myoconverter.xml.parsers import IParser
from myoconverter.xml.utils import str2bool
from myoconverter.xml import config as cfg

# Number of dead-band springs per limit for OpenSim's smooth onset (see CoordinateLimitForce.parse)
ONSET_NODES = 8
# Lower bound of an upper-limit spring's dead band (and upper bound of a lower-limit spring's): far beyond any joint
FAR = 1.0e6


def smooth_step(t):
  """ Simbody's Function::Step from 0 to 1 over t in [0, 1] (quintic: 10t^3 - 15t^4 + 6t^5), which OpenSim's
  CoordinateLimitForce uses to switch its limit springs (and damping) on over the transition region. """
  t = np.clip(t, 0.0, 1.0)
  return t ** 3 * (10.0 - 15.0 * t + 6.0 * t ** 2)


def onset_springs(stiffness, transition, nodes=ONSET_NODES):
  """ Dead-band springs (k_j, h_j), j = 0..nodes, whose sum  sum_j k_j * max(x - h_j, 0)  is the piecewise-linear
  interpolant of OpenSim's one-sided limit force  stiffness * x * smooth_step(x / transition)  at h_j = j/nodes *
  transition, continued with slope `stiffness` beyond the transition. That continuation is exact: for x >= transition
  OpenSim's force is stiffness * x, and the interpolant passes through (transition, stiffness * transition) with that
  slope. Inside the transition the error is at most max|g''| / 8 * stiffness * transition / nodes^2, with
  g(t) = t * smooth_step(t) (max|g''| = 4.62 at t = 0.386), i.e. 0.58 * stiffness * transition / nodes^2. Slopes
  rise and then fall (the onset curve is convex, then concave), so some k_j are negative (MuJoCo allows negative
  stiffness).

  :param stiffness: limit stiffness (N m / rad or N / m)
  :param transition: transition width (rad or m), > 0
  :return: list of (k_j, h_j), h_j measured from the limit, into the limited region
  """
  h = np.linspace(0.0, transition, nodes + 1)
  f = stiffness * h * smooth_step(h / transition)
  slopes = np.append(np.diff(f) / np.diff(h), stiffness)   # last: beyond the transition
  k = np.diff(np.concatenate([[0.0], slopes]))
  return list(zip(k, h))


class CoordinateLimitForce(IParser):
  """ This class parses and converts the OpenSim `CoordinateLimitForce` XML element to MuJoCo.

  OpenSim (CoordinateLimitForce): beyond `upper_limit`, the coordinate q feels  -K_up * (q - upper) * S((q - upper) / w)
  and below `lower_limit`  K_low * (lower - q) * S((lower - q) / w), with S Simbody's quintic step over the `transition`
  w; plus damping -c * qdot * S(...) beyond either limit. Rotational stiffness, limits, transition and damping are given
  per degree in the file. MuJoCo has no such force, but a fixed tendon of length q (coefficient 1 on the coordinate's
  joint) with a dead-band spring (springlength "low high", stiffness k) applies -k (q - high) above high and
  -k (q - low) below low, and nothing in between. Each limit's smooth onset is the sum of ONSET_NODES + 1 such springs
  (onset_springs: exact beyond the transition, interpolated inside it).

  Approximation: the damping (active only beyond a limit in OpenSim) is not converted; MuJoCo's tendon damping would
  act everywhere. It is reported by gate L4 (force at +-1 rad/s beyond each limit).
  """

  def parse(self, xml):
    """ This function handles the actual parsing and converting.

    :param xml: OpenSim `CoordinateLimitForce` XML element
    :return: None
    :raises: NotImplementedError if the coordinate is not a MuJoCo joint
    """

    name = xml.attrib["name"]
    applies = xml.find("appliesForce")
    if applies is not None and applies.text is not None and not str2bool(applies.text.strip()):
      logger.info(f"CoordinateLimitForce {name} has appliesForce false, not converted")
      return

    coordinate = xml.find("coordinate").text.strip()
    joint = cfg.M_WORLDBODY.find(f".//joint[@name='{coordinate}']")
    if joint is None:
      logger.critical(f"CoordinateLimitForce {name}: coordinate {coordinate} is not a joint of the converted model")
      raise NotImplementedError

    # Rotational coordinates: the file gives limits, transition in degrees and stiffness, damping per degree
    rotational = joint.attrib.get("type", "hinge") == "hinge"
    to_si = np.pi / 180.0 if rotational else 1.0

    def value(tag, default):
      element = xml.find(tag)
      return float(element.text) if element is not None and element.text is not None else default

    upper, lower = value("upper_limit", 0.0) * to_si, value("lower_limit", 0.0) * to_si
    k_upper, k_lower = value("upper_stiffness", 1.0) / to_si, value("lower_stiffness", 1.0) / to_si
    transition = value("transition", 0.1) * to_si
    damping = value("damping", 0.001) / to_si

    count = 0
    for side, limit, stiffness in (("upper", upper, k_upper), ("lower", lower, k_lower)):
      if stiffness == 0:
        continue
      for j, (k, h) in enumerate(onset_springs(stiffness, transition)):
        if k == 0:
          continue
        if side == "upper":
          springlength = f"{repr(-FAR)} {repr(float(limit + h))}"      # pushes q down above limit + h
        else:
          springlength = f"{repr(float(limit - h))} {repr(FAR)}"       # pushes q up below limit - h
        tendon = etree.SubElement(cfg.M_TENDON, "fixed", name=f"{name}_{side}_{j}", springlength=springlength,
                                  stiffness=repr(float(k)))
        etree.SubElement(tendon, "joint", joint=coordinate, coef="1")
        count += 1

    if damping != 0:
      logger.warning(f"CoordinateLimitForce {name}: damping {damping:.6g} beyond the limits is not converted")
    logger.info(f"CoordinateLimitForce {name} on {coordinate}: {count} dead-band springs (change 6.4)")
