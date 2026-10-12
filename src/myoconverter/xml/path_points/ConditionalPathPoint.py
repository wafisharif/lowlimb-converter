# Modified by lowlimb-converter (2026-10-10), from MyoConverter @ cadf380: conditional path points are fitted to
# OpenSim's path by conditional_path_point_dependencies(), with upstream's model as one candidate (Station 3, change 3.1).
# Modified by lowlimb-converter (2026-10-12): a point fitted as a switch toward its neighbour moves with a helper joint,
# created here (Station 5, change 5.3).
""" Contains the `ConditionalPathPoint` parser.

@author: Aleksi Ikkala
"""

import numpy as np
import os
import matplotlib
import matplotlib.pyplot as pp

from loguru import logger

from myoconverter.xml.parsers import IParser
from myoconverter.xml.utils import str2vec, add_helper_joint
from myoconverter.xml.path_points.PathPoint import PathPoint
from myoconverter.xml.path_points.MovingPathPoint import MovingPathPoint
from myoconverter.xml.path_points.utils import conditional_path_point_dependencies
from myoconverter.xml import config as cfg

matplotlib.use("agg")
pp.ioff()


class ConditionalPathPoint(IParser):
  """ This class parses and converts the OpenSim `ConditionalPathPoint` XML element to MuJoCo.

  Parsing a ConditionalPathPoint is difficult. We need to 'anchor' the conditional path point to a normal path point,
  and treat it like a moving path point (move from 'anchor' to its actual position when the independent coordinate is
  inside valid range). 

  We approximate the above movement behaviour with either a step function or a rectangular function. In case of a step
  function the movement is further approximated with a linear function, and in case of a rectangular function the
  movement is approximated with a quadratic function. """

  def __init__(self):
    """ Init method """

    #: PathPoint: A PathPoint object for modelling ConditionalPathPoint
    self.pathpoint = PathPoint()

    #: MovingPathPoint: A MovingPathPoint object for modelling ConditionalPathPoint
    self.moving_pathpoint = MovingPathPoint()

  def parse(self, xml, tendon, force_name, **kwargs):
    """ This function handles the actual parsing and converting.

    :param xml: OpenSim `ConditionalPathPoint` XML element
    :param tendon: MuJoCo `tendon/spatial` XML element
    :param force_name: Name of the actuator this path point belongs to
    :param kwargs: Optional keyword arguments
    :return: None
    """

    # Check if we should treat conditional path points as normal path points
    if cfg.TREAT_AS_NORMAL_PATH_POINT:
      logger.info(f"Treating ConditionalPathPoint {xml.attrib['name']}, as a normal PathPoint")
      self.pathpoint.parse(xml, tendon, force_name, **kwargs)
      return

    # lowlimb-converter (change 3.1): fit the point to OpenSim's path; upstream's model is one of the candidates
    upstream = self._upstream_dependencies(xml)
    modelled = conditional_path_point_dependencies(xml, force_name, upstream=upstream)
    if modelled is not None:
      dependencies, report = modelled
      scores = ", ".join(f"{k} {v:.3f} mm" for k, v in report["training_rms_mm"].items())
      logger.info(f"ConditionalPathPoint {xml.attrib['name']} of {force_name}: using '{report['chosen']}' "
                  f"(training RMS residual: {scores}; {report['training_poses']} poses, "
                  f"coordinates {report['coordinates']})")
      if "helper" in dependencies:   # change 5.3: the point's axes follow a helper joint driven by the coordinate
        helper, coordinate, p1 = dependencies["helper"]
        add_helper_joint(cfg.M_WORLDBODY, cfg.M_EQUALITY, helper, coordinate, p1)
        logger.info(f"ConditionalPathPoint {xml.attrib['name']} of {force_name}: helper joint {helper} driven by "
                    f"{coordinate}")
      self.moving_pathpoint.parse(xml, tendon, force_name, dependencies=dependencies, **kwargs)
      return
    logger.warning(f"ConditionalPathPoint {xml.attrib['name']} of {force_name}: could not be evaluated with OpenSim, "
                   "using the upstream method")
    self._upstream_parse(xml, tendon, force_name, upstream, **kwargs)

  def _upstream_parse(self, xml, tendon, force_name, dependencies, **kwargs):
    """ lowlimb-converter: upstream's handling, given its dependencies (None: no anchor, keep as a normal point). """
    if dependencies is None:
      logger.warning(f"Suitable 'anchor' PathPoint was not found for ConditionalPathPoint {xml.attrib['name']}, "
                     "treating this as a normal PathPoint.")
      self.pathpoint.parse(xml, tendon, force_name, **kwargs)
    else:
      self.moving_pathpoint.parse(xml, tendon, force_name, dependencies=dependencies, **kwargs)

  def _upstream_dependencies(self, xml):
    """ Upstream's model of this point (lowlimb-converter: split out of parse(), otherwise unchanged).

    :return: dependencies dict for MovingPathPoint.parse, or None if no anchor point exists
    """

    # Get socket parent frame
    socket_parent_frame = xml.find("socket_parent_frame").text

    # Get parent (all path_points)
    parent = xml.getparent()

    # Find index of this ConditionalPathPoint
    children = parent.getchildren()
    this_idx = parent.index(xml)

    # Check if previous PathPoint exists and is located in the same body
    idx = self._find_previous(children, this_idx-1, socket_parent_frame)

    # If not, check if next PathPoint exists and is located in the same body
    if idx == -1:
      idx = self._find_next(children, this_idx+1, socket_parent_frame)

    # Check again if a suitable anchor was found (idx >= 0)
    if idx == -1:
      return None

    else:

      # Create an output dir for plots (if it doesn't exist)
      output_dir = os.path.join(cfg.OUTPUT_PLOT_FOLDER, "conditional_path_points")
      if not os.path.isdir(output_dir):
        os.makedirs(output_dir)

      # Estimate the movement of this conditional path point in x, y, and z coordinates
      anchor_pos = str2vec(children[idx].find("location").text)
      cond_pos = str2vec(xml.find("location").text)
      range = str2vec(xml.find("range").text)

      coord_names = ["x", "y", "z"]
      dependencies = {"x": None, "y": None, "z": None}
      for coord, name in enumerate(coord_names):

        # Find the independent coordinate
        independent_coordinate = xml.find("socket_coordinate").text.split("/")[-1]

        # Get range of independent_coordinate
        joint = cfg.M_WORLDBODY.find(f".//joint[@name='{independent_coordinate}']")
        independent_joint_range = str2vec(joint.attrib["range"])

        # Get the ideal function to represent the movement of the conditional path point in this coordinate
        x_ideal = np.linspace(independent_joint_range[0], independent_joint_range[1], 100)
        y_ideal = np.ones_like(x_ideal)*anchor_pos[coord]
        y_ideal[np.argmin(np.abs(x_ideal-range[0])):np.argmin(np.abs(x_ideal-range[1]))+1] = cond_pos[coord]

        # Get range of the dependent "joint" (i.e., conditional path point) movement
        dependent_joint_range = np.array([min(y_ideal), max(y_ideal)])

        # A super simple check whether the conditional path point should be modelled as a step function or a
        # rectangular function
        rectangular = np.isclose(y_ideal[0], y_ideal[-1])

        if rectangular:

          # If rectangular function, approximate with a quadratic polynomial; note that this approach is agnostic to
          # the actual location of the rectangle, we just assume it is in the middle of the independent joint range
          x_points = np.array([independent_joint_range[0],
                           independent_joint_range[0] + (independent_joint_range[1] - independent_joint_range[0]) / 2,
                           independent_joint_range[1]])
          ymax = max(y_ideal)
          ymin = min(y_ideal)
          y_points = np.array([y_ideal[0], ymax if not np.isclose(y_ideal[0], ymax) else ymin, y_ideal[-1]])
          fit = np.polynomial.polynomial.Polynomial.fit(x_points, y_points, 2)

          # Get the polycoef representing the approximation
          polycoef = np.zeros((5,))
          polycoef[:fit.coef.shape[0]] = fit.convert().coef

          # For plotting
          y_approx = fit(x_ideal)

        else:

          # If step function, approximate with a linear polynomial; note that this approach is agnostic to where the
          # actual step happens, we just linearly interpolate the movement from min to max of independent joint range
          m = (y_ideal[-1] - y_ideal[0]) / (independent_joint_range[1]-independent_joint_range[0])
          c = y_ideal[-1] - m*independent_joint_range[1]

          # Get the polycoef representing the approximation
          polycoef = np.array([c, m, 0, 0, 0])

          # For plotting
          y_approx = m*x_ideal + c

        # Plot and save a figure of the approximation
        fig = self._plot_figure(x_ideal, y_ideal, y_approx, independent_coordinate, xml.attrib["name"], name,
                                rectangular)
        pp.savefig(os.path.join(output_dir, f"{xml.attrib['name']}_{name}.svg"))
        pp.close(fig)

        # Save the dependency
        dependencies[name] = (independent_coordinate, polycoef, dependent_joint_range)

      return dependencies

  def _find_previous(self, children, idx, socket_parent_frame):
    """ Find previous PathPoint.

    :param children: List of OpenSim path point XML elements
    :param idx: Idx of current path point of interest
    :param socket_parent_frame: Parent frame socket
    :return: -1 if no suitable path point is found, otherwise idx of a path point (in `children`)
    """
    while idx >= 0:
      child = children[idx]
      if child.tag == "PathPoint" and child.find("socket_parent_frame").text == socket_parent_frame:
        break
      idx -= 1
    return idx

  def _find_next(self, children, idx, socket_parent_frame):
    """ Find next PathPoint.

    :param children: List of OpenSim path point XML elements
    :param idx: Idx of current path point of interest
    :param socket_parent_frame: Parent frame socket
    :return: -1 if no suitable path point is found, otherwise idx of a path point (in `children`)
    """
    while idx < len(children):
      child = children[idx]
      if child.tag == "PathPoint" and child.find("socket_parent_frame").text == socket_parent_frame:
        break
      idx += 1
    return idx if idx < len(children) else -1

  def _plot_figure(self, x_ideal, y_ideal, y_approx, independent_joint, dependent_joint, coord, is_rectangular):
    fig = pp.figure(figsize=(10, 8))
    fun_type = "rectangular" if is_rectangular else "step"
    pp.plot(x_ideal, y_ideal, label=f"Ideal {fun_type} function")
    pp.plot(x_ideal, y_approx, label=f"Approximation of the {fun_type} function")
    pp.legend()
    pp.xlabel(f"Independent joint value ({independent_joint})")
    pp.ylabel(f"Dependent joint value ({dependent_joint}_{coord})")
    pp.title(f"Approximation of the {fun_type} function of ConditionalPathPoint {dependent_joint}, {coord} coordinate")
    return fig
