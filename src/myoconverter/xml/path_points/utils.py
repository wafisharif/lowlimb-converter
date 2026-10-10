# Modified by lowlimb-converter (2026-10-10), from MyoConverter @ cadf380: adds conditional_path_point_dependencies()
# (Station 3, change 3.1; see station3/FINDINGS.md).
""" This module contains a collection of utility functions useful for parsing and converting the OpenSim `PathPointSet`

@author: Aleksi Ikkala
"""

import numpy as np
from scipy.interpolate import interp1d
from scipy.interpolate import UnivariateSpline
import matplotlib
import matplotlib.pyplot as pp
import os

from loguru import logger

from myoconverter.xml.utils import str2vec, fit_spline
from numpy.polynomial.polynomial import Polynomial
from myoconverter.xml import config as cfg

matplotlib.use("agg")
pp.ioff()

_OSIM_CACHE = None  # lowlimb-converter: (file, model, state), see _osim_model()


def _osim_model():
  """ lowlimb-converter: the OpenSim model being converted, loaded once, in its default pose. """
  import opensim
  global _OSIM_CACHE
  if _OSIM_CACHE is None or _OSIM_CACHE[0] != cfg.OSIM_FILE:
    model = opensim.Model(cfg.OSIM_FILE)
    state = model.initSystem()
    _OSIM_CACHE = (cfg.OSIM_FILE, model, state)
  return _OSIM_CACHE[1], _OSIM_CACHE[2]


def _set_coordinates(model, state, values):
  """ lowlimb-converter: set coordinates (unlocked ones; coupled coordinates follow their constraint), realize. """
  import opensim
  cset = model.getCoordinateSet()
  for n, v in values.items():
    if not cset.get(n).getLocked(state):
      cset.get(n).setValue(state, float(v), False)
  constraints = model.getConstraintSet()
  for i in range(constraints.getSize()):
    cc = opensim.CoordinateCouplerConstraint.safeDownCast(constraints.get(i))
    if cc is None or not cc.get_isEnforced():
      continue
    independent = cc.getIndependentCoordinateNames()
    names = [independent.get(k) for k in range(independent.getSize())]
    x = opensim.Vector(len(names), 0.0)
    for k, n in enumerate(names):
      x.set(k, cset.get(n).getValue(state))
    cset.get(cc.getDependentCoordinateName()).setValue(state, cc.getFunction().calcValue(x), False)
  model.realizePosition(state)


class _LocalPath:
  """ lowlimb-converter: OpenSim's path around one conditional point, in the point's body frame.

  For a pose: the previous and next active path points (A, B), the point's own location (C), whether it is active,
  and the condition coordinate's value. Only this local piece of the path depends on how the point is modelled.
  """

  def __init__(self, model, state, points, index, point):
    self.model, self.state, self.points, self.index, self.point = model, state, points, index, point
    self.frame = point.getParentFrame().findBaseFrame()  # MuJoCo bodies are placed at the OpenSim body frames
    self.coordinate = point.getCoordinate()

  def _loc(self, i):
    p = self.points.get(i)
    v = p.getParentFrame().findStationLocationInAnotherFrame(self.state, p.getLocation(self.state), self.frame)
    return np.array([v.get(k) for k in range(3)])

  def snapshot(self, values):
    _set_coordinates(self.model, self.state, values)
    active = lambda i: self.points.get(i).isActive(self.state)
    before = next((i for i in range(self.index - 1, -1, -1) if active(i)), None)
    after = next((i for i in range(self.index + 1, self.points.getSize()) if active(i)), None)
    if before is None or after is None:
      return None
    return (self._loc(before), self._loc(after), self._loc(self.index), active(self.index),
            self.coordinate.getValue(self.state))


def _local_length(a, b, p):
  return np.linalg.norm(a - p, axis=-1) + np.linalg.norm(p - b, axis=-1)


def conditional_path_point_dependencies(xml, force_name, upstream=None, n_train=300, n_sweep=101, h=1e-4,
                                        seed=1):
  """ lowlimb-converter (Station 3, change 3.1): model a ConditionalPathPoint as a point moving with its coordinate,
  fitted so that the muscle's path length and moment arms match OpenSim's.

  OpenSim uses the point only while its coordinate is inside `range`; otherwise the path runs straight from the
  previous active point (A) to the next (B). MuJoCo can't switch a point on and off, so the point becomes a moving
  path point P(q): each axis a quartic of the condition coordinate q (MuJoCo's joint-equality form). The local path
  A-P-B is what changes, so P is fitted to make |AP| + |PB| match OpenSim's local path (|AC| + |CB| while active,
  |AB| while not) and its derivatives (moment arms) with respect to every coordinate that moves A, B or C. The fit
  uses training poses drawn here (seed 1: 300 random poses of those coordinates plus a sweep of q, everything else at
  its default), not the poses Station 3 evaluates on. Moment-arm samples whose finite difference crosses an on/off
  switch are left out (OpenSim's path jumps there).

  Starting point: per axis, the quartic closest to "C while active, the closest point of segment AB while not".
  Candidates compared on the same training objective: the fitted point, upstream's model (`upstream`: a linear or
  quadratic movement from a neighbouring point on the same body, or the point fixed at C when there is no such
  neighbour), and C fixed. The best one is used, so it is never worse than upstream on the training poses.

  :param xml: OpenSim `ConditionalPathPoint` XML element
  :param force_name: Name of the force (muscle) this path point belongs to
  :param upstream: upstream's dependencies dict for this point, or None if upstream would keep it fixed at C
  :return: (dependencies dict for MovingPathPoint.parse, report dict), or None if it can't be evaluated
  """
  import opensim
  from scipy.optimize import least_squares
  model, state = _osim_model()
  muscle = opensim.Muscle.safeDownCast(model.getForceSet().get(force_name))
  if muscle is None:
    return None
  points = muscle.getGeometryPath().getPathPointSet()
  names = [points.get(i).getName() for i in range(points.getSize())]
  if xml.attrib["name"] not in names:
    return None
  index = names.index(xml.attrib["name"])
  point = opensim.ConditionalPathPoint.safeDownCast(points.get(index))
  if point is None:
    return None
  local = _LocalPath(model, state, points, index, point)
  cname = local.coordinate.getName()
  cset = model.getCoordinateSet()
  if local.coordinate.getLocked(state) or cfg.M_WORLDBODY.find(f".//joint[@name='{cname}']") is None:
    return None
  free = [cset.get(i).getName() for i in range(cset.getSize()) if not cset.get(i).getLocked(state)]
  defaults = {n: cset.get(n).getValue(state) for n in free}
  lo_q, hi_q = local.coordinate.getRangeMin(), local.coordinate.getRangeMax()

  try:
    base = local.snapshot(defaults)
    if base is None:
      return None
    # coordinates that move A, B or C relative to the point's body
    relevant = [cname]
    for n in free:
      if n == cname:
        continue
      moved = local.snapshot({**defaults, n: defaults[n] + 0.05})
      if moved is None or any(np.abs(moved[k] - base[k]).max() > 1e-9 for k in range(3)):
        relevant.append(n)
    lo = np.array([cset.get(n).getRangeMin() for n in relevant])
    hi = np.array([cset.get(n).getRangeMax() for n in relevant])
    rng = np.random.default_rng(seed)
    poses = [dict(defaults, **dict(zip(relevant, r))) for r in rng.uniform(lo, hi, size=(n_train, len(relevant)))]
    poses += [dict(defaults, **{cname: q}) for q in np.linspace(lo_q, hi_q, n_sweep)]
    rows = []
    for pose in poses:
      b0 = local.snapshot(pose)
      if b0 is None:
        continue
      fd = []
      for n in relevant:
        sp = local.snapshot({**pose, n: pose[n] + h})
        sm = local.snapshot({**pose, n: pose[n] - h})
        fd.append((sp, sm) if sp is not None and sm is not None and sp[3] == sm[3] == b0[3] else None)
      rows.append((b0, fd))
  finally:
    _set_coordinates(model, state, defaults)

  def unpack(snaps):
    a = np.array([x[0] for x in snaps]); b = np.array([x[1] for x in snaps]); c = np.array([x[2] for x in snaps])
    act = np.array([x[3] for x in snaps]); q = np.array([x[4] for x in snaps])
    return a, b, q, np.where(act, _local_length(a, b, c), _local_length(a, b, b))
  a0, b0, q0, l0 = unpack([r[0] for r in rows])
  fdsets = []
  for j in range(len(relevant)):
    ok = [k for k, r in enumerate(rows) if r[1][j] is not None]
    if ok:
      ap, bp, qp, lp = unpack([rows[k][1][j][0] for k in ok])
      am, bm, qm, lm = unpack([rows[k][1][j][1] for k in ok])
      fdsets.append((ap, bp, qp, am, bm, qm, -(lp - lm) / (2 * h)))

  def residuals(polys):
    pos = lambda q: np.stack([f(q) for f in polys], axis=1)
    r = [_local_length(a0, b0, pos(q0)) - l0]
    for ap, bp, qp, am, bm, qm, ma in fdsets:
      r.append(-(_local_length(ap, bp, pos(qp)) - _local_length(am, bm, pos(qm))) / (2 * h) - ma)
    return np.concatenate(r)

  def scaled(theta):   # three quartics in q mapped to [-1, 1] (better conditioned for the optimiser)
    return [Polynomial(theta[5 * i:5 * i + 5], domain=[lo_q, hi_q], window=[-1, 1]) for i in range(3)]

  # starting point: C while active, closest point of AB while not, on a sweep of q
  xs = np.linspace(lo_q, hi_q, 401)
  target = []
  for q in xs:
    a, b, c, act, _ = local.snapshot({**defaults, cname: q}) or (None, None, None, True, None)
    if act or a is None:
      target.append(c if c is not None else base[2])
    else:
      t = np.clip(np.dot(c - a, b - a) / max(np.dot(b - a, b - a), 1e-30), 0.0, 1.0)
      target.append(a + t * (b - a))
  _set_coordinates(model, state, defaults)
  target = np.array(target)
  theta0 = np.concatenate([Polynomial.fit(xs, target[:, k], 4, domain=[lo_q, hi_q]).coef for k in range(3)])
  solution = least_squares(lambda th: residuals(scaled(th)), theta0, method="trf", max_nfev=2000)
  fitted = [Polynomial(f.convert().coef) for f in scaled(solution.x)]

  fixed = [Polynomial([base[2][k]]) for k in range(3)]
  candidates = [("path fit", fitted), ("fixed at its location (always active)", fixed)]
  if upstream is not None:
    candidates.append(("upstream anchor movement", [Polynomial(upstream[k][1]) for k in ["x", "y", "z"]]))
  scores = [(float(np.sqrt(np.mean(residuals(c) ** 2))), label, c) for label, c in candidates]
  score, label, chosen = min(scores, key=lambda x: x[0])

  dependencies = {}
  for k, axis in enumerate(["x", "y", "z"]):
    polycoef = np.zeros(5)
    coef = chosen[k].coef
    polycoef[:len(coef)] = coef
    values = chosen[k](xs)
    dependencies[axis] = (cname, polycoef, np.array([values.min(), values.max()]))
  report = {"chosen": label, "training_rms_mm": {lab: 1000 * sc for sc, lab, _ in scores},
            "coordinates": relevant, "training_poses": len(rows)}
  return dependencies, report


def update_moving_path_point_location(coordinate_name, path_point):
  """ Calculate median for a coordinate of a MovingPathPoint

  :param coordinate_name: Coordinate name (related x, y, or z coordinate)
  :param path_point: OpenSim `MovingPathPoint` XML element
  :return: Median of given coordinate
  :raises: RuntimeError: If a Coordinate is not found
  :raises: NotImplementedError: If a path point is not formatted as expected
  :raises: NotImplementedError: If a function type is other than 'SimmSpline', 'NaturalCubicSpline', or
  'PiecewiseLinearFunction'

  """

  # Make sure the coordinate exists
  if path_point.find(coordinate_name) is None:
    logger.critical(f"Coordinate f{coordinate_name} was not found in pathpoint {path_point.attrib['name']}")
    raise RuntimeError

  # Check if the function is wrapped in a multiplier function
  mult_fn = path_point.find(f"{coordinate_name}/MultiplierFunction")
  multiplier = 1
  if mult_fn is not None:
    multiplier = float(mult_fn.find("scale").text)
    if multiplier != 1:
      logger.warning(f"Encountered a MultiplierFunction with scale != 1 in pathpoint {path_point.attrib['name']}."
             f"This implementation has not been tested/validated, you better check it out yourself")

  # Parse x and y values
  if path_point.find(".//x") is None:
    logger.critical(f"Path point {path_point.attrib['name']} is not formatted as expected")
    raise NotImplementedError

  x_values = str2vec(path_point.find(f".//{coordinate_name}/*/x").text)
  y_values = str2vec(path_point.find(f".//{coordinate_name}/*/y").text) * multiplier

  # Figure out function type
  function_type = path_point.find(".//x").getparent().tag

  if function_type not in {"SimmSpline", "NaturalCubicSpline", "PiecewiseLinearFunction"}:
    logger.critical(f"Function type {function_type} not implemented (in pathpoint {path_point.attrib['name']})")
    raise NotImplementedError

  # Fit a cubic spline (if more than 2 values and function_type is spline), otherwise fit a piecewise linear line
  if len(x_values) > 3 and function_type in {"SimmSpline", "NaturalCubicSpline"}:
    mdl = UnivariateSpline(x_values, y_values)
  else:
    mdl = interp1d(x_values, y_values, kind="linear", fill_value="extrapolate")

  # Return the median of fit inside given range
  x = np.linspace(x_values[0], x_values[-1], 1000)
  return np.median(mdl(x))

def get_moving_path_point_dependency(path_point, coordinate_name, socket_name, cfg, output_dir):
  """ Get a MovingPathPoint's dependency on an independent coordinate.

  :param path_point:  OpenSim `MovingPathPoint` XML element
  :param coordinate_name: Name coordinate ("x_location", "y_location", or "z_location")
  :param socket_name: Name of the specific coordinate socket ("socket_x_coordinate", "socket_y_coordinate", or
  "socket_z_coordinate")
  :param cfg: Contains all the global config definitions
  :param output_dir: Output directory for plots
  :return: Name of the independent coordinate (joint) `path_point` is dependent on, quartic polynomial of the
  dependency, and movement range
  :raises: RuntimeError: If a Coordinate was not found
  :raises: NotImplementedError: If a path point is not formatted as expected
  :raises: RuntimeError: If a MuJoCo joint that corresponds to a Coordinate cannot be found
  """

  # Make sure the coordinate exists
  if path_point.find(coordinate_name) is None:
    logger.critical(f"Coordinate f{coordinate_name} was not found in path point {path_point.attrib['name']}")
    raise RuntimeError

  # Check if the function is wrapped in a multiplier function
  mult_fn = path_point.find(f"{coordinate_name}/MultiplierFunction")
  multiplier = 1
  if mult_fn is not None:
    multiplier = float(mult_fn.find("scale").text)
    if multiplier != 1:
      logger.warning(f"Encountered a MultiplierFunction with scale != 1 in path point {path_point.attrib['name']}."
             f"This implementation has not been tested/validated, you better check it out yourself")

  # Parse x and y values
  if path_point.find(".//x") is None:
    logger.critical(f"Path point {path_point.attrib['name']} is not formatted as expected")
    raise NotImplementedError

  x_values = str2vec(path_point.find(f".//{coordinate_name}//x").text)
  y_values = str2vec(path_point.find(f".//{coordinate_name}//y").text) * multiplier

  # Figure out function type
  function_type = path_point.find(".//x").getparent().tag

  if function_type not in {"SimmSpline", "NaturalCubicSpline", "PiecewiseLinearFunction"}:
    logger.critical(f"Function type {function_type} not implemented (in {path_point.tag} {path_point.attrib['name']}). "
                    f"Behaviour of this type of function has not been tested.")

  # Get polycoef
  fit, polycoef, range = fit_spline(x_values, y_values)

  # Find the independent coordinate
  independent_coordinate = path_point.find(socket_name).text.split("/")[-1]

  # Make sure the coordinate has been designated as one of the MuJoCo model joints
  if cfg.M_WORLDBODY.find(f".//joint[@name='{independent_coordinate}']") is None:
    logger.critical(f"Could not find a MuJoCo joint that corresponds to independent coordinate {independent_coordinate}")
    raise RuntimeError

  # Plot figures of the approximations (if necessary)
  if not cfg.TREAT_AS_NORMAL_PATH_POINT:
    fig = plot_figure(x_values, y_values, fit, independent_coordinate, path_point.attrib["name"], coordinate_name[0])
    pp.savefig(os.path.join(output_dir, f"{path_point.attrib['name']}_{coordinate_name[0]}.svg"))
    pp.close(fig)

  return independent_coordinate, polycoef, range


def plot_figure(x_values, y_values, fit, independent_joint, path_point_name, coord):

  # Initialise figure
  fig = pp.figure(figsize=(10, 8))

  # Calculate the quartic approximation
  x_approx = np.linspace(min(x_values), max(x_values), 100)
  y_approx = fit(x_approx)

  # Plot the OpenSim data points and approximation
  pp.plot(x_values, y_values, '.', markersize=10, label=f"OpenSim movement function data points")
  pp.plot(x_approx, y_approx, label=f"Approximation of the movement function")
  pp.legend()
  pp.xlabel(f"Independent joint value ({independent_joint})")
  pp.ylabel(f"Dependent joint value ({path_point_name}_{coord})")
  pp.title(f"Approximation of the movement function of MovingPathPoint {path_point_name}, {coord} coordinate")

  return fig
