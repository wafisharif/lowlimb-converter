# Added by lowlimb-converter (2026-10-10); not part of MyoConverter. Apache License 2.0.
# Change 4.2 (same day): training samples without an OpenSim equilibrium are left out, as in Station 4's check.
# Change 4.3 (2026-10-11): no longer started from Step 3's random result, and weakly pulled towards a rigid-tendon
# estimate, so the result doesn't depend on the particle swarm and left/right copies of a muscle get the same values.
# Change 5.4 (2026-10-12): training poses where the converted muscle path is more than 5 mm from OpenSim's are left
# out, because force parameters can't fix a path error and fitting them there distorts the force everywhere else.
# Change 4.4 (2026-10-13): at full activation and at the passive end, the fit is kept within LEVEL_SLACK of a fit made
# to that level alone (Rajagopal 2016's compliant lower-leg tendons traded full-activation accuracy for the rest); Step
# 3's result is kept only if it is at least as good at every activation level.
""" Deterministic refit of MuJoCo muscle force parameters to OpenSim's static muscle forces (Station 4, change 4.1).

MyoConverter's Step 3 fits four parameters of each MuJoCo muscle (lmin, lmax, fpmax, force scale) to OpenSim's force at
full activation, on a joint grid, with an unseeded particle swarm. Station 4 found the result fragile (a small change
in the converted geometry moved gluteus maximus's passive force error from 2% to 34% of Fmax) and blind to
sub-maximal and passive force.

This module refits, per muscle, the MuJoCo parameters that shape static force: the operating range (how tendon length
maps to MuJoCo's normalised length), lmin, lmax, fpmax and the peak force. The target is OpenSim's equilibrium tendon
force at activations 0, 0.25, 0.5 and 1 on training poses drawn here (seed 2; the coordinates that change the
muscle's length sampled uniformly over their ranges, everything else at its default), leaving out samples where
OpenSim has no equilibrium (fiber clamped at its minimum length while the tendon pulls). It uses scipy's least squares
from four fixed starting points. Step 3's own result is scored on the same data and kept if it is better (since
change 4.4: better or equal at every activation level), so the refit never makes the training fit worse everywhere.

Static force doesn't determine every parameter: in the poses a muscle can reach, several parameter sets give the same
force (Station 4 FINDINGS). Change 4.3 therefore adds a weak penalty (REG_WEIGHT) pulling the parameters towards a
rigid-tendon estimate: normalised length = (tendon length - tendon slack length) / (optimal fiber length x
cos(pennation)), with MuJoCo's default lmin, lmax, fpmax (0.5, 1.6, 1.3) and peak force = max isometric force.
Where the data determine a parameter the penalty changes little (mean static error within 0.06% of Fmax of the
unpenalised fit on the gait models); where they don't, it picks the value closest to that estimate. Before 4.3 the
first starting point was Step 3's result, which comes from an unseeded particle swarm whose result can change from
run to run, so the refitted values could too.

Change 5.4 (Station 5) leaves out training poses where the converted muscle's length differs from OpenSim's by more
than PATH_TOL (5 mm): there the converted path itself is wrong, and no force parameters can make up for it, but the
least squares tried, distorting the force-length curve in every other pose. On the gait models this only happens for
the hip flexors (iliopsoas, iliacus, psoas) with the hip extended beyond -90 degrees, where OpenSim's conditional path
point switches off and its path jumps by about 20 mm (Station 5 FINDINGS): iliopsoas's force error over the poses
with the hip at -90 degrees or more fell from 3.6% to 2.3% of Fmax (a = 1) and from 2.7% to 0.6% (a = 0). If more
than half of a muscle's poses were off, nothing is left out (its path is the problem, not a few poses).

Change 4.4 (Station 4, Rajagopal 2016): a rigid-tendon MuJoCo muscle can't follow how an elastic tendon shifts the
force-length curve with activation, so fitting all four levels at once trades accuracy between them. With Rajagopal
2016's long, compliant lower-leg tendons the equal-weight fit gave up 0.5-1.4% of Fmax at full activation, where
MyoConverter's Step 3 (which fits full activation only) was better. The two ends of the range now have a guarantee:
the same least squares is also run on the full-activation samples alone and on the passive (lowest activation) samples
alone, and while the combined fit is more than LEVEL_SLACK worse than that at either end (on the training poses), that
end's residuals get twice the weight (up to MAX_LEVEL_WEIGHT). The sub-maximal levels pay for it. Everything stays
deterministic (no dependence on Step 3's particle swarm); a muscle that already meets both guarantees keeps exactly
the parameters of the equal-weight fit. A reweighted fit is only kept if its largest error over every training sample
with an OpenSim force (including the poses change 5.4 leaves out) is at most MAX_ERROR_SLACK above the equal-weight
fit's; otherwise the equal-weight fit is used. Without that check, gastrocnemius lateralis in Rajagopal 2016 (30% of
its poses left out by change 5.4) got lmax at its lower bound and a passive force 600-700% of Fmax too high in poses
just outside the fitted ones. Step 3's result is kept only if it is at least as good as the refit at every activation
level.
"""
import numpy as np
from loguru import logger
from scipy.optimize import least_squares

import mujoco

ACTIVATIONS = (0.0, 0.25, 0.5, 1.0)
REG_WEIGHT = 0.01   # change 4.3: a parameter 1 unit from the estimate costs as much as 1% of Fmax RMS force error
GUARDED_LEVELS = (0.0, 1.0)   # change 4.4: activation levels kept close to a fit made to them alone
LEVEL_SLACK = 0.0025          # change 4.4: allowed RMS above that fit (fraction of Fmax)
MAX_LEVEL_WEIGHT = 256        # change 4.4: largest weight given to a guarded level's residuals
MAX_ERROR_SLACK = 0.25        # change 4.4: allowed growth of the largest training error over the equal-weight fit
PATH_TOL = 0.005    # change 5.4: training poses with the converted path further than this from OpenSim's are left out


# ---------------------------------------------------------------------------------------------------------------------
# MuJoCo's static muscle force, vectorised (MuJoCo engine_util_misc.c: mju_muscleGainLength, mju_muscleBias, with
# velocity 0 so the force-velocity factor is 1). Checked against mujoco.mju_muscleGain/mju_muscleBias in the tests.

def mujoco_active_fl(L, lmin, lmax):
  a = 0.5 * (lmin + 1)
  b = 0.5 * (1 + lmax)
  out = np.zeros_like(L)
  m1 = (L > lmin) & (L <= a)
  x = (L[m1] - lmin) / max(1e-10, a - lmin)
  out[m1] = 0.5 * x * x
  m2 = (L > a) & (L <= 1)
  x = (1 - L[m2]) / max(1e-10, 1 - a)
  out[m2] = 1 - 0.5 * x * x
  m3 = (L > 1) & (L <= b)
  x = (L[m3] - 1) / max(1e-10, b - 1)
  out[m3] = 1 - 0.5 * x * x
  m4 = (L > b) & (L < lmax)
  x = (lmax - L[m4]) / max(1e-10, lmax - b)
  out[m4] = 0.5 * x * x
  return out


def mujoco_passive_fl(L, lmax, fpmax):
  b = 0.5 * (1 + lmax)
  out = np.zeros_like(L)
  m1 = (L > 1) & (L <= b)
  x = (L[m1] - 1) / max(1e-10, b - 1)
  out[m1] = fpmax * 0.5 * x * x
  m2 = L > b
  x = (L[m2] - b) / max(1e-10, b - 1)
  out[m2] = fpmax * (0.5 + x)
  return out


def normalised_length(length, lengthrange, prm_range):
  L0 = (lengthrange[1] - lengthrange[0]) / max(1e-15, prm_range[1] - prm_range[0])
  return prm_range[0] + (length - lengthrange[0]) / max(1e-15, L0)


def static_force(length, act, lengthrange, prm):
  """ Tension (positive) of a MuJoCo muscle at rest. prm = gainprm[0:9] (range0, range1, force, scale, lmin, lmax,
  vmax, fpmax, fvmax); force must be > 0 (explicit peak force). """
  L = normalised_length(length, lengthrange, prm[0:2])
  return prm[2] * (act * mujoco_active_fl(L, prm[4], prm[5]) + mujoco_passive_fl(L, prm[5], prm[7]))


# ---------------------------------------------------------------------------------------------------------------------
# Posing the MuJoCo model: coordinates by joint name, dependent joints from their joint-equality polynomials.

def _joint_plan(m):
  driven = {}
  for e in range(m.neq):
    if int(m.eq_type[e]) == int(mujoco.mjtEq.mjEQ_JOINT):
      driven[int(m.eq_obj1id[e])] = (int(m.eq_obj2id[e]), np.array(m.eq_data[e, :5], dtype=float))
  return driven


def _solve_qpos(m, driven, values):
  qpos = m.qpos0.copy()
  for name, v in values.items():
    j = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)
    if j >= 0 and j not in driven:
      qpos[m.jnt_qposadr[j]] = v
  done = {j for j in range(m.njnt) if j not in driven}
  todo = dict(driven)
  while todo:
    progressed = False
    for j1, (j2, c) in list(todo.items()):
      if j2 >= 0 and j2 not in done:
        continue
      x = qpos[m.jnt_qposadr[j2]] - m.qpos0[m.jnt_qposadr[j2]] if j2 >= 0 else 0.0
      qpos[m.jnt_qposadr[j1]] = m.qpos0[m.jnt_qposadr[j1]] + c[0] + c[1] * x + c[2] * x ** 2 + c[3] * x ** 3 + c[4] * x ** 4
      done.add(j1)
      del todo[j1]
      progressed = True
    if not progressed:
      raise RuntimeError("joint equalities form a cycle")
  return qpos


# ---------------------------------------------------------------------------------------------------------------------

def _opensim_samples(osim_model, muscle_name, n_train, seed):
  """ Training poses for one muscle and OpenSim's equilibrium tendon force at each activation level.

  :return: list of coordinate-value dicts, forces (A x P, NaN where equilibrium failed), held activations (A x P),
    the coordinates sampled, and OpenSim's muscle-tendon length at each pose (change 5.4)
  """
  import opensim
  from myoconverter.xml.path_points.utils import _set_coordinates
  state = osim_model.initSystem()
  cset = osim_model.getCoordinateSet()
  muscles = osim_model.getMuscles()
  muscle = muscles.get(muscle_name)
  free = [cset.get(i).getName() for i in range(cset.getSize()) if not cset.get(i).getLocked(state)]
  defaults = {n: cset.get(n).getValue(state) for n in free}
  _set_coordinates(osim_model, state, defaults)
  base = muscle.getLength(state)
  relevant = []
  for n in free:
    for dq in (0.05, -0.05):
      _set_coordinates(osim_model, state, {**defaults, n: defaults[n] + dq})
      if abs(muscle.getLength(state) - base) > 1e-9:
        relevant.append(n)
        break
  _set_coordinates(osim_model, state, defaults)
  lo = np.array([cset.get(n).getRangeMin() for n in relevant])
  hi = np.array([cset.get(n).getRangeMax() for n in relevant])
  rng = np.random.default_rng(seed)
  poses = [dict(defaults, **dict(zip(relevant, r))) for r in rng.uniform(lo, hi, size=(n_train, len(relevant)))]
  force = np.full((len(ACTIVATIONS), len(poses)), np.nan)
  held = np.full((len(ACTIVATIONS), len(poses)), np.nan)
  path_length = np.full(len(poses), np.nan)
  fmax = muscle.getMaxIsometricForce()
  for p, pose in enumerate(poses):
    _set_coordinates(osim_model, state, pose)
    path_length[p] = muscle.getLength(state)
    for k, a in enumerate(ACTIVATIONS):
      for i in range(muscles.getSize()):
        muscles.get(i).setActivation(state, a)
      try:
        osim_model.equilibrateMuscles(state)
        osim_model.realizeDynamics(state)
      except Exception:
        continue
      # change 4.2: no OpenSim equilibrium (fiber clamped at its minimum length while the tendon pulls) -> no target
      if abs(muscle.getTendonForce(state) - muscle.getFiberForceAlongTendon(state)) > 1e-6 * fmax:
        continue
      force[k, p] = muscle.getTendonForce(state)
      held[k, p] = muscle.getActivation(state)
  _set_coordinates(osim_model, state, defaults)
  return poses, force, held, relevant, path_length


def refit_muscle(mjc_model, osim_model, muscle_name, n_train=400, seed=2):
  """ Refit one muscle's static force parameters in `mjc_model` (modified in place if the refit is better).

  :return: report dict (training RMS errors in % of Fmax for Step 3's parameters and for the refit, which was kept)
  """
  aid = mujoco.mj_name2id(mjc_model, mujoco.mjtObj.mjOBJ_ACTUATOR, muscle_name)
  fmax = osim_model.getMuscles().get(muscle_name).getMaxIsometricForce()
  poses, force, held, relevant, osim_length = _opensim_samples(osim_model, muscle_name, n_train, seed)

  # MuJoCo tendon length at each training pose
  driven = _joint_plan(mjc_model)
  data = mujoco.MjData(mjc_model)
  length = np.zeros(len(poses))
  for p, pose in enumerate(poses):
    data.qpos[:] = _solve_qpos(mjc_model, driven, pose)
    mujoco.mj_fwdPosition(mjc_model, data)
    length[p] = data.actuator_length[aid]

  valid = np.isfinite(force) & np.isfinite(held)
  # change 5.4: leave out poses where the converted path is off (unless that is most of them)
  off_path = ~(np.abs(length - osim_length) <= PATH_TOL)
  if off_path.any() and off_path.mean() <= 0.5:
    valid &= ~off_path[None, :]
    worst = 1000 * np.nanmax(np.abs(length - osim_length))
    logger.info(f"{muscle_name}: {int(off_path.sum())} of {len(poses)} training poses left out, converted path more "
                f"than {1000 * PATH_TOL:g} mm from OpenSim's (worst {worst:.1f} mm)")
  elif off_path.any():
    logger.warning(f"{muscle_name}: converted path more than {1000 * PATH_TOL:g} mm from OpenSim's in "
                   f"{int(off_path.sum())} of {len(poses)} training poses; all kept")
  act = np.where(valid, held, 0.0)
  target = np.where(valid, force, 0.0) / fmax
  lengths = np.broadcast_to(length, force.shape)

  def residual_for(lengthrange, prm, mask=None):
    mask = valid if mask is None else mask
    f = static_force(lengths[mask], act[mask], lengthrange, prm) / fmax
    return f - target[mask]

  def level_mask(a):                   # change 4.4: the samples of one activation level
    mask = np.zeros_like(valid)
    mask[ACTIVATIONS.index(a)] = True
    return mask & valid

  def level_rms(lengthrange, prm):
    out = {}
    for k, a in enumerate(ACTIVATIONS):
      mask = np.zeros_like(valid)
      mask[k] = True
      mask &= valid
      out[a] = float(np.sqrt(np.mean(residual_for(lengthrange, prm, mask) ** 2))) if mask.any() else float("nan")
    return out

  # Step 3's parameters (MuJoCo's own definition; gainprm[2] is the peak force in N)
  old_lr = np.array(mjc_model.actuator_lengthrange[aid], dtype=float)
  old_prm = np.array(mjc_model.actuator_gainprm[aid, :9], dtype=float)
  old_rms = float(np.sqrt(np.mean(residual_for(old_lr, old_prm) ** 2)))

  # refit: L = c0 + c1 * (length - mid) / half, parameters (c0, c1, lmin, lmax, fpmax, F0/Fmax)
  lr = np.array([length.min(), length.max()])
  if lr[1] - lr[0] < 1e-6:
    lr = lr + np.array([-0.005, 0.005])
  mid, half = lr.mean(), 0.5 * (lr[1] - lr[0])

  def unpack(theta):
    c0, c1, lmin, lmax, fpmax, f0 = theta
    prm = old_prm.copy()
    prm[0], prm[1] = c0 - c1, c0 + c1          # normalised length at lr[0] and lr[1]
    prm[2] = f0 * fmax
    prm[4], prm[5], prm[7] = lmin, lmax, fpmax
    return prm

  lower = np.array([0.0, 1e-3, 0.0, 1.01, 0.0, 0.25])
  upper = np.array([3.0, 3.0, 0.99, 3.0, 10.0, 4.0])

  # change 4.3: rigid-tendon estimate of the parameters, and a weak penalty towards it (see the module docstring)
  osim_muscle = osim_model.getMuscles().get(muscle_name)
  fiber = osim_muscle.getOptimalFiberLength() * np.cos(osim_muscle.getPennationAngleAtOptimalFiberLength())
  estimate = np.array([(mid - osim_muscle.getTendonSlackLength()) / fiber, half / fiber, 0.5, 1.6, 1.3, 1.0])
  estimate = np.clip(estimate, lower + 1e-3, upper - 1e-3)
  scale = np.array([1.0, estimate[1], 1.0, 1.0, 1.0, 1.0])     # the range's slope counts relative to its estimate

  starts = [estimate.copy()]
  for c0, c1 in ((1.0, 0.3), (0.9, 0.5), (1.1, 0.2)):
    starts.append(np.array([c0, c1, 0.5, 1.6, 1.3, 1.0]))

  def fit(mask, weights=None, first=None):
    w = np.ones(int(mask.sum()))
    for a, wa in (weights or {}).items():
      w[level_mask(a)[mask]] = wa
    pen = REG_WEIGHT * np.sqrt(float(w.sum())) / scale

    def residual(theta):
      return np.concatenate([w * residual_for(lr, unpack(theta), mask), pen * (theta - estimate)])
    best = None
    for s in ([first] if first is not None else []) + starts:
      s = np.clip(s, lower + 1e-9, upper - 1e-9)
      sol = least_squares(residual, s, bounds=(lower, upper), method="trf", max_nfev=3000)
      if best is None or sol.cost < best[0]:
        best = (sol.cost, sol.x)
    return best[1]

  theta = fit(valid)
  # change 4.4: keep each guarded level (full activation, and the passive end) within LEVEL_SLACK of the fit made to
  # that level alone, by doubling the weight of a level that misses it (up to MAX_LEVEL_WEIGHT)
  best_alone = {a: float(np.sqrt(np.mean(residual_for(lr, unpack(fit(level_mask(a))), level_mask(a)) ** 2)))
                for a in GUARDED_LEVELS if level_mask(a).any()}
  weights = {a: 1 for a in best_alone}
  equal_theta = theta
  while True:
    rms = level_rms(lr, unpack(theta))
    missed = [a for a in best_alone if rms[a] > best_alone[a] + LEVEL_SLACK and weights[a] < MAX_LEVEL_WEIGHT]
    if not missed:
      break
    for a in missed:
      weights[a] *= 2
    theta = fit(valid, weights, first=theta)

  # The reweighted fit must also stay sane on every training sample with an OpenSim force, including the poses left
  # out by change 5.4 (path a few mm off): if its largest error there exceeds the equal-weight fit's by more than
  # MAX_ERROR_SLACK, the equal-weight fit is used instead (heavy weights can pick a degenerate curve, e.g. lmax at its
  # bound, whose passive force explodes just outside the fitted lengths).
  with_force = np.isfinite(force) & np.isfinite(held)

  def max_error(th):
    f = static_force(lengths[with_force], held[with_force], lr, unpack(th)) / fmax
    return float(np.max(np.abs(f - force[with_force] / fmax))) if with_force.any() else 0.0
  stopped = theta is not equal_theta and max_error(theta) > max_error(equal_theta) + MAX_ERROR_SLACK
  if stopped:
    theta, weights = equal_theta, {a: 1 for a in best_alone}
  new_rms = float(np.sqrt(np.mean(residual_for(lr, unpack(theta)) ** 2)))   # data only, comparable with Step 3's

  report = {"muscle": muscle_name, "coordinates": relevant, "samples": int(valid.sum()),
            "excluded_samples": int((~valid).sum()), "off_path_poses": int(off_path.sum()),
            "step3_rms_pct": 100 * old_rms, "refit_rms_pct": 100 * new_rms, "level_weights": dict(weights),
            "weighting_dropped_by_max_error": stopped,
            "level_alone_rms_pct": {a: 100 * v for a, v in best_alone.items()},
            "step3_level_rms_pct": {a: 100 * v for a, v in level_rms(old_lr, old_prm).items()},
            "refit_level_rms_pct": {a: 100 * v for a, v in level_rms(lr, unpack(theta)).items()}}
  # change 4.4: Step 3's particle-swarm result is kept only if it is at least as good at every activation level (the
  # refit no longer minimises the plain sum over levels, so that sum can't decide; before 4.4: new_rms < old_rms)
  old_levels, new_levels = report["step3_level_rms_pct"], report["refit_level_rms_pct"]
  step3_dominates = all(old_levels[a] <= new_levels[a] for a in ACTIVATIONS if np.isfinite(new_levels[a]))
  if not step3_dominates:
    prm = unpack(theta)
    mjc_model.actuator_lengthrange[aid] = lr
    mjc_model.actuator_gainprm[aid, :9] = prm
    mjc_model.actuator_biasprm[aid, :9] = prm
    report["kept"] = "refit"
  else:
    report["kept"] = "step3"
  return report
