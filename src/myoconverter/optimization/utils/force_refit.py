# Added by lowlimb-converter (2026-10-10); not part of MyoConverter. Apache License 2.0.
# Change 4.2 (same day): training samples without an OpenSim equilibrium are left out, as in Station 4's check.
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
from several fixed starting points, so it is deterministic. Step 3's own result is one of the candidates and is
kept if it scores better, so the refit is never worse on the training poses.
"""
import numpy as np
from loguru import logger
from scipy.optimize import least_squares

import mujoco

ACTIVATIONS = (0.0, 0.25, 0.5, 1.0)


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

  :return: list of coordinate-value dicts, forces (A x P, NaN where equilibrium failed), held activations (A x P)
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
  fmax = muscle.getMaxIsometricForce()
  for p, pose in enumerate(poses):
    _set_coordinates(osim_model, state, pose)
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
  return poses, force, held, relevant


def refit_muscle(mjc_model, osim_model, muscle_name, n_train=400, seed=2):
  """ Refit one muscle's static force parameters in `mjc_model` (modified in place if the refit is better).

  :return: report dict (training RMS errors in % of Fmax for Step 3's parameters and for the refit, which was kept)
  """
  aid = mujoco.mj_name2id(mjc_model, mujoco.mjtObj.mjOBJ_ACTUATOR, muscle_name)
  fmax = osim_model.getMuscles().get(muscle_name).getMaxIsometricForce()
  poses, force, held, relevant = _opensim_samples(osim_model, muscle_name, n_train, seed)

  # MuJoCo tendon length at each training pose
  driven = _joint_plan(mjc_model)
  data = mujoco.MjData(mjc_model)
  length = np.zeros(len(poses))
  for p, pose in enumerate(poses):
    data.qpos[:] = _solve_qpos(mjc_model, driven, pose)
    mujoco.mj_fwdPosition(mjc_model, data)
    length[p] = data.actuator_length[aid]

  valid = np.isfinite(force) & np.isfinite(held)
  act = np.where(valid, held, 0.0)
  target = np.where(valid, force, 0.0) / fmax
  lengths = np.broadcast_to(length, force.shape)

  def residual_for(lengthrange, prm):
    f = static_force(lengths[valid], act[valid], lengthrange, prm) / fmax
    return f - target[valid]

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

  def residual(theta):
    return residual_for(lr, unpack(theta))

  L_old = normalised_length(lr, old_lr, old_prm[0:2])     # Step 3's mapping, expressed on our lengthrange
  starts = [np.array([L_old.mean(), 0.5 * (L_old[1] - L_old[0]), old_prm[4], old_prm[5], old_prm[7],
                      old_prm[2] / fmax])]
  for c0, c1 in ((1.0, 0.3), (0.9, 0.5), (1.1, 0.2)):
    starts.append(np.array([c0, c1, 0.5, 1.6, 1.3, 1.0]))
  lower = np.array([0.0, 1e-3, 0.0, 1.01, 0.0, 0.25])
  upper = np.array([3.0, 3.0, 0.99, 3.0, 10.0, 4.0])
  best = None
  for s in starts:
    s = np.clip(s, lower + 1e-9, upper - 1e-9)
    sol = least_squares(residual, s, bounds=(lower, upper), method="trf", max_nfev=3000)
    rms = float(np.sqrt(np.mean(sol.fun ** 2)))
    if best is None or rms < best[0]:
      best = (rms, sol.x)
  new_rms, theta = best

  report = {"muscle": muscle_name, "coordinates": relevant, "samples": int(valid.sum()),
            "excluded_samples": int((~valid).sum()), "step3_rms_pct": 100 * old_rms, "refit_rms_pct": 100 * new_rms}
  if new_rms < old_rms:
    prm = unpack(theta)
    mjc_model.actuator_lengthrange[aid] = lr
    mjc_model.actuator_gainprm[aid, :9] = prm
    mjc_model.actuator_biasprm[aid, :9] = prm
    report["kept"] = "refit"
  else:
    report["kept"] = "step3"
  return report
