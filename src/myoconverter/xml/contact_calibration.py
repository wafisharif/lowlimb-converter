# Added by lowlimb-converter (2026-10-12): not in MyoConverter @ cadf380 (Station 6, change 6.6).
# Modified by lowlimb-converter (2026-10-13): coordinates driven by a coordinate coupler follow it in the rigid model
# instead of being locked against it (OpenSim couldn't assemble such models), the MuJoCo copy is posed the same way, and
# the root to drop is the first ground joint with an unlocked translation (Station 6, change 6.9).
# Modified by lowlimb-converter (2026-10-14): the calibrated parameters must keep the whole, unlocked model stable
# (Station 6, change 6.12).
# Modified by lowlimb-converter (2026-10-14): OpenSim's drops are simulated in a fresh process (Station 6, change 6.13).
""" Calibrate the converted contact pairs against OpenSim simulations of the model (lowlimb-converter, change 6.6).

OpenSim's Hunt-Crossley contact (force ~ k x^1.5 (1 + 1.5 c v), Stribeck friction) has no MuJoCo equivalent: MuJoCo's
soft contact is a mass-normalised linear spring-damper set by solref (time constant, damping ratio) with Coulomb
friction. So the pair parameters are fitted to what OpenSim does with the model: the model is made rigid in its default
pose (every coordinate except the root joint's locked, and those a coupler drives following it; every force except the
contact forces off), dropped from 0.02 m and 0.10 m above the height where its lowest contact geometry touches, and
simulated for 1 s in OpenSim (accuracy 1e-8) and in MuJoCo (the converted model's own time step and integrator). The
time constant, damping ratio and friction coefficient of each group of pairs with the same OpenSim material minimise
the RMS difference of the root position over both drops (multi-start Nelder-Mead in log space; time constant at
least 2 time steps, MuJoCo's stability limit).

Change 6.12: the parameters must also keep the whole converted model (nothing locked, no muscle excitation) stable
when it is released from its keyframe for 1 s: with WalkerModel's best-fitting parameters (time constant 7 ms, damping
ratio 0.31) the free model, whose shanks weigh 0.05 kg, was thrown 4 m into the air. The best-fitting candidate (the
refined fits, then the start grid) that passes is used; if none does, the pairs keep MuJoCo's defaults.

Only these two drops are used; station6/GATES.md (gate L7) holds out other drops (other heights, tilted, sliding) to
check the result. Models whose root joint can't translate, or whose root translation doesn't separate the contacts
(nothing to drop), keep MuJoCo's default solref and OpenSim's static friction, with a warning. The result is written
to <output>/contact_calibration.json.
"""
import json
import os

import numpy as np
from loguru import logger

DROPS = (0.02, 0.10)          # m above touching (station6/GATES.md, L7 calibration scenarios)
DURATION, SAMPLE, ACCURACY = 1.0, 0.001, 1e-8


def coupled_coordinates(model):
  """ change 6.9: the coordinates an enforced CoordinateCouplerConstraint drives. """
  import opensim
  out = set()
  cons = model.getConstraintSet()
  for i in range(cons.getSize()):
    cc = opensim.CoordinateCouplerConstraint.safeDownCast(cons.get(i))
    if cc is not None and cc.get_isEnforced():
      out.add(cc.getDependentCoordinateName())
  return out


def _rigid_opensim(osim_path):
  import opensim
  model = opensim.Model(osim_path)
  ground = model.getGround().getName()
  root = None
  js = model.getJointSet()
  for i in range(js.getSize()):
    j = js.get(i)
    # change 6.9: a joint locked in place (ToyLandingModel's platform) isn't the one to drop
    if j.getParentFrame().findBaseFrame().getName() == ground and \
        any(j.get_coordinates(k).getMotionType() == 2 and not j.get_coordinates(k).getDefaultLocked()
            for k in range(j.numCoordinates())):
      root = j
      break
  if root is None:
    return None
  root_names = [root.get_coordinates(k).getName() for k in range(root.numCoordinates())]
  cs = model.getCoordinateSet()
  coupled = coupled_coordinates(model)            # change 6.9: these follow their coupler
  for i in range(cs.getSize()):
    if cs.get(i).getName() not in root_names and cs.get(i).getName() not in coupled:
      cs.get(i).setDefaultLocked(True)
  fs = model.getForceSet()
  for i in range(fs.getSize()):
    if opensim.HuntCrossleyForce.safeDownCast(fs.get(i)) is None:
      fs.get(i).set_appliesForce(False)
  state = model.initSystem()
  return model, state, root, root_names


def _vertical(model, state, root):
  """Name of the root's translation that raises the root body most against gravity."""
  g = model.get_gravity()
  up = -np.array([g.get(i) for i in range(3)])
  body = root.getChildFrame().findBaseFrame()
  best, best_dot = None, -np.inf
  for k in range(root.numCoordinates()):
    c = root.get_coordinates(k)
    if c.getMotionType() != 2:
      continue
    p0 = body.getPositionInGround(state)
    p0 = np.array([p0.get(i) for i in range(3)])
    c.setValue(state, c.getValue(state) + 1e-3, False)
    model.realizePosition(state)
    p1 = body.getPositionInGround(state)
    p1 = np.array([p1.get(i) for i in range(3)])
    c.setValue(state, c.getValue(state) - 1e-3, False)
    model.realizePosition(state)
    dot = (p1 - p0) @ up
    if dot > best_dot:
      best, best_dot = c.getName(), dot
  return best


def _contact_gap(model, state, pairs):
  """Smallest signed distance over the contact pairs (OpenSim geometry; sphere-sphere and sphere-half-space)."""
  import opensim
  from scipy.spatial.transform import Rotation

  def pose(cg):
    X = cg.getFrame().getTransformInGround(state)
    R = np.array([[X.R().get(i, j) for j in range(3)] for i in range(3)])
    p = np.array([X.p().get(i) for i in range(3)])
    loc = np.array([cg.get_location().get(i) for i in range(3)])
    ori = np.array([cg.get_orientation().get(i) for i in range(3)])
    return p + R @ loc, R @ Rotation.from_euler("XYZ", ori).as_matrix()

  gaps = []
  for a, b in pairs:
    sa, sb = opensim.ContactSphere.safeDownCast(a), opensim.ContactSphere.safeDownCast(b)
    pa, Ra = pose(a)
    pb, Rb = pose(b)
    if sa is not None and sb is not None:
      gaps.append(np.linalg.norm(pa - pb) - sa.getRadius() - sb.getRadius())
    elif sa is not None:
      gaps.append(-Rb[:, 0] @ (pa - pb) - sa.getRadius())
    elif sb is not None:
      gaps.append(-Ra[:, 0] @ (pb - pa) - sb.getRadius())
  return min(gaps)


_CHILD = """
import pickle, sys
sys.path.insert(0, sys.argv[3])
import myoconverter.xml.config  # noqa: F401  (the converter's modules are imported through config)
from myoconverter.xml.contact_calibration import _opensim_drops_here
with open(sys.argv[1], "rb") as f:
  args = pickle.load(f)
result = _opensim_drops_here(*args)
with open(sys.argv[2], "wb") as f:
  pickle.dump(result, f)
"""
ATTEMPTS = 2                  # change 6.13: fresh processes tried before the calibration gives up


def _opensim_drops(osim_path, contact_pairs):
  """ Root-coordinate trajectories of the rigid model dropped from DROPS; None if it can't be dropped.

  Change 6.13: simulated in a fresh Python process. Inside the conversion's own process (after the path-point fits'
  OpenSim model), OpenSim's first integration step of ToyLandingModel's drop failed now and then ("Unable to advance
  time past 0": 1 in 12 conversions, also with that model released); in a fresh process it never failed (48 of 48).
  A failure is tried once more in another fresh process; if that fails too, RuntimeError with OpenSim's message.
  """
  import pickle
  import subprocess
  import sys
  import tempfile
  src = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
  with tempfile.TemporaryDirectory() as tmp:
    args, out = os.path.join(tmp, "args.pkl"), os.path.join(tmp, "drops.pkl")
    with open(args, "wb") as f:
      pickle.dump((osim_path, [tuple(p) for p in contact_pairs]), f)
    for attempt in range(1, ATTEMPTS + 1):
      p = subprocess.run([sys.executable, "-c", _CHILD, args, out, src], capture_output=True, text=True)
      if p.returncode == 0 and os.path.isfile(out):
        with open(out, "rb") as f:
          return pickle.load(f)
      lines = [ln for ln in (p.stdout + p.stderr).splitlines() if ln.strip() and not ln.startswith("[")]
      error = "\n".join(lines[-6:]) or f"exit code {p.returncode}"
      if attempt < ATTEMPTS:
        logger.warning(f"OpenSim drop of the rigid model failed (attempt {attempt}), trying a fresh process: {error}")
  raise RuntimeError(error)


def _opensim_drops_here(osim_path, contact_pairs):
  """ _opensim_drops in this process (change 6.13: run in a fresh one). """
  import opensim
  rigid = _rigid_opensim(osim_path)
  if rigid is None:
    return None
  model, state, root, root_names = rigid
  vertical = _vertical(model, state, root)
  cgs = model.getContactGeometrySet()
  pairs = [(cgs.get(a), cgs.get(b)) for a, b in contact_pairs if cgs.contains(a) and cgs.contains(b)]
  if vertical is None or not pairs:
    return None
  cs = model.getCoordinateSet()
  y0 = cs.get(vertical).getDefaultValue()

  def gaps(y):
    cs.get(vertical).setValue(state, y, False)
    model.realizePosition(state)
    return np.array([_contact_gap(model, state, [p]) for p in pairs])
  # change 6.9: only pairs that raising the root separates (not two parts of the rigid model, e.g. heel and toes)
  moving = np.abs(gaps(y0 + 0.1) - gaps(y0)) > 1e-9
  pairs = [p for p, keep in zip(pairs, moving) if keep]
  if not pairs:
    return None

  def gap(y):
    cs.get(vertical).setValue(state, y, False)
    model.realizePosition(state)
    return _contact_gap(model, state, pairs)
  g0, g1 = gap(y0), gap(y0 + 0.1)
  touch = y0 - g0 * 0.1 / (g1 - g0)
  touch -= gap(touch) * 0.1 / (g1 - g0)
  times = np.arange(0.0, DURATION + 1e-12, SAMPLE)
  runs = []
  for h in DROPS:
    s = model.initializeState()
    for n in root_names:
      cs.get(n).setValue(s, cs.get(n).getDefaultValue(), False)
    cs.get(vertical).setValue(s, touch + h, False)
    model.realizeVelocity(s)
    q0 = {n: cs.get(n).getValue(s) for n in root_names}
    manager = opensim.Manager(model)
    manager.setIntegratorAccuracy(ACCURACY)
    s.setTime(0.0)
    manager.initialize(s)
    traj = np.zeros((len(times), len(root_names)))
    for k, t in enumerate(times):
      if t > 0:
        s = manager.integrate(float(t))
      traj[k] = [cs.get(n).getValue(s) for n in root_names]
    runs.append((q0, traj))
  translational = [root.get_coordinates(k).getMotionType() == 2 for k in range(root.numCoordinates())]
  # change 6.9: the rigid pose (every coordinate's value, coupled ones as assembled), for the MuJoCo copy
  s = model.initializeState()
  pose = {cs.get(i).getName(): cs.get(i).getValue(s) for i in range(cs.getSize())}
  return {"root": root.getName(), "coordinates": root_names, "translational": translational, "times": times,
          "runs": runs, "touch": touch, "vertical": vertical, "pose": pose}


def _rigid_mujoco(xml_path, keep, pose):
  """The converted model, rigid in OpenSim's rigid pose (coordinate name -> value; joints the converter couples
  follow their joint equalities, change 6.9) except for the joints in `keep`."""
  import mujoco
  from myoconverter.optimization.utils.force_refit import _joint_plan, _solve_qpos
  spec = mujoco.MjSpec.from_file(xml_path)
  for g in list(spec.geoms):
    if g.type == mujoco.mjtGeom.mjGEOM_MESH:
      spec.delete(g)
  for mesh in list(spec.meshes):
    spec.delete(mesh)
  m0 = spec.compile()
  d0 = mujoco.MjData(m0)
  d0.qpos[:] = _solve_qpos(m0, _joint_plan(m0), pose)
  mujoco.mj_kinematics(m0, d0)
  for j in list(spec.joints):
    if j.name in keep:
      continue
    body = j.parent
    spec.delete(j)
    b = mujoco.mj_name2id(m0, mujoco.mjtObj.mjOBJ_BODY, body.name)
    p = int(m0.body_parentid[b])
    body.pos = d0.xmat[p].reshape(3, 3).T @ (d0.xpos[b] - d0.xpos[p])
    inv = np.zeros(4)
    mujoco.mju_negQuat(inv, d0.xquat[p])
    rel = np.zeros(4)
    mujoco.mju_mulQuat(rel, inv, d0.xquat[b])
    body.alt.type = mujoco.mjtOrientation.mjORIENTATION_QUAT
    body.quat = rel
  for collection in (spec.equalities, spec.actuators, spec.tendons, spec.keys):
    for e in list(collection):
      spec.delete(e)
  return spec.compile()


def _mujoco_run(m, d, names, q0, times):
  import mujoco
  mujoco.mj_resetData(m, d)
  qadr = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in names]
  for a, n in zip(qadr, names):
    d.qpos[a] = q0[n]
  mujoco.mj_forward(m, d)
  t, out = [0.0], [d.qpos[qadr].copy()]
  while d.time < times[-1] - 1e-12:
    mujoco.mj_step(m, d)
    t.append(d.time)
    out.append(d.qpos[qadr].copy())
    if not np.all(np.isfinite(out[-1])):
      return None
  out = np.array(out)
  return np.column_stack([np.interp(times, t, out[:, k]) for k in range(len(names))])


STABLE_RISE, STABLE_SPEED = 0.5, 100.0     # change 6.12: centre of mass rise (m) and joint speed (rad/s, m/s) limits


def _full_mujoco(xml_path):
  """ change 6.12: the converted model as it is (nothing locked), without mesh geoms. """
  import mujoco
  spec = mujoco.MjSpec.from_file(xml_path)
  for g in list(spec.geoms):
    if g.type == mujoco.mjtGeom.mjGEOM_MESH:
      spec.delete(g)
  for mesh in list(spec.meshes):
    spec.delete(mesh)
  return spec.compile()


def _stable(m, pair_params):
  """ change 6.12: the whole converted model, released from its keyframe with these pair parameters (pair name ->
  (timeconst, dampratio, friction)) and no muscle excitation, stays finite for DURATION s, no joint moves faster than
  STABLE_SPEED and its centre of mass doesn't rise more than STABLE_RISE (a soft contact that is too stiff for the light
  bodies of the free model throws it into the air). """
  import mujoco
  for name, (tc, ratio, mu) in pair_params.items():
    k = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_PAIR, name)
    m.pair_solref[k] = [tc, ratio]
    m.pair_friction[k, :2] = mu
  d = mujoco.MjData(m)
  if m.nkey > 0:
    mujoco.mj_resetDataKeyframe(m, d, 0)
  mujoco.mj_forward(m, d)
  up = -np.asarray(m.opt.gravity) / max(np.linalg.norm(m.opt.gravity), 1e-12)
  h0 = float(up @ d.subtree_com[0])
  while d.time < DURATION:
    mujoco.mj_step(m, d)
    if not (np.all(np.isfinite(d.qpos)) and np.abs(d.qvel).max(initial=0.0) <= STABLE_SPEED
            and float(up @ d.subtree_com[0]) - h0 <= STABLE_RISE):
      return False
  return True


def calibrate(xml_path, osim_path, groups, output_folder):
  """ Fit solref (time constant, damping ratio) and friction per group; return {group: parameters} or None.

  :param groups: {group name: {"pairs": [MuJoCo pair names], "geometry_pairs": [(OpenSim geometry, geometry)],
    "static_friction": ..., "dynamic_friction": ...}}
  """
  import mujoco
  from scipy.optimize import minimize
  all_geometry_pairs = [p for g in groups.values() for p in g["geometry_pairs"]]
  ref = _opensim_drops(osim_path, all_geometry_pairs)
  if ref is None:
    logger.warning("Contact calibration skipped: the model has no unlocked root translation that separates its "
                   "contacts, so it can't be dropped; pairs keep MuJoCo's default solref and OpenSim's static friction")
    return None
  names, times = ref["coordinates"], ref["times"]
  trans = np.array(ref["translational"])
  m = _rigid_mujoco(xml_path, set(names), ref["pose"])
  d = mujoco.MjData(m)
  pair_ids = {g: [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_PAIR, p) for p in v["pairs"]] for g, v in groups.items()}
  order = sorted(groups)
  min_tc = 2 * m.opt.timestep

  def apply(x):
    for i, g in enumerate(order):
      tc, ratio, mu = np.exp(x[3 * i:3 * i + 3])
      for k in pair_ids[g]:
        m.pair_solref[k] = [tc, ratio]
        m.pair_friction[k, :2] = mu

  def cost(x):
    tcs = np.exp(x[0::3])
    if np.any(tcs < min_tc) or np.any(np.exp(x[1::3]) > 10) or np.any(np.exp(x[2::3]) > 3):
      return 1e6
    apply(x)
    total = 0.0
    for q0, traj in ref["runs"]:
      sim = _mujoco_run(m, d, names, q0, times)
      if sim is None:
        return 1e6
      total += np.sqrt(np.mean(np.sum((sim[:, trans] - traj[:, trans]) ** 2, axis=1)))
    return total / len(ref["runs"])

  # starts: a small grid around MuJoCo's defaults and OpenSim's two friction coefficients, then the best few refined
  candidates = []
  for tc in (max(min_tc, 0.005), 0.01, 0.02):
    for ratio in (0.3, 1.0):
      for which in ("static_friction", "dynamic_friction"):
        x = np.log(np.concatenate([[tc, ratio, max(groups[g][which], 0.05)] for g in order]))
        candidates.append((cost(x), x))
  candidates.sort(key=lambda c: c[0])
  fits = [minimize(cost, x0, method="Nelder-Mead", options={"xatol": 1e-3, "fatol": 1e-4, "maxiter": 400})
          for c0, x0 in candidates[:3]]
  # change 6.12: the best of the refined fits and the start grid that keeps the whole (unlocked) model stable
  m_full = _full_mujoco(xml_path)

  def params(x):
    out = {}
    for i, g in enumerate(order):
      tc, ratio, mu = (float(v) for v in np.exp(x[3 * i:3 * i + 3]))
      out.update({p: (tc, ratio, mu) for p in groups[g]["pairs"]})
    return out
  ranked = sorted([(r.fun, r.x) for r in fits] + candidates, key=lambda c: c[0])
  rejected = 0
  best = None
  for fun, x in ranked:
    if fun < 1e6 and _stable(m_full, params(x)):
      best = (fun, x)
      break
    rejected += 1
  if best is None:
    logger.error("Contact calibration: no candidate keeps the whole model stable; pairs keep MuJoCo's default solref "
                 "and OpenSim's static friction")
    with open(os.path.join(output_folder, "contact_calibration.json"), "w") as f:
      json.dump({"failed": "no candidate keeps the whole model stable", "candidates": len(ranked)}, f, indent=1)
    return None
  if rejected:
    logger.info(f"Contact calibration: {rejected} better-fitting candidate(s) rejected, the whole model wasn't stable "
                "with them (change 6.12)")
  best_fun, best_x = best
  apply(best_x)
  result = {}
  for i, g in enumerate(order):
    tc, ratio, mu = (float(v) for v in np.exp(best_x[3 * i:3 * i + 3]))
    result[g] = {"timeconst": tc, "dampratio": ratio, "friction": mu, "pairs": groups[g]["pairs"]}
  report = {"method": "rigid-model drops, multi-start Nelder-Mead (contact_calibration.py)", "drops_m": list(DROPS),
            "duration_s": DURATION, "root": ref["root"], "coordinates": names,
            "root_position_rms_mm": 1000 * float(best_fun), "start_rms_mm": 1000 * float(candidates[0][0]),
            "rejected_unstable": rejected,
            "groups": result}
  with open(os.path.join(output_folder, "contact_calibration.json"), "w") as f:
    json.dump(report, f, indent=1)
  logger.info(f"Contact calibration: root position RMS over the calibration drops {1000 * best_fun:.2f} mm; "
              + "; ".join(f"{g}: timeconst {v['timeconst']:.4g} s, dampratio {v['dampratio']:.3g}, friction "
                          f"{v['friction']:.3g}" for g, v in result.items()))
  return result
