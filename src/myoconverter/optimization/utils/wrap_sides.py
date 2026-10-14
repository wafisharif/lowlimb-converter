# Added by lowlimb-converter (2026-10-13); not part of MyoConverter. Apache License 2.0.
""" Keep a wrap's side site only where it brings the muscle path closer to OpenSim's (Station 3, change 3.5).

OpenSim restricts a wrap object to one side with its `quadrant` (e.g. "-y"). MyoConverter gives every wrap over a
constrained object a MuJoCo side site, and Step 2 moves those sites to fit OpenSim's moment arms. A side site makes
MuJoCo wrap on that side whenever the straight segment crosses the object, even when that means going most of the
way around it; OpenSim, whose path is the shortest one that respects the quadrant, doesn't. In Rajagopal 2016 the
iliacus and psoas, which wrap over the pelvic brim (quadrant -y), came out 130-180 mm too long in about 0.3% of the
Station 2 poses (hip extended and strongly abducted or adducted), with or without Step 2; without their side sites
their lengths match OpenSim's within 0.03 mm in every pose. Gluteus maximus 1 (quadrant over the greater trochanter)
needs its side site: without it the path is 40-50 mm off.

So after Step 2, for every muscle whose path has a wrap with a side site: OpenSim's muscle length is sampled in
N_POSES poses (the coordinates that change the muscle's length uniformly over their ranges, everything else at its
default; seed SEED), and each side site in turn is dropped if the MuJoCo path without it is closer to OpenSim's (RMS
length error). Models without side sites (the gait models) are not touched.
"""
import os

import mujoco
import numpy as np
from loguru import logger
from lxml import etree

N_POSES = 400
SEED = 3


def _opensim_lengths(osim_model, state, muscle_name):
  """ OpenSim's muscle length in N_POSES poses over the coordinates that change it.

  :return: list of coordinate-value dicts (unlocked coordinates), lengths (m)
  """
  from myoconverter.xml.path_points.utils import _set_coordinates
  cset = osim_model.getCoordinateSet()
  muscle = osim_model.getMuscles().get(muscle_name)
  free = [cset.get(i).getName() for i in range(cset.getSize()) if not cset.get(i).getLocked(state)]
  defaults = {n: cset.get(n).getDefaultValue() for n in free}
  _set_coordinates(osim_model, state, defaults)
  base = muscle.getLength(state)
  relevant = []
  for n in free:
    for dq in (0.05, -0.05):
      _set_coordinates(osim_model, state, {**defaults, n: defaults[n] + dq})
      if abs(muscle.getLength(state) - base) > 1e-9:
        relevant.append(n)
        break
  lo = np.array([cset.get(n).getRangeMin() for n in relevant])
  hi = np.array([cset.get(n).getRangeMax() for n in relevant])
  rng = np.random.default_rng(SEED)
  poses = [dict(defaults, **dict(zip(relevant, r))) for r in rng.uniform(lo, hi, size=(N_POSES, len(relevant)))]
  lengths = np.zeros(len(poses))
  for p, pose in enumerate(poses):
    _set_coordinates(osim_model, state, pose)
    lengths[p] = muscle.getLength(state)
  _set_coordinates(osim_model, state, defaults)
  return poses, lengths


def _mujoco_lengths(xml_text, folder, actuator, poses):
  """ MuJoCo tendon length of `actuator` in the poses (dependent joints from their joint equalities). """
  from myoconverter.optimization.utils.force_refit import _joint_plan, _solve_qpos
  path = os.path.join(folder, ".wrap_sides_candidate.xml")
  with open(path, "w") as f:
    f.write(xml_text)
  try:
    m = mujoco.MjModel.from_xml_path(path)
  finally:
    os.remove(path)
  d = mujoco.MjData(m)
  aid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, actuator)
  driven = _joint_plan(m)
  out = np.zeros(len(poses))
  for p, pose in enumerate(poses):
    d.qpos[:] = _solve_qpos(m, driven, pose)
    mujoco.mj_fwdPosition(m, d)
    out[p] = d.actuator_length[aid]
  return out


def choose_side_sites(mjc_model_path, osim_model, muscle_list):
  """ Drop the side sites that don't bring a muscle's path closer to OpenSim's (module docstring). The MuJoCo file is
  rewritten only if a side site is dropped.

  :param mjc_model_path: the cvt2 model (rewritten in place)
  :param osim_model: the OpenSim model (opensim.Model)
  :param muscle_list: the muscles to consider
  :return: list of per-muscle reports
  """
  tree = etree.parse(mjc_model_path)
  root = tree.getroot()
  candidates = []
  for muscle in muscle_list:
    act = root.find(f"actuator/*[@name='{muscle}']")
    tendon = root.find(f"tendon/spatial[@name='{act.get('tendon')}']") if act is not None else None
    if tendon is not None and tendon.find("geom[@sidesite]") is not None:
      candidates.append((muscle, tendon))
  if not candidates:
    return []

  state = osim_model.initSystem()
  folder = os.path.dirname(os.path.abspath(mjc_model_path))
  reports, changed = [], False
  for muscle, tendon in candidates:
    poses, osim_len = _opensim_lengths(osim_model, state, muscle)

    def rms():
      text = etree.tostring(root, encoding="unicode")
      return float(np.sqrt(np.mean((_mujoco_lengths(text, folder, muscle, poses) - osim_len) ** 2)))

    before = current = rms()
    dropped = []
    for geom in tendon.findall("geom[@sidesite]"):
      site = geom.attrib.pop("sidesite")
      without = rms()
      if without < current:
        current = without
        dropped.append(site)
        changed = True
      else:
        geom.set("sidesite", site)
    reports.append({"muscle": muscle, "rms_before_mm": 1000 * before, "rms_after_mm": 1000 * current,
                    "dropped": dropped})
    logger.info(f"{muscle}: side sites " + (f"dropped {dropped}" if dropped else "kept")
                + f"; length RMS vs OpenSim {1000 * before:.3f} -> {1000 * current:.3f} mm over {N_POSES} poses"
                + " (lowlimb-converter change 3.5)")
  if changed:
    tree.write(mjc_model_path, pretty_print=True)
  return reports
