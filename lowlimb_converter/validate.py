"""Validation report for a converted model: how closely the MuJoCo model matches its OpenSim original.

Runs the project's own checks (lowlimb_converter.checks, the code behind Stations 2-4) on any OpenSim model and
its MuJoCo conversion:

  kinematics  every body's position and orientation in ~1000-2000 poses (Station 2)
  paths       every muscle's length, and its moment arm about every coordinate it crosses (Station 3)
  forces      every muscle's static force at activation 0, 0.25, 0.5 and 1 (Station 4)
  elements    everything else the converter carries over, as far as the model has it (Station 6): every body's mass
              properties; CoordinateLimitForces (generalized force over a sweep); PathActuators and PathSprings
              (length, generalized force); contact geometry (signed distances) and, for a model that can be
              dropped, the contact drops held out from the converter's calibration
  walking     a walking trial you supply (OpenSim IK coordinates + ground reaction forces): every muscle's length and
              moment arms along it, and the joint moments needed to walk with the measured forces (Station 5);
              optionally an open-loop replay of recorded muscle excitations (Station 5 level 3, reported only)

Writes report.json (summary), report.md (the same, readable), one detailed JSON per level and the OpenSim
reference data (.npz) into the output folder.

Each level first checks that the comparison itself is valid (the same names on both sides, poses held exactly, and
so on: gates K0, M0, F0 in the station GATES.md files). Kinematics also has absolute limits (1 mm, 0.5 deg: gates
K1-K2). Muscle paths and forces have no absolute limits in the project's gates (they are judged against
MyoConverter's result for the same model), so here they are reported, with the project's own results for the two
gait models as a point of reference.
"""
import json
import os
import time

import numpy as np

from lowlimb_converter import __version__

LEVELS = ("kinematics", "paths", "forces", "elements")   # the default: needs only the two models
ALL_LEVELS = LEVELS + ("walking",)                  # walking needs a trial (see validate())
WALKING_MOMENT_LIMIT = 0.02   # N*m/kg: Station 5's absolute part of gate W3 (hip, knee, ankle, lumbar)

# Our results for the two gait models (Stations 2-4, see the station FINDINGS.md files), shown as reference points.
REFERENCE_RESULTS = {
    "kinematics": "gait models: worst body 0.22 mm, 0.0000 deg",
    "paths": "gait models: mean moment-arm RMS 0.33-0.91 mm, worst pair 3.4-4.7 mm; mean length RMS 0.46-0.59 mm",
    "forces": "gait models: mean RMS 1.2-2.9% Fmax passive, 3.0-3.5% at full activation",
    "elements": "Station 6 (opensim-models): mass properties exact; limit forces within 1% of peak; path "
                "actuators/springs within 1 mm and 2%; contact geometry within 1e-6 m",
    "walking": "gait models (OpenSim's example trials): mean length RMS 0.05-0.13 mm, moment arm 0.11-0.51 mm, "
               "joint moments 0.0027-0.0048 N*m/kg",
}


def _save_npz(path, data):
    np.savez_compressed(path, **data)


def _dump(path, data):
    with open(path, "w") as f:
        json.dump(data, f, indent=1)


def summarize_kinematics(r):
    if r["K0"] != "PASS":
        return {"valid": False, "problems": r["K0_problems"], "status": "invalid"}
    return {"valid": True, "status": "pass" if r["K1"] == r["K2"] == "PASS" else "fail",
            "poses": r["poses"], "bodies": r["bodies_compared"],
            "worst_position_mm": r["pos_worst_mm"], "worst_orientation_deg": r["rot_worst_deg"],
            "position_rms_mm": r["pos_rms_all_mm"], "limits": {"position_mm": 1.0, "orientation_deg": 0.5},
            "bodies_over_position_limit": r["bodies_failing_K1"],
            "bodies_over_orientation_limit": r["bodies_failing_K2"]}


def summarize_paths(r):
    if r["M0"] != "PASS":
        return {"valid": False, "problems": r["M0_problems"], "status": "invalid"}
    s = r["summary"]
    lengths = {m: v["length_rms_mm"] for m, v in r["per_muscle"].items()}
    worst_len = max(lengths, key=lengths.get)
    return {"valid": True, "status": "reported", "poses": r["poses"], "muscles": r["muscles"],
            "pairs_scored": r["pairs_scored"], "moment_arm_rms_mm_mean": s["ma_rms_mm_mean"],
            "moment_arm_rms_mm_median": s["ma_rms_mm_median"], "worst_pair": s["ma_rms_mm_worst"],
            "pairs_within_2mm": s["pairs_within_2mm"], "pairs_within_5pct_of_peak": s["pairs_within_5pct_peak"],
            "length_rms_mm_mean": float(np.mean(list(lengths.values()))),
            "worst_length": {"muscle": worst_len, "rms_mm": lengths[worst_len]},
            "opensim_moment_arm": r.get("opensim_moment_arm", "analytic")}


def summarize_forces(r):
    if r["F0"] != "PASS":
        return {"valid": False, "problems": r["F0_problems"], "status": "invalid"}
    return {"valid": True, "status": "reported", "poses": r["poses"], "muscles": r["muscles"],
            "by_activation": r["summary"], "samples_without_opensim_equilibrium": r["excluded_samples"]}


def _kinematics(osim_path, xml_path, out_dir):
    from lowlimb_converter.checks import mujoco_compare, osim_reference
    ref = osim_reference.reference(osim_path)
    _save_npz(os.path.join(out_dir, "kinematics_reference.npz"), ref)
    r = mujoco_compare.compare(xml_path, ref, keep_arrays=False)
    _dump(os.path.join(out_dir, "kinematics.json"), r)
    return summarize_kinematics(r)


def _paths(osim_path, xml_path, out_dir):
    from lowlimb_converter.checks import mujoco_paths, osim_paths
    # wrapped paths: OpenSim's computeMomentArm isn't the derivative of its own length there (station3/GATES.md,
    # amendment 2026-10-12), so moment arms are -dL/dq on both sides
    definition = "length-derivative" if osim_paths.has_wrapped_paths(osim_path) else "analytic"
    ref = osim_paths.reference(osim_path, definition)
    _save_npz(os.path.join(out_dir, "paths_reference.npz"), ref)
    r = mujoco_paths.evaluate(xml_path, ref)
    _dump(os.path.join(out_dir, "paths.json"), r)
    return summarize_paths(r)


def _forces(osim_path, xml_path, out_dir):
    from lowlimb_converter.checks import mujoco_forces, osim_forces
    ref = osim_forces.reference(osim_path)
    _save_npz(os.path.join(out_dir, "forces_reference.npz"), ref)
    r = mujoco_forces.evaluate(xml_path, ref)
    _dump(os.path.join(out_dir, "forces.json"), r)
    return summarize_forces(r)


def summarize_walking(r, replay=None):
    if r["W0"] != "PASS":
        return {"valid": False, "problems": r["W0_problems"], "status": "invalid"}
    s = r["summary"]
    lengths = {m: v["length_rms_mm"] for m, v in r["per_muscle"].items()}
    worst_len = max(lengths, key=lengths.get)
    gated = {c: v for c, v in r["per_coordinate"].items() if v.get("gated")}
    over = sorted(c for c, v in gated.items() if v["rms"] > WALKING_MOMENT_LIMIT)
    out = {"valid": True, "status": "pass" if not over else "fail", "frames": r["frames"],
           "time_range_s": r["time_range"], "muscles": r["muscles"], "pairs_scored": r["pairs_scored"],
           "length_rms_mm_mean": s["length_rms_mm_mean"],
           "worst_length": {"muscle": worst_len, "rms_mm": lengths[worst_len]},
           "moment_arm_rms_mm_mean": s["ma_rms_mm_mean"], "worst_pair": s["ma_rms_mm_worst"],
           "joint_moment_rms_mean": s["moment_rms_mean_gated"], "worst_joint_moment": s["moment_rms_worst_gated"],
           "joint_moment_limit": WALKING_MOMENT_LIMIT, "joint_moments_over_limit": over,
           "joint_moments": {c: {"rms": v["rms"], "unit": v["unit"], "rms_pct_peak": v["rms_pct_peak"]}
                             for c, v in gated.items()}}
    if replay is not None:
        out["replay"] = {k: {"time_to_5deg_s": v["time_to_5deg_s"],
                             "first_coordinate": v["first_coordinate_over_5deg"],
                             "compared_until_s": v["compared_until_s"], "mujoco_status": v["mujoco_status"]}
                         for k, v in replay.items() if k in ("as_converted", "fine_integration")}
        out["replay"]["opensim_status"] = replay["opensim_status"]
    return out


def _walking(osim_path, xml_path, out_dir, walking):
    from lowlimb_converter.checks import mujoco_gait, osim_gait, osim_paths
    definition = "length-derivative" if osim_paths.has_wrapped_paths(osim_path) else "analytic"
    ref = osim_gait.reference(osim_path, walking["ik"], walking["grf"], walking.get("start"), walking.get("end"),
                              moment_arm=definition)
    _save_npz(os.path.join(out_dir, "walking_reference.npz"), ref)
    r = mujoco_gait.evaluate(xml_path, ref)
    _dump(os.path.join(out_dir, "walking.json"), r)
    replay = None
    if walking.get("controls"):
        from lowlimb_converter.checks import mujoco_replay, osim_replay
        rref = osim_replay.reference(osim_path, walking["ik"], walking["grf"], walking["controls"])
        _save_npz(os.path.join(out_dir, "replay_reference.npz"), rref)
        replay = mujoco_replay.evaluate(xml_path, rref)
        _dump(os.path.join(out_dir, "replay.json"), replay)
    return summarize_walking(r, replay)


def _elements(osim_path, xml_path, out_dir):
    """Station 6's checks, each where the model has the element (lowlimb_converter.checks)."""
    from lowlimb_converter.checks import (contact_dynamics, contact_geometry, limit_forces, mass_properties,
                                          path_forces)
    from lowlimb_converter.support import read_osim
    root = read_osim(osim_path).getroot().find("Model")
    present = lambda *tags: any(root.find(f"ForceSet/objects/{t}") is not None for t in tags)
    checks = {"mass_properties": (mass_properties.evaluate, "L3", True),
              "limit_forces": (limit_forces.evaluate, "L4", present("CoordinateLimitForce")),
              "path_forces": (path_forces.evaluate, "L5", present("PathActuator", "PathSpring")),
              "contact_geometry": (contact_geometry.evaluate, "L6", present("HuntCrossleyForce"))}
    out = {"valid": True, "checks": {}}
    for name, (fn, key, applies) in checks.items():
        if not applies:
            continue
        r = fn(osim_path, xml_path)
        _dump(os.path.join(out_dir, f"{name}.json"), r)
        out["checks"][name] = {"status": r[key].lower(), "problems": r.get("problems", [])}
        if name == "mass_properties":
            out["checks"][name].update({"bodies": len(r["bodies"]), "failing": r["failing"],
                                        "invalid_opensim_inertia": r["invalid_opensim_inertia"]})
        elif name == "limit_forces":
            out["checks"][name]["forces"] = {n: round(v["max_error_pct_peak"], 4) for n, v in r["forces"].items()}
        elif name == "path_forces":
            out["checks"][name]["forces"] = {n: {"length_rms_mm": v["length_rms_mm"],
                                                 "worst_force_pct": max(x["rms_pct_peak"] for x in
                                                                        v["generalized_force"].values())}
                                             for n, v in r["forces"].items()}
        elif name == "contact_geometry":
            out["checks"][name]["pairs"] = len(r["pairs"])
            out["checks"][name]["max_error_m"] = max((v["max_abs_error_m"] for v in r["pairs"].values()), default=0)
    if present("HuntCrossleyForce"):
        try:
            r = contact_dynamics.evaluate(osim_path, xml_path, ("held-out",))
            _dump(os.path.join(out_dir, "contact_drops.json"), r)
            out["checks"]["contact_drops"] = {
                "status": "pass" if r["C1"] == "PASS" and r["C2"] == "PASS" else "fail",
                "runs": {n: {"position_rms_mm": v["position_rms_mm"], "rotation_rms_deg": v["rotation_rms_deg"],
                             "mujoco_default_position_rms_mm": v["mujoco_default_position_rms_mm"]}
                         for n, v in r["runs"].items()}}
        except NotImplementedError as e:          # e.g. the root can't translate: nothing to drop
            out["checks"]["contact_drops"] = {"status": "not applicable", "reason": str(e)}
    statuses = [c["status"] for c in out["checks"].values()]
    out["status"] = "fail" if "fail" in statuses or "invalid" in statuses else "pass"
    return out


RUNNERS = {"kinematics": _kinematics, "paths": _paths, "forces": _forces, "elements": _elements}


def validate(osim_path, xml_path, out_dir, levels=LEVELS, progress=print, walking=None):
    """Compare `xml_path` (MuJoCo) with `osim_path` (OpenSim); write the report into `out_dir`; return the summary.

    levels: any of "kinematics", "paths", "forces", "walking". Kinematics only needs the bodies and joints, so it can
    be run on the Step 1 output (_cvt1.xml); the others need the muscles (use the final _cvt3.xml).
    walking (needed for the "walking" level): {"ik": OpenSim IK .mot/.sto of the trial, "grf": ExternalLoads .xml of
    its ground reaction forces, optional "start"/"end" (s), optional "controls": a .sto of muscle excitations (e.g.
    OpenSim CMC's *_controls.sto) for the open-loop replay}. The model is replayed as the trial says (OpenSim's
    pipelines record data on a scaled subject; replaying it on the generic model is fine for this comparison, since
    both simulators get exactly the same inputs).
    """
    unknown = set(levels) - set(ALL_LEVELS)
    if unknown:
        raise ValueError(f"unknown levels {sorted(unknown)}; choose from {ALL_LEVELS}")
    if "walking" in levels:
        if not walking or not walking.get("ik") or not walking.get("grf"):
            raise ValueError("the walking level needs a trial: IK coordinates and ground reaction forces")
        for k in ("ik", "grf", "controls"):
            if walking.get(k) and not os.path.isfile(walking[k]):
                raise FileNotFoundError(walking[k])
    for p in (osim_path, xml_path):
        if not os.path.isfile(p):
            raise FileNotFoundError(p)
    os.makedirs(out_dir, exist_ok=True)
    report = {"tool": "lowlimb-converter", "version": __version__,
              "opensim_model": os.path.abspath(osim_path), "mujoco_model": os.path.abspath(xml_path),
              "levels": {}}
    for level in ALL_LEVELS:
        if level not in levels:
            continue
        progress(f"[{level}] comparing with OpenSim ...")
        t = time.time()
        if level == "walking":
            report["walking_trial"] = {k: (os.path.abspath(v) if isinstance(v, str) else v)
                                       for k, v in walking.items() if v is not None}
            report["levels"][level] = _walking(osim_path, xml_path, out_dir, walking)
        else:
            report["levels"][level] = RUNNERS[level](osim_path, xml_path, out_dir)
        report["levels"][level]["seconds"] = round(time.time() - t, 1)
        progress(f"[{level}] {report['levels'][level]['status']} ({report['levels'][level]['seconds']} s)")
    report["ok"] = all(v["status"] in ("pass", "reported") for v in report["levels"].values())
    _dump(os.path.join(out_dir, "report.json"), report)
    with open(os.path.join(out_dir, "report.md"), "w") as f:
        f.write(to_markdown(report))
    return report


def _invalid(lines, v):
    lines += ["The comparison itself is not valid, so no numbers are reported:", ""]
    lines += [f"- {p}" for p in v["problems"][:20]]
    if len(v["problems"]) > 20:
        lines.append(f"- ... {len(v['problems']) - 20} more")
    lines.append("")


def to_markdown(report):
    lines = ["# Validation report", "",
             f"- OpenSim model: `{report['opensim_model']}`",
             f"- MuJoCo model: `{report['mujoco_model']}`",
             f"- lowlimb-converter {report['version']}", ""]
    lv = report["levels"]
    if "kinematics" in lv:
        v = lv["kinematics"]
        lines += ["## Body kinematics", ""]
        if not v["valid"]:
            _invalid(lines, v)
        else:
            wp, wr = v["worst_position_mm"], v["worst_orientation_deg"]
            lines += [f"**{v['status'].upper()}** (limits: {v['limits']['position_mm']} mm, "
                      f"{v['limits']['orientation_deg']} deg in every pose)", "",
                      f"- {v['bodies']} bodies in {v['poses']} poses",
                      f"- worst position error: {wp['value']:.4f} mm ({wp['body']}, pose {wp['pose_label']})",
                      f"- worst orientation error: {wr['value']:.4f} deg ({wr['body']}, pose {wr['pose_label']})",
                      f"- RMS position error, all bodies and poses: {v['position_rms_mm']:.4f} mm"]
            if v["bodies_over_position_limit"] or v["bodies_over_orientation_limit"]:
                lines.append(f"- over the limits: {sorted(set(v['bodies_over_position_limit']) | set(v['bodies_over_orientation_limit']))}")
            lines += [f"- reference: {REFERENCE_RESULTS['kinematics']}", ""]
    if "paths" in lv:
        v = lv["paths"]
        lines += ["## Muscle paths (lengths and moment arms)", ""]
        if not v["valid"]:
            _invalid(lines, v)
        else:
            w = v["worst_pair"]
            lines += [f"- {v['muscles']} muscles, {v['pairs_scored']} muscle-coordinate pairs scored "
                      f"(peak moment arm >= 1 mm), {v['poses']} poses",
                      f"- moment-arm RMS error: mean {v['moment_arm_rms_mm_mean']:.2f} mm, "
                      f"median {v['moment_arm_rms_mm_median']:.2f} mm",
                      f"- worst pair: {w['muscle']} / {w['coordinate']}: {w['rms_mm']:.2f} mm RMS "
                      f"({w['rms_pct_peak']:.1f}% of its peak), largest single error {w['max_mm']:.1f} mm",
                      f"- pairs within 2 mm RMS: {v['pairs_within_2mm']} / {v['pairs_scored']}; "
                      f"within 5% of peak: {v['pairs_within_5pct_of_peak']}",
                      f"- muscle length RMS error: mean {v['length_rms_mm_mean']:.2f} mm, worst "
                      f"{v['worst_length']['muscle']} {v['worst_length']['rms_mm']:.2f} mm",
                      f"- reference: {REFERENCE_RESULTS['paths']}", ""]
    if "forces" in lv:
        v = lv["forces"]
        lines += ["## Static muscle forces", ""]
        if not v["valid"]:
            _invalid(lines, v)
        else:
            lines += [f"- {v['muscles']} muscles held still in {v['poses']} poses; error = RMS over poses, "
                      "as % of the muscle's max isometric force (Fmax)", "",
                      "| Activation | Mean RMS error | Median | Worst muscle | Muscles within 5% |",
                      "| --- | --- | --- | --- | --- |"]
            for key, s in v["by_activation"].items():
                lines.append(f"| {key[1:]} | {s['rms_pct_mean']:.2f}% | {s['rms_pct_median']:.2f}% | "
                             f"{s['worst_muscle']} {s['worst_rms_pct']:.1f}% | {s['muscles_within_5pct']} / {v['muscles']} |")
            lines += ["", f"- samples left out because OpenSim has no static equilibrium there: "
                      f"{v['samples_without_opensim_equilibrium']}",
                      f"- reference: {REFERENCE_RESULTS['forces']}", ""]
    if "elements" in lv:
        v = lv["elements"]
        lines += ["## Other elements (Station 6 checks)", "", f"**{v['status'].upper()}**", ""]
        c = v["checks"]
        if "mass_properties" in c:
            x = c["mass_properties"]
            lines.append(f"- mass properties: {x['status']} ({x['bodies']} bodies; mass, centre of mass and inertia "
                         f"within 1e-9 relative" + (f"; off: {x['failing']}" if x["failing"] else "")
                         + (f"; OpenSim inertia invalid, not checked: {x['invalid_opensim_inertia']}"
                            if x["invalid_opensim_inertia"] else "") + ")")
        if "limit_forces" in c:
            x = c["limit_forces"]
            lines.append(f"- coordinate limit forces: {x['status']} (largest error, % of peak: {x['forces']}; "
                         "damping beyond the limits not converted)")
        if "path_forces" in c:
            x = c["path_forces"]
            lines.append(f"- path actuators and springs: {x['status']} ("
                         + "; ".join(f"{n}: length RMS {y['length_rms_mm']:.3f} mm, force {y['worst_force_pct']:.2f}%"
                                     for n, y in x["forces"].items()) + ")")
        if "contact_geometry" in c:
            x = c["contact_geometry"]
            lines.append(f"- contact geometry: {x['status']} ({x['pairs']} pairs, largest error "
                         f"{x['max_error_m']:.1e} m)")
        if "contact_drops" in c:
            x = c["contact_drops"]
            if x["status"] == "not applicable":
                lines.append(f"- contact drops: not applicable ({x['reason']})")
            else:
                lines.append(f"- contact drops (held out from the calibration): {x['status']} ("
                             + "; ".join(f"{n}: {y['position_rms_mm']:.1f} mm vs MuJoCo defaults "
                                         f"{y['mujoco_default_position_rms_mm']:.1f} mm" for n, y in x["runs"].items())
                             + ")")
        lines += [f"- reference: {REFERENCE_RESULTS['elements']}", ""]
    if "walking" in lv:
        v = lv["walking"]
        lines += ["## Walking", ""]
        if not v["valid"]:
            _invalid(lines, v)
        else:
            tr = report.get("walking_trial", {})
            w, wm = v["worst_pair"], v["worst_joint_moment"]
            lines += [f"**{v['status'].upper()}** (limit: every hip, knee, ankle and lumbar joint moment within "
                      f"{v['joint_moment_limit']} N*m/kg RMS)", "",
                      f"- trial: `{os.path.basename(tr.get('ik', ''))}` with `{os.path.basename(tr.get('grf', ''))}`, "
                      f"{v['frames']} frames, {round(v['time_range_s'][0], 2) + 0.0:.2f}-"
                      f"{round(v['time_range_s'][1], 2) + 0.0:.2f} s",
                      f"- muscle length RMS error: mean {v['length_rms_mm_mean']:.3f} mm, worst "
                      f"{v['worst_length']['muscle']} {v['worst_length']['rms_mm']:.3f} mm",
                      f"- moment-arm RMS error ({v['pairs_scored']} pairs): mean {v['moment_arm_rms_mm_mean']:.3f} mm, "
                      f"worst {w['muscle']} / {w['coordinate']} {w['rms_mm']:.2f} mm",
                      f"- joint moments needed to walk, RMS error: mean {v['joint_moment_rms_mean']:.4f} N*m/kg, worst "
                      f"{wm['coordinate']} {wm['rms']:.4f} ({wm['rms_pct_peak']:.1f}% of its peak)"]
            if v["joint_moments_over_limit"]:
                lines.append(f"- over the limit: {v['joint_moments_over_limit']}")
            if "replay" in v:
                apart = []
                for k, x in v["replay"].items():
                    if isinstance(x, dict):
                        t = x["time_to_5deg_s"]
                        apart.append(f"{'%.3f s' % t if t is not None else 'never'} ({k.replace('_', ' ')})")
                lines.append("- open-loop replay of the muscle excitations (reported only): first > 5 deg apart after "
                             + ", ".join(apart))
            lines += [f"- reference: {REFERENCE_RESULTS['walking']}", ""]
    lines += ["## Limits of this report", "",
              "- MuJoCo muscles have rigid tendons; OpenSim's are elastic. Static force can match closely, but how a "
              "muscle's force changes with speed (force-velocity) is not checked here.",
              ("- The walking level replays one trial; other movements may differ." if "walking" in lv else
               "- Only static poses are compared. Add a walking trial (--ik, --grf) for the walking level."), ""]
    return "\n".join(lines)
