"""Validation report for a converted model: how closely the MuJoCo model matches its OpenSim original.

Runs the project's own checks (lowlimb_converter.checks, the code behind Stations 2-4) on any OpenSim model and
its MuJoCo conversion:

  kinematics  every body's position and orientation in ~1000-2000 poses (Station 2)
  paths       every muscle's length, and its moment arm about every coordinate it crosses (Station 3)
  forces      every muscle's static force at activation 0, 0.25, 0.5 and 1 (Station 4)

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

LEVELS = ("kinematics", "paths", "forces")

# Our results for the two gait models (Stations 2-4, see the station FINDINGS.md files), shown as reference points.
REFERENCE_RESULTS = {
    "kinematics": "gait models: worst body 0.22 mm, 0.0000 deg",
    "paths": "gait models: mean moment-arm RMS 0.33-0.91 mm, worst pair 3.4-4.7 mm; mean length RMS 0.46-0.59 mm",
    "forces": "gait models: mean RMS 1.2-2.9% Fmax passive, 3.0-3.5% at full activation",
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
            "worst_length": {"muscle": worst_len, "rms_mm": lengths[worst_len]}}


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
    ref = osim_paths.reference(osim_path)
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


RUNNERS = {"kinematics": _kinematics, "paths": _paths, "forces": _forces}


def validate(osim_path, xml_path, out_dir, levels=LEVELS, progress=print):
    """Compare `xml_path` (MuJoCo) with `osim_path` (OpenSim); write the report into `out_dir`; return the summary.

    levels: any of "kinematics", "paths", "forces". Kinematics only needs the bodies and joints, so it can be run on
    the Step 1 output (_cvt1.xml); paths and forces need the muscles (use the final _cvt3.xml).
    """
    unknown = set(levels) - set(LEVELS)
    if unknown:
        raise ValueError(f"unknown levels {sorted(unknown)}; choose from {LEVELS}")
    for p in (osim_path, xml_path):
        if not os.path.isfile(p):
            raise FileNotFoundError(p)
    os.makedirs(out_dir, exist_ok=True)
    report = {"tool": "lowlimb-converter", "version": __version__,
              "opensim_model": os.path.abspath(osim_path), "mujoco_model": os.path.abspath(xml_path),
              "levels": {}}
    for level in LEVELS:
        if level not in levels:
            continue
        progress(f"[{level}] comparing with OpenSim ...")
        t = time.time()
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
    lines += ["## Limits of this report", "",
              "- MuJoCo muscles have rigid tendons; OpenSim's are elastic. Static force can match closely, but how a "
              "muscle's force changes with speed (force-velocity) is not checked here.",
              "- Only static poses are compared. Walking-level checks (replayed gait, joint moments) are not part of "
              "this report yet.", ""]
    return "\n".join(lines)
