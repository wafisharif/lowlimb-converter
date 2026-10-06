"""Extract MyoConverter's own validation errors into plain tables (Station 0 baseline numbers).

Reads per-muscle pickles, not upstream's summary matrices. Those order rows by filesystem
glob order and copy one group's cost into several joint columns, which is easy to misread.

Definitions (checked against upstream code @ cadf380):
  Step 1 endpoint error  : O2MStep1.endPointsPlot. Per sample |dx|+|dy|+|dz| in m (an L1
                           distance, although upstream calls it "rms"). mean/std over endpoints x samples.
  Step 2 moment-arm error: UtilsLengthOpt.getMomentArmDiff. RMS over joint-angle samples of
                           (MA_osim - MA_mjc) in m (upstream writes osim + mjc because the two use
                           opposite sign conventions). One value per muscle per wrapping-coordinate
                           group. Also reported relative to that group's peak |MA_osim|.
  Step 3 force error     : UtilsForceOpt.getMuscleForceDiff. RMS of (F_osim - F_mjc)/Fmax over
                           sampled lengths and activations, as a fraction of max isometric force.
  "_org" = before that step's optimisation, "_opt" = after.

Usage: python extract_metrics.py <model_output_dir>
Writes <dir>/metrics.json, <dir>/metrics_moment_arms.csv, <dir>/metrics_forces.csv
"""
import sys, os, glob, json, pickle, csv
import numpy as np

SKIP = {"config.pkl", "overall_comp_momentarms.pkl", "overall_comp_muscleforces.pkl"}


def first_pickle(path):
    # upstream re-dumps into the same file opened 'rb+', so a file can hold >1 object;
    # the first one is the original
    with open(path, "rb") as f:
        return pickle.load(f)


def summary(vals):
    vals = np.asarray(vals, float)
    if not vals.size:
        return {"n": 0}
    return {"n": int(vals.size), "mean": float(vals.mean()), "std": float(vals.std()),
            "median": float(np.median(vals)), "max": float(vals.max())}


def extract(out):
    res = {}

    ep = os.path.join(out, "Step1_xmlConvert", "end_points", "end_point_error.pkl")
    if os.path.exists(ep):
        e = first_pickle(ep)
        res["step1_endpoint_L1_m"] = {"mean": float(e["mean"]), "std": float(e["std"])}

    ma_rows = []
    for p in sorted(glob.glob(os.path.join(out, "Step2_muscleKinematics", "*.pkl"))):
        if os.path.basename(p) in SKIP:
            continue
        d = first_pickle(p)
        for ij, joints in enumerate(d["wrapping_coordinates"]):
            r = d["opt_results"][ij]
            if isinstance(r, list):
                r = r[0] if r else None
            if not r:
                continue
            peak = float(np.max(np.abs(np.asarray(d["osim_ma_data"][ij], float))))
            ma_rows.append({
                "muscle": d["muscle_name"], "joints": "+".join(joints),
                "rms_org_m": float(r["cost_org"]), "rms_opt_m": float(r["cost_opt"]),
                "peak_abs_ma_osim_m": peak,
                "rel_opt_pct": 100 * float(r["cost_opt"]) / max(peak, 1e-3),
                "optimised": bool(len(r.get("par_opt", [])) > 0)})
    if ma_rows:
        res["step2_moment_arm_rms_m"] = {
            "org": summary([r["rms_org_m"] for r in ma_rows]),
            "opt": summary([r["rms_opt_m"] for r in ma_rows]),
            "opt_rel_pct": summary([r["rel_opt_pct"] for r in ma_rows]),
            "worst_opt": max(ma_rows, key=lambda x: x["rms_opt_m"]),
            "groups_optimised": sum(r["optimised"] for r in ma_rows),
            "groups_total": len(ma_rows)}
        with open(os.path.join(out, "metrics_moment_arms.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(ma_rows[0]))
            w.writeheader(); w.writerows(ma_rows)

    f_rows = []
    for p in sorted(glob.glob(os.path.join(out, "Step3_muscleKinetics", "*.pkl"))):
        if os.path.basename(p) in SKIP:
            continue
        d = first_pickle(p)
        r = d.get("opt_results")
        if not r:
            continue
        f_rows.append({
            "muscle": d["muscle_name"], "fmax_N": float(d["mtu_par_set"]["fmax"]),
            "rms_org_frac_fmax": float(r["cost_org"]), "rms_opt_frac_fmax": float(r["cost_opt"])})
    if f_rows:
        res["step3_force_rms_frac_fmax"] = {
            "org": summary([r["rms_org_frac_fmax"] for r in f_rows]),
            "opt": summary([r["rms_opt_frac_fmax"] for r in f_rows]),
            "worst_opt": max(f_rows, key=lambda x: x["rms_opt_frac_fmax"])}
        with open(os.path.join(out, "metrics_forces.csv"), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(f_rows[0]))
            w.writeheader(); w.writerows(f_rows)

    with open(os.path.join(out, "metrics.json"), "w") as f:
        json.dump(res, f, indent=2)
    return res


if __name__ == "__main__":
    print(json.dumps(extract(sys.argv[1]), indent=2))
