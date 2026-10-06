"""Station 0 reproducibility check: our rerun vs the outputs upstream ships in models/mjc/.
Gates R1-R3 are defined in baseline/GATES.md.
Usage: python compare_to_upstream.py <our_model_dir> <shipped_metrics_dir> <shipped_cvt3.xml>
Writes <our_model_dir>/reproducibility.json and prints PASS/FAIL per gate.
"""
import sys, os, glob, json, csv
import mujoco

ours, shipped_dir, shipped_xml = sys.argv[1:4]
report = {}

def names(m, obj, n):
    return sorted(mujoco.mj_id2name(m, obj, i) for i in range(n))

# R1
a = mujoco.MjModel.from_xml_path(glob.glob(os.path.join(ours, "*_cvt3.xml"))[0])
b = mujoco.MjModel.from_xml_path(shipped_xml)
r1 = {"joints_equal": names(a, mujoco.mjtObj.mjOBJ_JOINT, a.njnt) == names(b, mujoco.mjtObj.mjOBJ_JOINT, b.njnt),
      "actuators_equal": names(a, mujoco.mjtObj.mjOBJ_ACTUATOR, a.nu) == names(b, mujoco.mjtObj.mjOBJ_ACTUATOR, b.nu),
      "neq_ours": int(a.neq), "neq_shipped": int(b.neq)}
r1["pass"] = r1["joints_equal"] and r1["actuators_equal"] and a.neq == b.neq
report["R1_structure"] = r1

# R2
def rows(path, key):
    with open(path) as f:
        return {key(r): r for r in csv.DictReader(f)}
ma_o = rows(os.path.join(ours, "metrics_moment_arms.csv"), lambda r: (r["muscle"], r["joints"]))
ma_s = rows(os.path.join(shipped_dir, "metrics_moment_arms.csv"), lambda r: (r["muscle"], r["joints"]))
diffs = {f"{k[0]}|{k[1]}": abs(float(ma_o[k]["rms_opt_m"]) - float(ma_s[k]["rms_opt_m"]))
         for k in ma_o.keys() & ma_s.keys()}
r2 = {"same_groups": set(ma_o) == set(ma_s), "n_groups": len(diffs),
      "max_abs_diff_m": max(diffs.values()) if diffs else None,
      "worst": max(diffs, key=diffs.get) if diffs else None,
      "missing_in_ours": sorted("|".join(k) for k in set(ma_s) - set(ma_o)),
      "extra_in_ours": sorted("|".join(k) for k in set(ma_o) - set(ma_s))}
r2["pass"] = r2["same_groups"] and bool(diffs) and r2["max_abs_diff_m"] <= 1e-6
report["R2_moment_arms"] = r2

# R3
f_o = rows(os.path.join(ours, "metrics_forces.csv"), lambda r: r["muscle"])
f_s = rows(os.path.join(shipped_dir, "metrics_forces.csv"), lambda r: r["muscle"])
common = sorted(f_o.keys() & f_s.keys())
per = {m: float(f_o[m]["rms_opt_frac_fmax"]) - float(f_s[m]["rms_opt_frac_fmax"]) for m in common}
mean_o = sum(float(f_o[m]["rms_opt_frac_fmax"]) for m in common) / max(len(common), 1)
mean_s = sum(float(f_s[m]["rms_opt_frac_fmax"]) for m in common) / max(len(common), 1)
r3 = {"same_muscles": set(f_o) == set(f_s), "n": len(common),
      "mean_ours": mean_o, "mean_shipped": mean_s,
      "mean_rel_diff": abs(mean_o - mean_s) / mean_s if mean_s else None,
      "max_abs_muscle_diff": max(abs(v) for v in per.values()) if per else None,
      "worst_muscle": max(per, key=lambda k: abs(per[k])) if per else None,
      "per_muscle_diff_ours_minus_shipped": per}
r3["pass"] = (r3["same_muscles"] and bool(per) and r3["mean_rel_diff"] <= 0.10
              and r3["max_abs_muscle_diff"] <= 0.05)
report["R3_forces"] = r3

with open(os.path.join(ours, "reproducibility.json"), "w") as f:
    json.dump(report, f, indent=2)
for k, v in report.items():
    extra = {kk: vv for kk, vv in v.items() if kk not in ("pass", "per_muscle_diff_ours_minus_shipped")}
    print(f"{k}: {'PASS' if v['pass'] else 'FAIL'}  {extra}")
