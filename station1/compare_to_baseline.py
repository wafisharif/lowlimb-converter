"""Station 1a gates P3-P5: ported conversion vs the Station 0 baseline (see station1/GATES.md).
Usage: python compare_to_baseline.py <ported_model_dir> <baseline_model_dir>
Both dirs must contain *_cvt3.xml (loadability in current MuJoCo is gate P2, checked by check_counts.py) and metrics_moment_arms.csv / metrics_forces.csv
(from baseline/extract_metrics.py). Writes <ported_model_dir>/vs_baseline.json; exit code 0 only if all pass.
"""
import sys, os, glob, json, csv
from lxml import etree

ported, base = sys.argv[1:3]
MA_TOL_M = 1e-4        # P4
F_MEAN_FACTOR = 1.05   # P5
F_MUSCLE_TOL = 0.02    # P5


def structure(xml_path):
    """Joint names, actuator names and equality-constraint count, read from the MJCF XML.
    Only instances inside <worldbody>/<actuator>/<equality> count (not <default> classes).
    Reading XML avoids compiling, so the baseline (MuJoCo 2.3.7 syntax, meshes not in git) can be compared."""
    root = etree.parse(xml_path).getroot()
    joints = sorted(j.get("name") for wb in root.iter("worldbody") for j in wb.iter("joint", "freejoint"))
    acts = sorted(a.get("name") for blk in root.iter("actuator") for a in blk if isinstance(a.tag, str))
    neq = sum(1 for blk in root.iter("equality") for e in blk if isinstance(e.tag, str))
    return joints, acts, neq


def rows(path, key):
    with open(path) as f:
        return {key(r): r for r in csv.DictReader(f)}


rep = {}
ja, aa, na = structure(glob.glob(os.path.join(ported, "*_cvt3.xml"))[0])
jb, ab, nb = structure(glob.glob(os.path.join(base, "*_cvt3.xml"))[0])
p3 = {"joints_equal": ja == jb, "actuators_equal": aa == ab, "n_joints": len(ja), "n_actuators": len(aa),
      "neq_ported": na, "neq_baseline": nb,
      "joints_only_in_ported": sorted(set(ja) - set(jb)), "joints_only_in_baseline": sorted(set(jb) - set(ja))}
p3["pass"] = p3["joints_equal"] and p3["actuators_equal"] and na == nb
rep["P3_same_structure"] = p3

ma_p = rows(os.path.join(ported, "metrics_moment_arms.csv"), lambda r: (r["muscle"], r["joints"]))
ma_b = rows(os.path.join(base, "metrics_moment_arms.csv"), lambda r: (r["muscle"], r["joints"]))
d_ma = {f"{k[0]}|{k[1]}": float(ma_p[k]["rms_opt_m"]) - float(ma_b[k]["rms_opt_m"]) for k in ma_p.keys() & ma_b.keys()}
p4 = {"same_groups": set(ma_p) == set(ma_b), "n": len(d_ma),
      "max_increase_m": max(d_ma.values()) if d_ma else None,
      "worst": max(d_ma, key=d_ma.get) if d_ma else None,
      "max_decrease_m": min(d_ma.values()) if d_ma else None,
      "n_worse_than_tol": sum(v > MA_TOL_M for v in d_ma.values()),
      "per_group_ported_minus_baseline_m": d_ma}
p4["pass"] = p4["same_groups"] and bool(d_ma) and p4["n_worse_than_tol"] == 0
rep["P4_moment_arms"] = p4

f_p = rows(os.path.join(ported, "metrics_forces.csv"), lambda r: r["muscle"])
f_b = rows(os.path.join(base, "metrics_forces.csv"), lambda r: r["muscle"])
common = sorted(f_p.keys() & f_b.keys())
d_f = {m: float(f_p[m]["rms_opt_frac_fmax"]) - float(f_b[m]["rms_opt_frac_fmax"]) for m in common}
mean_p = sum(float(f_p[m]["rms_opt_frac_fmax"]) for m in common) / max(len(common), 1)
mean_b = sum(float(f_b[m]["rms_opt_frac_fmax"]) for m in common) / max(len(common), 1)
p5 = {"same_muscles": set(f_p) == set(f_b), "n": len(common), "mean_ported": mean_p, "mean_baseline": mean_b,
      "mean_ratio": mean_p / mean_b if mean_b else None,
      "max_increase": max(d_f.values()) if d_f else None, "worst": max(d_f, key=d_f.get) if d_f else None,
      "max_decrease": min(d_f.values()) if d_f else None,
      "per_muscle_ported_minus_baseline": d_f}
p5["pass"] = (p5["same_muscles"] and bool(d_f) and mean_p <= F_MEAN_FACTOR * mean_b
              and p5["max_increase"] <= F_MUSCLE_TOL)
rep["P5_forces"] = p5

with open(os.path.join(ported, "vs_baseline.json"), "w") as f:
    json.dump(rep, f, indent=2)
ok = True
for k, v in rep.items():
    ok &= v["pass"]
    short = {kk: vv for kk, vv in v.items() if not kk.startswith("per_") and kk != "pass"}
    print(f"{k}: {'PASS' if v['pass'] else 'FAIL'}  {short}")
sys.exit(0 if ok else 1)
