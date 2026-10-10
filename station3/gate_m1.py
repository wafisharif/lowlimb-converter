"""Station 3 gate M1: our moment-arm error is not worse than MyoConverter's, pair by pair.

Usage: python station3/gate_m1.py ours.json myoconverter.json out.json [reasons.json]
  reasons.json (optional): {"<muscle>|<coordinate>": "reason, and the test that shows it"} for pairs that miss.
Exit 0 if M1 passes (every scored pair within MyoConverter's RMS + 0.1 mm, or listed with a reason).
"""
import json
import sys

TOL_MM = 0.1


def main():
    ours, base = (json.load(open(p)) for p in sys.argv[1:3])
    out_path = sys.argv[3]
    reasons = json.load(open(sys.argv[4])) if len(sys.argv) > 4 else {}
    for r, name in ((ours, "ours"), (base, "MyoConverter")):
        if r.get("M0") != "PASS":
            print(f"M1 NOT EVALUATED: M0 failed for {name}")
            return 1
    b = {(q["muscle"], q["coordinate"]): q for q in base["pairs"]}
    rows, misses = [], []
    for q in ours["pairs"]:
        if not q["scored"]:
            continue
        key = (q["muscle"], q["coordinate"])
        if key not in b:
            misses.append((key, "pair missing from the MyoConverter column"))
            continue
        delta = q["rms_mm"] - b[key]["rms_mm"]
        row = {"muscle": key[0], "coordinate": key[1], "ours_rms_mm": q["rms_mm"], "myoconverter_rms_mm": b[key]["rms_mm"],
               "delta_mm": delta, "peak_osim_mm": q["peak_osim_mm"], "ok": delta <= TOL_MM}
        rows.append(row)
        if not row["ok"]:
            misses.append((key, f"ours {q['rms_mm']:.3f} mm vs MyoConverter {b[key]['rms_mm']:.3f} mm"))
    unexplained = [(k, why) for k, why in misses if f"{k[0]}|{k[1]}" not in reasons]
    result = {
        "M1": "PASS" if not unexplained else "FAIL",
        "pairs_scored": len(rows), "pairs_better": sum(r["delta_mm"] < -TOL_MM for r in rows),
        "pairs_same": sum(abs(r["delta_mm"]) <= TOL_MM for r in rows), "pairs_worse": sum(r["delta_mm"] > TOL_MM for r in rows),
        "misses_listed_with_reason": [{"pair": f"{k[0]}|{k[1]}", "detail": why, "reason": reasons[f"{k[0]}|{k[1]}"]}
                                      for k, why in misses if f"{k[0]}|{k[1]}" in reasons],
        "misses_unexplained": [{"pair": f"{k[0]}|{k[1]}", "detail": why} for k, why in unexplained],
        "ours_rms_mean_mm": sum(r["ours_rms_mm"] for r in rows) / len(rows),
        "myoconverter_rms_mean_mm": sum(r["myoconverter_rms_mm"] for r in rows) / len(rows),
        "pairs": rows,
    }
    with open(out_path, "w") as f:
        json.dump(result, f, indent=1)
    print(f"M1 {result['M1']}: {len(rows)} pairs | better {result['pairs_better']}, same {result['pairs_same']}, "
          f"worse {result['pairs_worse']} | mean RMS ours {result['ours_rms_mean_mm']:.3f} mm vs MyoConverter "
          f"{result['myoconverter_rms_mean_mm']:.3f} mm | unexplained misses {len(unexplained)}")
    for k, why in unexplained:
        print(f"   {k[0]} / {k[1]}: {why}")
    return 0 if not unexplained else 1


if __name__ == "__main__":
    sys.exit(main())
