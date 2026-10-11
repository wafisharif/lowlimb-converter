"""Station 4 gates F1 (full activation) and F2 (passive): ours not worse than MyoConverter, muscle by muscle.

Usage: python station4/gate_f.py ours.json myoconverter.json out.json [reasons.json]
  reasons.json (optional): {"<muscle>|<a1 or a0>": "reason, and the test that shows it"} for misses.
Exit 0 if F1 and F2 pass (every muscle within MyoConverter's RMS + 0.5% Fmax, or listed with a reason).
"""
import json
import sys

TOL_PCT = 0.5
GATES = {"F1": "a1", "F2": "a0"}


def main():
    ours, base = (json.load(open(p)) for p in sys.argv[1:3])
    out_path = sys.argv[3]
    reasons = json.load(open(sys.argv[4])) if len(sys.argv) > 4 else {}
    for r, name in ((ours, "ours"), (base, "MyoConverter")):
        if r.get("F0") != "PASS":
            print(f"F1/F2 NOT EVALUATED: F0 failed for {name}")
            return 1
    result = {}
    ok_all = True
    for gate, key in GATES.items():
        rows, unexplained, listed = [], [], []
        for mn, v in ours["per_muscle"].items():
            b = base["per_muscle"].get(mn)
            if b is None:
                unexplained.append({"muscle": mn, "detail": "missing from the MyoConverter column"})
                continue
            delta = v[key]["rms_pct"] - b[key]["rms_pct"]
            rows.append({"muscle": mn, "ours_rms_pct": v[key]["rms_pct"], "myoconverter_rms_pct": b[key]["rms_pct"],
                         "delta_pct": delta, "ok": delta <= TOL_PCT})
            if delta > TOL_PCT:
                item = {"muscle": mn, "detail": f"ours {v[key]['rms_pct']:.2f}% vs MyoConverter {b[key]['rms_pct']:.2f}%"}
                if f"{mn}|{key}" in reasons:
                    listed.append({**item, "reason": reasons[f"{mn}|{key}"]})
                else:
                    unexplained.append(item)
        status = "PASS" if not unexplained else "FAIL"
        ok_all &= status == "PASS"
        result[gate] = {"status": status, "activation": key, "muscles": len(rows),
                        "better": sum(r["delta_pct"] < -TOL_PCT for r in rows),
                        "same": sum(abs(r["delta_pct"]) <= TOL_PCT for r in rows),
                        "worse": sum(r["delta_pct"] > TOL_PCT for r in rows),
                        "ours_mean_rms_pct": sum(r["ours_rms_pct"] for r in rows) / len(rows),
                        "myoconverter_mean_rms_pct": sum(r["myoconverter_rms_pct"] for r in rows) / len(rows),
                        "misses_listed_with_reason": listed, "misses_unexplained": unexplained, "muscles_detail": rows}
        g = result[gate]
        print(f"{gate} ({key}) {status}: {g['muscles']} muscles | better {g['better']}, same {g['same']}, worse {g['worse']} "
              f"| mean RMS ours {g['ours_mean_rms_pct']:.2f}% vs MyoConverter {g['myoconverter_mean_rms_pct']:.2f}% Fmax "
              f"| unexplained misses {len(unexplained)}")
        for u in unexplained:
            print(f"   {u['muscle']}: {u['detail']}")
    with open(out_path, "w") as f:
        json.dump(result, f, indent=1)
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
