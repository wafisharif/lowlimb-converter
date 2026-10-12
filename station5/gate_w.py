"""Station 5 gates W1 (muscle lengths), W2 (moment arms) and W3 (joint moments): ours vs MyoConverter.

Usage: python station5/gate_w.py ours.json myoconverter.json out.json [reasons.json]
  reasons.json (optional): {"<W1|W2|W3>|<muscle, muscle/coordinate or coordinate>": "reason, and the test that
  shows it"} for misses.
Exit 0 if W1-W3 pass. Gates: station5/GATES.md.
"""
import json
import sys

W1_MM, W2_MM, W3_ABS, W3_REL = 0.5, 0.1, 0.02, 0.005


def main():
    ours, base = (json.load(open(p)) for p in sys.argv[1:3])
    out_path = sys.argv[3]
    reasons = json.load(open(sys.argv[4])) if len(sys.argv) > 4 else {}
    for r, name in ((ours, "ours"), (base, "MyoConverter")):
        if r.get("W0") != "PASS":
            print(f"W1-W3 NOT EVALUATED: W0 failed for {name}")
            return 1
    result, ok_all = {}, True

    def judge(gate, rows):
        nonlocal ok_all
        listed = [dict(r, reason=reasons[f"{gate}|{r['item']}"]) for r in rows if not r["ok"]
                  and f"{gate}|{r['item']}" in reasons]
        unexplained = [r for r in rows if not r["ok"] and f"{gate}|{r['item']}" not in reasons]
        status = "PASS" if not unexplained else "FAIL"
        ok_all &= status == "PASS"
        result[gate] = {"status": status, "items": len(rows), "worse": sum(not r["ok"] for r in rows),
                        "misses_listed_with_reason": listed, "misses_unexplained": unexplained, "detail": rows}
        return status, unexplained

    rows = []
    for mn, v in ours["per_muscle"].items():
        b = base["per_muscle"].get(mn)
        if b is None:
            rows.append({"item": mn, "ok": False, "detail": "missing from the MyoConverter column"})
            continue
        rows.append({"item": mn, "ours_rms_mm": v["length_rms_mm"], "myoconverter_rms_mm": b["length_rms_mm"],
                     "ok": v["length_rms_mm"] <= b["length_rms_mm"] + W1_MM})
    s1, u1 = judge("W1", rows)

    bpairs = {(p["muscle"], p["coordinate"]): p for p in base["pairs"]}
    rows = []
    for p in ours["pairs"]:
        if not p["scored"]:
            continue
        b = bpairs.get((p["muscle"], p["coordinate"]))
        item = f"{p['muscle']}/{p['coordinate']}"
        if b is None:
            rows.append({"item": item, "ok": False, "detail": "missing from the MyoConverter column"})
            continue
        rows.append({"item": item, "ours_rms_mm": p["rms_mm"], "myoconverter_rms_mm": b["rms_mm"],
                     "ok": p["rms_mm"] <= b["rms_mm"] + W2_MM})
    s2, u2 = judge("W2", rows)

    rows = []
    for c, v in ours["per_coordinate"].items():
        if not v["gated"]:
            continue
        b = base["per_coordinate"].get(c)
        if b is None:
            rows.append({"item": c, "ok": False, "detail": "missing from the MyoConverter column"})
            continue
        rows.append({"item": c, "ours_rms": v["rms"], "myoconverter_rms": b["rms"], "unit": v["unit"],
                     "ok": v["rms"] <= W3_ABS and v["rms"] <= b["rms"] + W3_REL})
    s3, u3 = judge("W3", rows)

    def mean(gate, key):
        vals = [r[key] for r in result[gate]["detail"] if key in r]
        return sum(vals) / len(vals) if vals else float("nan")
    print(f"W1 (lengths) {s1}: {result['W1']['items']} muscles | mean RMS ours {mean('W1', 'ours_rms_mm'):.3f} mm vs "
          f"MyoConverter {mean('W1', 'myoconverter_rms_mm'):.3f} mm | unexplained misses {len(u1)}")
    print(f"W2 (moment arms) {s2}: {result['W2']['items']} scored pairs | mean RMS ours "
          f"{mean('W2', 'ours_rms_mm'):.3f} mm vs MyoConverter {mean('W2', 'myoconverter_rms_mm'):.3f} mm | "
          f"unexplained misses {len(u2)}")
    print(f"W3 (joint moments) {s3}: {result['W3']['items']} coordinates | mean RMS ours "
          f"{mean('W3', 'ours_rms'):.4f} vs "
          f"MyoConverter {mean('W3', 'myoconverter_rms'):.4f} N*m/kg | unexplained misses {len(u3)}")
    for u in u1 + u2 + u3:
        print(f"   {u}")
    with open(out_path, "w") as f:
        json.dump(result, f, indent=1)
    return 0 if ok_all else 1


if __name__ == "__main__":
    sys.exit(main())
