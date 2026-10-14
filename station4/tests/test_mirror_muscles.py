"""Station 4: a muscle that misses F1/F2 only against MyoConverter's luckier copy of it.

MyoConverter's Step 3 fits every muscle with an unseeded particle swarm, so its error for one muscle is one draw: the
left and right copies of the same muscle, identical in OpenSim apart from the mirror image, can end up far apart
(Rajagopal 2016, vastus lateralis at full activation: 1.45% of Fmax on the right, 7.87% on the left). Our refit is
deterministic, and both copies get the same error. For each listed miss (muscle, activation level) this checks:

1. the muscle and its mirror copy (_r <-> _l) are the same muscle in OpenSim: equal max isometric force, optimal
   fiber length, tendon slack length and pennation angle; the same number of path points, each on the mirrored body
   at the mirrored location (z negated, within 1e-9 m); the same wrap objects (mirrored names);
2. MyoConverter's two copies differ by more than the gate tolerance (0.5% of Fmax) at that level: its number for the
   muscle is not reproducible, it depends on the draw;
3. ours are symmetric (the two copies within 0.1% of Fmax) and within the tolerance of the mean of MyoConverter's two
   copies.

Usage: python station4/tests/test_mirror_muscles.py <Rajagopal2016.osim> <ours.json> <myoconverter.json> <out.json>
         <muscle|level> [...]
  level: a1 (F1) or a0 (F2), as in station4/<model>/f_reasons.json
Exit 0 if every check passes for every listed miss.
"""
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(os.path.dirname(HERE)))
from lowlimb_converter.support import read_osim   # noqa: E402

TOL_PCT = 0.5          # the F1/F2 tolerance (station4/GATES.md)
SYMMETRY_PCT = 0.1
fails = 0


def check(ok, msg):
    global fails
    print(("  ok   " if ok else "  FAIL ") + msg)
    fails += 0 if ok else 1


def mirror(name):
    return name[:-2] + ("_l" if name.endswith("_r") else "_r")


def muscle_xml(root, name):
    for m in root.iter():
        if isinstance(m.tag, str) and m.get("name") == name and m.find("GeometryPath") is not None:
            return m
    return None


def definition(m):
    props = {k: float(m.findtext(k)) for k in ("max_isometric_force", "optimal_fiber_length", "tendon_slack_length",
                                                "pennation_angle_at_optimal")}
    points = []
    for p in m.find("GeometryPath/PathPointSet/objects"):
        if not isinstance(p.tag, str):
            continue
        frame = (p.findtext("socket_parent_frame") or p.findtext("body") or "").strip()
        loc = np.array([float(v) for v in p.findtext("location").split()])
        points.append((p.tag, frame, loc))
    wraps = [w.findtext("wrap_object").strip() for w in m.iter("PathWrap")]
    return props, points, wraps


def main():
    osim, ours_path, mc_path, out_path = sys.argv[1:5]
    items = [a.split("|") for a in sys.argv[5:]]
    root = read_osim(osim).getroot()
    ours = json.load(open(ours_path))["per_muscle"]
    mc = json.load(open(mc_path))["per_muscle"]
    report = {}
    for name, level in items:
        other = mirror(name)
        print(f"{name} ({level}), mirror copy {other}")
        a, b = muscle_xml(root, name), muscle_xml(root, other)
        check(a is not None and b is not None, "both copies exist in OpenSim")
        if a is None or b is None:
            continue
        pa, pts_a, wr_a = definition(a)
        pb, pts_b, wr_b = definition(b)
        check(all(np.isclose(pa[k], pb[k], rtol=1e-12, atol=0) for k in pa), f"same parameters {pa}")
        def side(body):
            body = body.rstrip("/").split("/")[-1]
            return mirror(body) if body.endswith(("_r", "_l")) else body
        same_path = len(pts_a) == len(pts_b) and all(
            ta == tb and side(fa) == fb.rstrip("/").split("/")[-1] and np.allclose(la * [1, 1, -1], lb, atol=1e-9)
            for (ta, fa, la), (tb, fb, lb) in zip(pts_a, pts_b))
        check(same_path, f"{len(pts_a)} path points, each on the mirrored body at the mirrored location")
        check([mirror(w) for w in wr_a] == wr_b, f"wrap objects {wr_a} / {wr_b}")
        m1, m2 = mc[name][level]["rms_pct"], mc[other][level]["rms_pct"]
        o1, o2 = ours[name][level]["rms_pct"], ours[other][level]["rms_pct"]
        check(abs(m1 - m2) > TOL_PCT, f"MyoConverter: {m1:.2f}% vs {m2:.2f}% of Fmax for the two copies "
              f"(differ by more than {TOL_PCT}%)")
        check(abs(o1 - o2) <= SYMMETRY_PCT, f"ours: {o1:.2f}% vs {o2:.2f}% (symmetric within {SYMMETRY_PCT}%)")
        mean = 0.5 * (m1 + m2)
        check(o1 <= mean + TOL_PCT, f"ours {o1:.2f}% <= mean of MyoConverter's copies {mean:.2f}% + {TOL_PCT}%")
        report[f"{name}|{level}"] = {"myoconverter": m1, "myoconverter_mirror": m2, "ours": o1, "ours_mirror": o2,
                                     "myoconverter_mean": mean}
    with open(out_path, "w") as f:
        json.dump(report, f, indent=1)
    print("ALL CHECKS PASS" if fails == 0 else f"{fails} CHECK(S) FAILED")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
