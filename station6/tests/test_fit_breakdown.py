"""Station 6, change 6.14: a spline fit whose weighted least-squares solve breaks down.

fit_spline_on_range (Station 2, change 2.3) fits a quartic to an OpenSim spline by iteratively reweighted least
squares. The reweighting drives many weights to exactly zero, and on some machines (GitHub's Linux runners, now and
then) numpy's lstsq then raises LinAlgError ("SVD did not converge"), which failed the whole conversion (SoccerKick
and Leg39 in the station6-limitations workflow). The failure depends on the machine's LAPACK, so it is simulated here:
numpy's Polynomial.fit, as the converter's xml/utils.py sees it, raises LinAlgError from the k-th weighted solve on.

Checks, on a knee-like spline (the shape of the gait models' moving path points):
- without a breakdown, the fit is bit for bit the converter before's (the change does nothing then);
- with a breakdown at k = 1, 2, 5, 50, the fit returns, is finite, and its largest error lies between the full fit's
  and the upstream fit's (the first candidate, all that's left at k = 1); the converter before raises LinAlgError.

Usage: python station6/tests/test_fit_breakdown.py <src_dir> <old_src_dir>
"""
import importlib
import os
import sys

import numpy as np

fails = 0


def check(ok, msg):
    global fails
    print(("  ok   " if ok else "  FAIL ") + msg)
    fails += 0 if ok else 1


# A SimmSpline like a moving path point's x_location against knee angle (m against rad)
KNOTS_X = np.array([-2.0944, -1.74533, -1.39626, -1.0472, -0.698132, -0.349066, -0.174533, 0.197344, 0.337395,
                    0.490178, 1.52146, 2.0944])
KNOTS_Y = np.array([-0.0032, 0.00179, 0.00411, 0.00410, 0.00212, -0.0010, -0.0031, -0.0052, -0.0054, -0.0055,
                    -0.0055, -0.0055])
RANGE = [-2.0944, 0.174533]


def load(src):
    """The converter's xml.utils from src (a fresh import, so two converters can be compared in one process)."""
    for name in [m for m in sys.modules if m == "myoconverter" or m.startswith("myoconverter.")]:
        del sys.modules[name]
    sys.path.insert(0, src)
    try:
        importlib.import_module("myoconverter.xml.config")
        return importlib.import_module("myoconverter.xml.utils")
    finally:
        sys.path.remove(src)


def with_breakdown(utils, k):
    """Make utils' Polynomial.fit raise LinAlgError from the k-th weighted solve on; return a function undoing it."""
    real = utils.Polynomial
    count = {"n": 0}

    class Failing(real):
        @classmethod
        def fit(cls, *args, **kwargs):
            if kwargs.get("w") is not None:
                count["n"] += 1
                if count["n"] >= k:
                    raise np.linalg.LinAlgError("SVD did not converge in Linear Least Squares")
            return real.fit(*args, **kwargs)

    utils.Polynomial = Failing

    def undo():
        utils.Polynomial = real
    return undo


def main():
    src, old = (os.path.abspath(p) for p in sys.argv[1:3])
    print("6.14 a spline fit whose least-squares solve breaks down")
    old_utils = load(old)
    ref = old_utils.fit_spline_on_range("SimmSpline", KNOTS_X, KNOTS_Y, RANGE)
    utils = load(src)
    new = utils.fit_spline_on_range("SimmSpline", KNOTS_X, KNOTS_Y, RANGE)
    check(np.array_equal(new[1], ref[1]) and np.array_equal(new[2], ref[2]) and new[3] == ref[3],
          f"without a breakdown, the same fit as the converter before, bit for bit (max error {new[3]:.3g} m)")
    results = {}
    for k in (1, 2, 5, 50):
        undo = with_breakdown(utils, k)
        try:
            results[k] = utils.fit_spline_on_range("SimmSpline", KNOTS_X, KNOTS_Y, RANGE)
        except np.linalg.LinAlgError as e:
            results[k] = e
        finally:
            undo()
    upstream = results[1][3] if not isinstance(results[1], Exception) else float("nan")   # k = 1: upstream fit only
    for k, r in results.items():
        if isinstance(r, Exception):
            check(False, f"breakdown from weighted solve {k}: the fit returns (it raised {r})")
            continue
        fit, coef, joint_range, err = r
        ok = np.all(np.isfinite(coef)) and np.all(np.isfinite(joint_range)) and new[3] <= err <= upstream
        check(ok, f"breakdown from weighted solve {k}: the fit returns, finite, between the full fit and upstream's "
                  f"(max error {err:.3g} m; full {new[3]:.3g}, upstream {upstream:.3g})")
        undo_old = with_breakdown(old_utils, k)
        try:
            old_utils.fit_spline_on_range("SimmSpline", KNOTS_X, KNOTS_Y, RANGE)
            check(False, f"breakdown from weighted solve {k}: the converter before fails (it returned)")
        except np.linalg.LinAlgError:
            check(True, f"breakdown from weighted solve {k}: the converter before fails (LinAlgError)")
        finally:
            undo_old()
    print("ALL CHECKS PASS" if fails == 0 else f"{fails} CHECK(S) FAILED")
    return 0 if fails == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
