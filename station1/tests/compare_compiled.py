"""Compare two dumps from dump_compiled.py. Prints every array that differs beyond tolerance.
Usage: python compare_compiled.py <a.npz> <b.npz> [atol=1e-6]"""
import sys, numpy as np
a, b = np.load(sys.argv[1]), np.load(sys.argv[2]); atol = float(sys.argv[3]) if len(sys.argv) > 3 else 1e-6
only = sorted(set(a.files) ^ set(b.files)); bad = []
for k in sorted(set(a.files) & set(b.files)):
    x, y = a[k], b[k]
    if x.shape != y.shape: bad.append((k, f"shape {x.shape} vs {y.shape}")); continue
    if x.dtype.kind in "US":
        if not (x == y).all(): bad.append((k, "names differ"))
    elif not np.allclose(x.astype(float), y.astype(float), atol=atol, rtol=0, equal_nan=True):
        d = np.abs(x.astype(float) - y.astype(float)); bad.append((k, f"max |diff| {np.nanmax(d):.3g} at {np.unravel_index(np.nanargmax(d), d.shape)}"))
print(f"compared {len(set(a.files) & set(b.files))} arrays (atol={atol}); in one dump only: {only}")
for k, msg in bad: print("  DIFF", k, msg)
print("EQUIVALENT" if not bad else f"{len(bad)} ARRAYS DIFFER")
