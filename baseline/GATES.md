# Gates for Station 0–1

Written before the result each gate applies to was looked at. Changing a gate after seeing results
needs a new dated entry here and a deviation row in `EXPERIMENTS.md`; the old text is never edited.

## 2026-10-05: Station 1 structural gates (`check_counts.py`)

Written before gait2354 was run.

- **JOINT GATE:** MuJoCo joints not driven by a joint-equality constraint must equal the OpenSim coordinates, by name.
- **MUSCLE GATE:** OpenSim muscle names must equal MuJoCo actuator names.
- **KEYFRAME GATE:** cvt3 loads, and keyframe 0 violates no equality constraint (max |violation| < 1e-6).

## 2026-10-05: Station 0 reproducibility gates (`compare_to_upstream.py`)

Our rerun of upstream @ cadf380 against the outputs upstream ships in `models/mjc/`.
Written after reading upstream's shipped metrics, before extracting metrics from our rerun.
Note: the shipped outputs may have been produced with a different MuJoCo/OpenSim build than our pinned env.

- **R1 STRUCTURE:** our cvt3 and the shipped cvt3 have identical joint names, actuator names and equality-constraint count.
- **R2 MOMENT ARMS:** Step 2 ran no optimisation on either gait model in the shipped outputs, so its errors are
  deterministic. The same (muscle, joint group) set must exist, and each `rms_opt_m` must match within 1e-6 m.
- **R3 FORCES:** Step 3 uses particle swarm optimisation with unseeded random starts, so exact
  equality is not expected. Gate: |mean ours − mean shipped| ≤ 10% of mean shipped, and no muscle
  differs by more than 0.05 Fmax. Per-muscle differences are always reported.
- **Step 1 endpoint error is not gated:** both gait models contain zero markers, so upstream's Step 1
  endpoint check compares nothing and reports NaN. Our Station 2 must supply its own reference points.
