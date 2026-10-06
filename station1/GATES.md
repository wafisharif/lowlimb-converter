# Station 1a gates: port to the modern stack

Written 2026-10-06, committed before the vendored pipeline was run on the modern stack.
Changing a gate after seeing results needs a new dated entry here and a deviation row in
`EXPERIMENTS.md`. The old text is never edited.

**Modern stack:** Python 3.12, OpenSim 4.6 and MuJoCo 3.15.0 from PyPI, NumPy 2.5.3
(`env/requirements.lock`).

**Baseline:** our Station 0 rerun in `baseline/<model>/` (MyoConverter @ cadf380, MuJoCo 2.3.7,
OpenSim 4.4.1). We do not use upstream's shipped outputs here, because their before-optimisation
Step 3 errors do not match a rerun (see EXPERIMENTS.md, 2026-10-06). The final cvt3 models are
equivalent.

**Pipeline settings:** identical to `baseline/run_baseline.py` (all 3 steps, validation on,
`speedy=False`, PDF on, ground geom on, constraints for moving/conditional path points).

## Gates, per model (gait10dof18musc, gait2354)

- **P1 RUNS:** the pipeline finishes all 3 conversion steps and 3 validation steps without an exception.
- **P2 STRUCTURAL:** `baseline/check_counts.py` JOINT, MUSCLE and KEYFRAME gates PASS under OpenSim 4.6 and
  MuJoCo 3.15.0. That means the cvt3 model loads and initialises in current MuJoCo.
- **P3 SAME STRUCTURE AS BASELINE:** ported cvt3 and baseline cvt3 have identical joint names, actuator names
  and equality-constraint counts, read directly from the MJCF XML. Compiling isn't possible here: the baseline uses
  MuJoCo 2.3.7 syntax, and its meshes are not in git. The XML counts were checked against the compiled counts from
  Station 0 before this gate was committed.
- **P4 NO MOMENT-ARM REGRESSION:** the same (muscle, joint group) set; every group's after-optimisation RMS
  moment-arm error ≤ baseline + 0.1 mm. (0.1 mm is about 1–3% of the baseline mean error; the port should not
  change geometry, so differences can only come from simulator numerics.)
- **P5 NO FORCE REGRESSION:** the same muscle set; mean after-optimisation force error ≤ 1.05 × baseline mean,
  and no muscle worse than baseline by more than 0.02 Fmax.

Always reported, never gated: per-group and per-muscle differences (improvements too), before-optimisation
errors, run time.

If a gate fails, thresholds are not loosened. We find the cause, log it, fix it in a separate commit, and
rerun the full pipeline.
