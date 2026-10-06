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

# Station 1b gates: element-support report

Written 2026-10-06, before the report tool existed or had been run on any model. A copy was posted to the plan doc
(version history timestamps it) before any run.

**What the tool does:** reads an OpenSim .osim file and lists every element the converter could act on, each with
exactly one status:

- `converted`: converted with no known approximation
- `approximated`: converted, but with a documented approximation (named in the report)
- `skipped`: the converter knowingly skips it and logs a warning
- `ignored`: the converter never looks at it (no warning)
- `unsupported`: the converter would stop with an error here

The model's predicted outcome is `fails` if any element is `unsupported`, otherwise `converts`. Every status rule
cites the vendored source line it comes from.

## Gates

- **S1 COMPLETE (every model):** every element in BodySet (incl. each body's attached geometry and wrap objects),
  JointSet (incl. coordinates and each CustomJoint transform axis), ConstraintSet, ForceSet (incl. every path point
  and path wrap), MarkerSet, the ground's geometry and wrap objects, every other top-level set or component, and
  the model's gravity, appears exactly once with exactly one status. 0 unclassified. Element counts are checked
  against an independent XPath count of the .osim file.
- **S2 MATCHES THE CONVERTER (gait10dof18musc, gait2354; models we have converted):** every body, coordinate,
  muscle and marker reported `converted`/`approximated` exists under its expected name in our Station 1a cvt1
  output; every `skipped`/`ignored` element does not; every `skipped` element's warning text is in the conversion
  log; predicted outcome `converts`.
- **S3 PREDICTS NEW MODELS (Rajagopal2016, RajagopalLaiUhlrich2023 from opensim-org/opensim-models):** the predicted
  outcome (`converts`, or `fails` with the first `unsupported` element in converter traversal order) matches what
  the ported pipeline's XML conversion step actually does. Checked by running that step.

If a gate fails we fix the report tool (separate change, logged), not the gate.
