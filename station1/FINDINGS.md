# Station 1a: port to the modern stack

**Result: the vendored MyoConverter pipeline runs on Python 3.12, OpenSim 4.6, MuJoCo 3.15.0 and
NumPy 2.5.3, and passes every Station 1a gate (P1–P5) for both gait models.** Its moment-arm and
force results match the Station 0 baseline to floating-point precision. It took four fixes, each a
separate commit, each checked against the old behaviour.

## Fixes

| # | Commit | Problem on the modern stack | Fix | How we know behaviour is unchanged |
| --- | --- | --- | --- | --- |
| 1 | 3567f1f | `networkx` missing (trimesh mesh repair needs it; not imported directly) | Added to `env/requirements.in` | Environment only |
| 2 | c451216 | `<option collision="predefined"/>` was removed in MuJoCo 3.0.0 | MuJoCo's documented migration: delete it, default geom `contype=0 conaffinity=0`, remove the one explicit `contype/conaffinity` | Compiled models: no geom can collide automatically, and the explicit contact pairs are identical (19 per model) |
| 3 | 74853ef | `mjModel.eq_active` renamed `eq_active0`; `actuator_moment` became sparse | Rename (2 places); rebuild the dense matrix with `mju_sparse2dense` | `station1/tests/test_actuator_moment.py`: matches MuJoCo 2.3.7 to 3.5e-17 |
| 4 | 5a6408b | Particle-swarm stall check compared a float to `[]`, which NumPy 2 rejects | Start `obj_g_old` as `None` (Step 2 and Step 3 optimisers) | Same decisions as NumPy 1.x on every iteration of a test sequence |

Fix 3 was found by checking every MuJoCo name the code uses against MuJoCo 3.15, not only the line
that crashed. That audit also confirmed no removed NumPy aliases are used and that numbers are written
to XML only through `'%.4g'` formatting or `str()` of a scalar, which NumPy 2 left unchanged. A scan
of every generated XML for `np.` text found nothing.

## Gate results

Both run with `station1/run_station1.sh` on 2 CPU cores (Linux x86_64, cloud sandbox).

| Model | P1 runs | P2 structural | P3 same structure | P4 moment arms (tol 0.1 mm) | P5 forces (tol ×1.05 mean, +0.02 per muscle) | Wall time |
| --- | --- | --- | --- | --- | --- | --- |
| gait10dof18musc | PASS | PASS | PASS (38 joints, 18 actuators, 28 constraints) | PASS (worst +7.2e-18 m over 18 groups) | PASS (mean ratio 1.000000000000, worst +2.8e-16 Fmax) | 145 s |
| gait2354 | PASS | PASS | PASS (61 joints, 54 actuators, 38 constraints) | PASS (worst +5.6e-18 m over 58 groups) | PASS (mean ratio 1.000000000000, worst +8.9e-16 Fmax) | 533 s |

Before-optimisation errors, which are reported but not gated, also match the baseline: forces within 2.4e-15 / 3.3e-15 Fmax and moment arms within 7.2e-18 / 6.9e-18 m (gait10 / gait2354).

## Are the converted models the same models?

The text of cvt3 differs from the baseline, because MuJoCo's XML writer changed style between versions
(defaults omitted, new optional attributes). So we compiled both models (baseline in MuJoCo 2.3.7, ours
in 3.15.0) and compared 82 physics arrays (`station1/tests/dump_compiled.py`, `compare_compiled.py`).
Every difference has a known cause:

- **Muscle dynamics type 3 → 4.** MuJoCo 3 inserted new enum values (`mjDYN_FILTEREXACT`), so this is a renumbering.
- **contype/conaffinity 1 or 2 → 0 on every geom.** This is fix 2; contacts come only from the identical explicit pairs, as before.
- **Mass of "ground" and the virtual moving-path-point bodies:** 0 and 5.2e-7 kg in the baseline, 0.001 kg in ours.
  The template asks for `boundmass="0.001"`. MuJoCo 2.3.7's XML writer dropped that setting when saving cvt2/cvt3,
  and MuJoCo 3.15 keeps it. Total mass differs by 0.015% (gait10: 75.1646 vs 75.1756 kg, 11 bodies) and 0.020% (gait2354: 75.1646 vs 75.1796 kg, 15 bodies), all on massless virtual
  points (ground is welded to the world). Static moment-arm and force checks are unaffected. **Flag for Station 5 dynamics.**
- **Mesh geom frames (up to 0.9 mm and a small rotation) on mesh geoms.** These come from the mesh files, not MuJoCo:
  compiling our model with the baseline's mesh files removes all of them. Mesh *vertices* are identical. trimesh 5's
  repair step orients some faces differently than trimesh 3.23, which changes the computed mesh volume and centre.
  Placed on their bodies, every vertex matches the baseline to 4e-8 m (`station1/tests/mesh_vertices.py`), so the
  visual shape and the convex-hull collision shape are the same.

Other differences, none affecting physics:

- cvt1 carries our template change notice as an XML comment (Apache 2.0 requires the notice in the modified template).
- The PDF report library logs "Ignoring unsupported SVG tag: <metadata>" for plots from newer matplotlib.
- Two runs on the modern stack produce byte-identical models, metrics and mesh files.

## How to reproduce

```bash
python3.12 -m venv .venv && .venv/bin/pip install --require-hashes -r env/requirements.lock
bash station1/run_station1.sh gait10dof18musc upstream station1/gait10dof18musc .venv/bin/python
```

`upstream` is the MyoConverter clone at cadf380, which supplies the OpenSim models. Verified on Linux. On macOS it
should run natively on Apple Silicon with no Docker (the lockfile includes those wheels; OpenSim's wheel needs macOS 15
or newer), but that is **not verified yet**. The `macos` job in `.github/workflows/station1-modern-stack.yml` checks it.

---

# Station 1b: element-support report

**Result: `lowlimb_converter/support.py` lists every element of an OpenSim model with one status and predicts the
XML conversion step correctly on all four models tried.** Gates S1–S3 (`station1/GATES.md`) all pass. Reports and
provenance are in `station1/support/`.

| Model | Elements | Converted | Approximated | Skipped | Ignored | Unsupported | Predicted | Actual XML step |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| gait10dof18musc | 154 | 115 | 35 | 0 | 4 | 0 | converts | converts (Station 1a) |
| gait2354 | 363 | 284 | 75 | 0 | 4 | 0 | converts | converts (Station 1a) |
| Rajagopal2016 | 766 | 652 | 112 | 0 | 2 | 0 | converts | converts |
| RajagopalLaiUhlrich2023 | 770 | 652 | 104 | 0 | 12 | 2 | fails at `walker_knee_r/translation1` | fails at `walker_knee_r/translation1` |

- **S1 complete:** for all four models, every category count matches an independent whole-file XPath count
  (bodies, joints, coordinates, transform axes, constraints, forces, path points, path wraps, markers, wrap objects,
  geometry, other sets, gravity), with no duplicates and 0 unclassified.
- **S2 matches the converter:** for both gait models, every body, coordinate, muscle (+ tendon), mesh, path point,
  moving/conditional point, spline axis and wrap lands in cvt1 under the predicted name, and nothing listed as
  skipped/ignored does. Both no-anchor ConditionalPathPoint warnings are in the log, and no others.
- **S3 predicts new models:** checked against the converter's stack frames at the moment of failure, not just the
  error text. A third, deliberate case (Rajagopal2016 with only its model folder for meshes) was predicted to fail at
  mesh `talus_r/talus_r_geom_1`, and did.
- Deliberately corrupted reports (a path point removed, a body mislabelled, the outcome flipped) all fail S1/S2,
  so the checks can fail.

## What the reports found

- **Gravity is never converted.** All four models specify 9.80665 m/s²; the converted models use MuJoCo's default
  9.81 m/s² (0.034% higher).
- **Rajagopal's 17 torque actuators (lumbar and arms) are 10× too weak.** OpenSim's `optimal_force` is 10 N·m per unit
  control; the converter reads it but drops it (an upstream TODO), so the MuJoCo motors give 1 N·m per unit. Confirmed
  in the compiled model. This must be fixed before Rajagopal is validated.
- **RajagopalLaiUhlrich2023 is blocked only by its knees.** Both `walker_knee` joints use a `PolynomialFunction`, which
  the converter does not handle. MuJoCo joint couplings are quartic polynomials, so a polynomial of degree ≤ 4
  could be converted exactly. This is a candidate improvement for model 4.
- **Silent losses to watch for in other models:** attached geometry other than meshes (e.g. contact spheres),
  `prescribed` coordinates, ligament forces (only the resting length survives, so they exert no force), and any
  top-level ContactGeometrySet / ControllerSet / ComponentSet / ProbeSet contents.

## How the tool was corrected along the way

The first runs exposed bugs in the report tool, not in the gates. Each was fixed, and the checks rerun:

1. It looked for joint functions under a `<function>` wrapper; OpenSim 4 files put them directly under the axis.
   gait10 was wrongly predicted to fail. Found before S2 ran.
2. It expected coordinates inside an `<objects>` wrapper. Also found before S2 ran.
3. The first S3 run failed for both Rajagopal models. The tool had missed that CustomJoint axes are processed as
   translations first, and that each coordinate's first axis must pass `_designate_dof` (only `SimmSpline` or
   `LinearFunction`). It also didn't check that mesh files exist. All three were added.
4. S3's matching rule was made **stricter** after that first run. It had required the axis name to appear in the
   error text, which `_designate_dof`'s error never contains. It now requires the converter's stack frames at the
   moment of failure to be on exactly the predicted element.
