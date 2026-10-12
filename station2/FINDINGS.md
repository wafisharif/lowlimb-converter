# Station 2: kinematic check

**Result: all three models pass K0–K2.** Rajagopal2016 failed K2 at the patella (0.64°, gate 0.5°) until change 2.4
(2026-10-11) coupled the patella to the knee through two chained quartics; it is now 0.036° off at worst. Gates and
method: `station2/GATES.md` (posted to the plan doc before any run). Per-run results: `station2/<model>/<run>.json`.

Each model was posed 1000 times at random plus a 41-point sweep of every free coordinate (1410 / 1943 / 2189 poses),
and every OpenSim body was compared (12 / 12 / 22 bodies).

| Model | Converter | Worst position (mm) | RMS (mm) | Worst orientation (°) | K0 | K1 ≤ 1 mm | K2 ≤ 0.5° |
| --- | --- | --- | --- | --- | --- | --- | --- |
| gait10dof18musc | MyoConverter (Station 0, MuJoCo 2.3.7) | 0.882 | 0.524 | 0.000 | pass | pass | pass |
| | run1: Station 1a port (unchanged) | 0.882 | 0.524 | 0.000 | pass | pass | pass |
| | run4: after changes 2.1–2.3 | **0.218** | **0.114** | 0.000 | pass | pass | pass |
| gait2354 | MyoConverter (Station 0, MuJoCo 2.3.7) | 0.894 | 0.579 | 0.010 | pass | pass | pass |
| | run1: Station 1a port (unchanged) | 0.894 | 0.579 | 0.010 | pass | pass | pass |
| | run4: after changes 2.1–2.3 | **0.218** | **0.105** | 0.000 | pass | pass | pass |
| Rajagopal2016 | run1: Station 1a port (unchanged) | 15.55 | 4.07 | 0.893 | pass | **fail** | **fail** |
| | run4: after changes 2.1–2.3 | **0.152** | **0.034** | 0.639 | pass | pass | **fail** |
| | run5: after change 2.4 | 0.152 | 0.034 | **0.036** | pass | pass | **pass** |

MyoConverter's own output and our port give the same numbers (to 2e-12 mm), as Station 1a's equivalence check
predicted. (run1 used the cvt1 file and the gated run the cvt3 file; they differ by 1.4e-4 mm at most, because
MuJoCo re-saves cvt3 with normalised 6-digit axes.)

## What was wrong, and the four changes

Each change was made alone and all three models were rerun after it (`run2_frames`, `run3_precision`,
`run4_splinefit`, `run5_chained`).

| Change | What it fixes | gait10 / gait2354 worst | Rajagopal2016 worst |
| --- | --- | --- | --- |
| run1 (none) | | 0.882 mm / 0.894 mm, 0.010° | 15.55 mm, 0.893° |
| 2.1 frames | Joints with an offset child frame rotated about the wrong point | no change (files byte-identical) | **0.204 mm**, 0.893° |
| 2.2 precision | Every number was written with 4 significant figures | 0.879 mm / 0.879 mm, **0.000°** | 0.177 mm, 0.894° |
| 2.3 spline fit | Quartic fit to the knee/patella splines was fitted in the wrong place | **0.218 mm** / **0.218 mm** | 0.152 mm, **0.639°** |
| 2.4 chained coupling | No single quartic follows the patella's rotation spline | no change (files byte-identical) | 0.152 mm, **0.036°** |

- **2.1 Joint frames** (`xml/bodies/Body.py`, `xml/joints/Joint.py`). OpenSim rotates a joint about its child frame;
  MuJoCo rotates about the joint's `pos`, which the converter never set, so it rotated about the body origin.
  Rajagopal's knee centre is 9.0 mm from the tibia origin, so the shank swung on the wrong pivot: 15.6 mm off at full
  flexion. The body placement also composed the two frames in the wrong order. That did no harm in these three
  models (where offset frames exist, parent and child orientations are equal), but it does in general. A test
  model with arbitrary frames on four joints (`station2/tests/test_frame_offsets.py`) is off by up to 75 mm and 27°
  with the old code and matches exactly with the new.
- **2.2 Number precision** (`xml/utils.py`, `vec2str`). Offsets, axes, angles and polynomial coefficients were
  written as `'%.4g'`. That tilted gait2354's oblique subtalar and MTP axes by up to 0.01°, moved Rajagopal's arm
  bodies by up to 0.065 mm, and turned the 90° root rotation into 1.571 rad (90.0117°), which tilts gravity
  relative to the model by 0.0117° in every converted model. Now written at full precision.
- **2.3 Spline fit** (`xml/joints/CustomJoint.py`, `xml/utils.py`). MuJoCo can only couple joints with a quartic, so
  OpenSim's knee and patella splines are approximated. Upstream fitted a least-squares quartic to the spline's knots
  across all knots. In the gait models the knots cover −120° to +120° of knee flexion but the knee only moves
  −120° to +10°, so the fit spent its accuracy where the knee never goes: 0.87 mm error at a straight knee. The
  new fit targets OpenSim's own spline (evaluated with OpenSim) over the coordinate's range and minimises the
  largest error there; upstream's fit is kept as a candidate, so it is never worse on that range by that measure.
  Joint ranges now cover every value the fit takes, and are never narrower than upstream's.
- **2.4 Chained couplings** (`xml/joints/CustomJoint.py`, `xml/utils.py` `fit_spline_chained`). The patella's
  rotation is a spline of knee angle that is flat up to about 10° and then turns sharply. MuJoCo couples a joint to
  another with a quartic, and the best possible quartic is 0.639° off. But a coupling can go through a helper
  joint: helper = p1(knee), patella = p2(helper), two quartics in sequence, which can follow the bend. Both quartics
  are fitted together to OpenSim's spline over the knee's range (largest error minimised, deterministic). The helper
  joint sits on its own near-massless body under the world body (the template's minimum body mass, 0.001 kg, like
  MyoConverter's virtual path-point bodies but attached to nothing that moves, so it adds no mass to the skeleton).
  The converter only chains a coupling where one quartic is off by more than half of the Station 2 limits (0.25°,
  0.5 mm) and the chain at least halves the error: in these three models that is the two patella rotations only
  (gait-model knees: 0.2 mm, left as single quartics; the gait models' converted files are byte-identical).
  - Static result: patella orientation worst 0.639° → 0.036°, RMS 0.56° → 0.031°; every other body unchanged.
  - In motion (`station2/tests/test_chained_coupling.py`: MuJoCo simulation, gravity off, knee driven through
    11–81° for 4 s): the patella follows OpenSim's spline of the simulated knee angle within 0.076° (single quartic:
    0.697°). The helper adds no instability (equality residuals of the same size as before, simulation finite).
  - Since Station 5's change 5.3 the helper is rescaled so it adds almost no inertia or damping to the knee (its
    slope |d helper / d knee| ≤ 0.1; the coupling is the same function): every body is unchanged (`run7_switch`) and
    the patella follows within 0.099° in motion. A first try with slope ≤ 0.01 made it lag 2.4° (the helper became
    too cheap to move for MuJoCo's soft equality on the patella; see station5/FINDINGS.md).
  - The patella's position was not the problem (0.15 mm, within its quartic fits) and is unchanged.

## The check itself was checked

- **Mutations** (`station2/tests/test_harness.py`): moving one body 2 mm gives exactly 2.000000000 mm on that body
  alone; rotating it 1° gives exactly 1.000000000000°; changing only the ground frame, renaming a body, adding an
  unexplained joint, dropping a polynomial term, or losing a pose all fail K0–K2 as expected.
- **Independent cross-check:** the knee sweep recomputed with neither the ground-frame transform nor the
  harness's pose code (tibia relative to femur, read straight from OpenSim and MuJoCo) agrees pose by pose to 4e-13 mm.
- **OpenSim side:** setting Rajagopal's coupled patella coordinate 0.01 rad off shows up as QErr 0.01, so the
  zero QErr in every reference pose means the couplings were really satisfied.
- Re-run Station 1b's tests on the changed converter: all pass, same predictions.

## Effects beyond Station 2 (not gated here)

- The full pipeline still runs and passes the structural gates (Station 1a P1–P2) on gait10dof18musc.
- As expected, Station 1a's "matches the baseline" gates (P3–P5) no longer hold. MyoConverter's own Step 3 force
  error for gait10 went from 0.111 to 0.105 Fmax (mean); its moment-arm RMS error after Step 2 changed by −0.03 to
  +0.23 mm per muscle group (worst rect_fem_l). Station 3 will measure moment arms properly.
- The Station 1a CI workflow is now manual-only (run it on tag `station1a` to re-verify the port). A new
  `station2-kinematics` workflow runs Station 2 and its tests, plus P1–P2, on every push.

## Where the code prediction (written before the run) was wrong

`GATES.md` predicted Rajagopal would fail below all 14 joints with offset child frames. Only the two knees failed on
position: the other 12 have zero child translation and equal parent/child orientations, which the old code handled
correctly. The patella's K2 failure was not predicted.
