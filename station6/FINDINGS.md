# Station 6: past MyoConverter's limitations

**Result (run5, the final converter): gates L0–L6 pass; L7 (contact dynamics on held-out drops) fails.** Of the 56
model files OpenSim distributes (opensim-models @ d9b05d4), 54 now convert (MyoConverter: 40); the support report
predicts every file's outcome before converting, for both converters. On every converted file the mass properties are
exactly OpenSim's (MyoConverter's fail on 32 of its 40), and every coordinate limit force, path actuator, path spring
and contact pair's geometry matches OpenSim's within the gates' limits. The contact *dynamics* don't generalise:
calibrated on two vertical drops of OpenSim's WalkerModel, the converted contacts follow OpenSim within 8.0 mm on a
held-out drop from another height (MuJoCo's defaults: 16.2 mm), but tilted and sliding drops part from OpenSim by
37–62 mm, worse than MuJoCo's defaults on two of the three.
Gates and method: `station6/GATES.md` (posted to the plan doc before any Station 6 code existed; amendment of
2026-10-13 to the checks). Per-run results: `station6/results/<run>/`.

## What MyoConverter couldn't do, and what changed

| Change | What | Files it unlocks or fixes |
| --- | --- | --- |
| 6.1 | Read every OpenSim 4 file: element names with `::` (lxml refused them), coordinates that omit `<range>`, `<clamped>` or `<locked>` (OpenSim's class defaults), mesh extensions in any case, placeholder meshes (`Unassigned`, skipped), joints whose parent socket names a body | WalkerModel ×2, ToyLanding ×3, DynamicJumper, SoccerKick, double pendulum |
| 6.2 | Planar, Free, Ball and Gimbal joints, converted as the CustomJoint each one is | WalkerModel ×2, SoccerKick, Pendulum; synthetic tests for all four (exact to 1e-12 mm) |
| 6.3 | Mass properties exactly OpenSim's: the template's compiler bounds raised every inertia below 0.001 kg m² | every model (MyoConverter's fail L3 on 32 of 40 files: toes, tali, hands, forearms, patellae) |
| 6.4 | CoordinateLimitForce as dead-band springs (MuJoCo fixed tendons) on the coordinate: exact beyond OpenSim's smooth onset, interpolated in it | 24 limit forces in 9 files, within 0.11–0.22% of their peak force |
| 6.5 | PathActuator (tendon and motor) and PathSpring (tendon with a one-sided spring) | the gait10 exoskeleton examples ×4: lengths within 0.19 mm, forces within 1.6% |
| 6.6 | ContactSphere and ContactHalfSpace as geoms, HuntCrossleyForce as contact pairs, with the pair parameters calibrated on rigid OpenSim drops of the model | WalkerModel ×2, ToyLanding ×3, DynamicJumper, SoccerKick: geometry exact (1e-14 m) |
| 6.7 | ASCII STL meshes rewritten as binary STL (MuJoCo reads binary only); OBJ meshes copied | SoccerKick |
| 6.8 | A half-space on a body that can move becomes a large box with the half-space's surface as its face (MuJoCo allows planes only on static bodies) | ToyLanding ×3 |
| 6.9 | The calibration's rigid drop: coupled coordinates follow their coupler, the root is the first ground joint with an unlocked translation, only pairs the drop separates count, the path-point fits' cached OpenSim model is released first | DynamicJumper, ToyLanding ×3 |
| 6.10 | No damping on joints a coupling drives (the template's 0.05 N·m·s/rad fought the coupling once 6.3 gave light bodies their real inertia) | Rajagopal's patellae lagged 1.9° in motion; walking joint moments 0.0014 → 0.0011 N·m/kg (Station 5) |
| 6.11 | A mesh file named in another case finds its file (as OpenSim does on macOS and Windows) | Leg39 |
| 6.12 | The calibrated contact parameters must keep the whole, unlocked model stable | WalkerModel: the best fit to the drops threw the free model 4 m into the air |
| 6.13 | The calibration's OpenSim drops run in a fresh process (bit for bit the same result) | ToyLandingModel: in the conversion's own process OpenSim's first step failed in 1 of 12 conversions (run4), leaving its contacts uncalibrated |

Still not converted: PointOnLineConstraint (BouncingBlock ×2, the only files left), bushing forces (ToyLanding's
AFOs: skipped, as reported), PointActuator and TorqueActuator (no file uses them), contact meshes and elastic
foundations, and geometry attached to offset frames inside bodies (visual only).

## Results, gate by gate (run5)

| Gate | What | Result |
| --- | --- | --- |
| L0 | The support report predicts the XML step's outcome, element by element, for every file (both converters) | **pass**: 56/56 for ours, 56/56 for MyoConverter's (Station 1a port with the report as of 4313be5); S1 on all 56 files |
| L1 | Every file converts except those whose first unsupported element is out of scope | **pass**: 54/56 (MyoConverter 40/56; it stops on `::` names ×9, unsupported elements ×6, a mesh name ×1) |
| L2 | Station 2's kinematics on WalkerModel ×2 and SoccerKick (plus the synthetic joints) | **pass**: WalkerModel exact; SoccerKick 0.40 mm / 0.05°; synthetic Planar, Free, Ball, Gimbal within 1e-12 mm. Reported for every other converted file: all pass except DynamicJumper (1.38 mm / 1.39°, its patella) and Leg39 (1.58 mm / 1.39°) |
| L3 | Mass, centre of mass and inertia within 1e-9 of OpenSim's, every body of every file | **pass**: 54/54 files (MyoConverter: 32 of its 40 fail) |
| L4 | Every limit force within 1% of its peak over a sweep | **pass**: 24 forces in 9 files, worst 0.22% |
| L5 | Path actuators and springs: length RMS ≤ 1 mm, force RMS ≤ 2% | **pass**: actuator exact, spring 0.19 mm / 1.57% |
| L6 | Contact geometry: signed distance within 1e-6 m; OpenSim's force on exactly where a pair penetrates | **pass**: 7 files, 66 pairs, worst 3e-14 m (with MuJoCo's own kinematics, reported: up to 0.86 mm, the L2 body errors) |
| L7 | WalkerModel's held-out drops: each within 10 mm and 2°; mean at most half of MuJoCo's defaults | **fail**: C1 and C2. Position / rotation RMS, ours vs MuJoCo's defaults: drop from 0.05 m 7.98 mm / 0.55° vs 16.17 mm / 1.23°; tilted +0.15 rad 37.5 mm / 1.64° vs 43.0 mm / 3.41°; tilted −0.15 rad 61.8 mm / 3.84° vs 13.6 mm / 1.38°; sliding at 0.5 m/s 56.8 mm / 4.25° vs 11.4 mm / 1.12°. Mean 41.0 mm vs 21.0 mm (run3, before 6.12: 36.4 mm) |

## Contact dynamics (L7): why it fails

MuJoCo's soft contact is a linear spring–damper set by a time constant and a damping ratio, with Coulomb friction;
OpenSim's Hunt–Crossley contact is a nonlinear spring (force ∝ penetration^1.5) with velocity-dependent damping and a
regularised (Stribeck plus viscous) friction law. The converter fits the time constant, damping ratio and friction of
each material to two vertical drops of the model made rigid (0.02 and 0.10 m), which it reproduces within 6.8 and 2.8
mm (MuJoCo's defaults: 13.8 and 16.1 mm). The fit is sharp: on those drops, changing the time constant from 7 to 5 ms
takes the error from 3.7 to 16 mm, and the damping ratio it prefers (about 0.3) makes contacts bouncy. Tilted and
sliding drops excite rocking and sliding that two vertical drops don't constrain, and on two of the three MuJoCo's
defaults (time constant 20 ms, damping ratio 1, friction 1) do better than the fit.

How it was found matters: the gate keeps four drops out of development, but a development run of the gate script on
run2's conversions evaluated them by mistake (it ran every gate, L7 included). The result (C1 and C2 FAIL) is
disclosed in the amendment; the calibration was not tuned after it. Change 6.12 came from a different, reported-only
check: released unlocked, the converted WalkerModel was thrown 4 m into the air with the best-fitting parameters (its
shanks weigh 0.05 kg). The calibration now keeps the best candidate that leaves the whole model stable; run4's and
run5's L7 numbers are with that change. Released unlocked, the WalkerModel is no longer thrown up (after 1 s its
Pelvis_ty is −0.16 m in MuJoCo, −0.87 m in OpenSim), so it is stable, but it does not follow OpenSim (RMS over the
second: Pelvis_ty 0.70 m, Pelvis_tx 0.42 m, Pelvis_rx 83°; reported only, not a gate).

What would be needed: a contact model closer to Hunt–Crossley (MuJoCo's solimp can make stiffness grow with
penetration; a velocity-dependent friction can be approximated with soft friction constraints), and calibration
scenarios that include rocking and sliding. Until then the converted contacts are geometrically exact, and their
parameters a starting point to check against OpenSim for the motion at hand; `osim2mujoco validate` reports the
held-out drops for any model with a WalkerModel-like root.

## Checked along the way

- Every change has a test that fails on the converter before it (`station6/tests/`: reading, joint types, forces and
  mass, contacts, more files; `station2/tests/test_chained_coupling.py` for 6.10). 6.13's original failure is
  intermittent (1 in 12 conversions), so its test checks the isolation itself: the drops run with OpenSim blocked in
  the calling process, which the converter before can't do.
- The calibration and the gate's harness are separate code; both simulate OpenSim's rigid drop alike (2e-14 m).
- run1 (stopped after L0–L1 failed on five files: ASCII STL, a plane on a moving body, an OpenSim drop that couldn't
  be assembled) and run2 (stopped after L1 failed on Leg39) are in `station6/results/run1`, `run2`; run3 passed L0–L6
  and failed L7 before change 6.12. run4 gave run5's gate results exactly, but ToyLandingModel's contact calibration
  had failed inside its conversion (reported in `gates.json`; no gate checks that a calibration ran): change 6.13, then
  run5, where every contact model is calibrated (`gates.json`, L7 "calibrations").
- The MyoConverter column of L0–L1 and L3 is the Station 1a port: upstream MyoConverter with only the four port fixes,
  on the same modern stack.
