# Station 5: walking

**Result: both gait models (run7_damping, the current converter) and Rajagopal 2016 (run4_damping) pass W0–W3. Along
OpenSim's own walking data, muscle lengths are 33–42× closer to OpenSim's than MyoConverter's (Rajagopal 2016: 39×),
moment arms 11–15× closer (5.6×), and the joint moments needed to walk 1.4–1.6× closer (10×); no muscle, pair or joint
is worse than MyoConverter beyond the gate tolerances.**
Gates and method: `station5/GATES.md` (posted to the plan doc before any Station 5 code existed). Per-run results:
`station5/<model>/<run>.json`, `<run>_w.json`, and level 3 in `<run>_replay.json` / `<run>_replay_nogrf.json`.

The walking trial is OpenSim's example for each model (opensim-models @ d9b05d4, `Pipelines/`): 151 frames over
2.5 s (gait10dof18musc) and 73 frames over 1.2 s (gait2354). Errors are RMS over the frames: lengths per muscle and
moment arms per scored muscle–coordinate pair in mm, joint moments per coordinate in N·m per kg of body mass.

| Model | Converter | Length: mean (mm) | Moment arm: mean (mm) | Joint moment: mean (N·m/kg) | W1 / W2 / W3 |
| --- | --- | --- | --- | --- | --- |
| gait10dof18musc | MyoConverter (Station 0, MuJoCo 2.3.7) | 5.27 | 7.61 | 0.0075 | — |
| | run1: after Station 4 | 0.255 | 0.641 | 0.0047 | fail / fail / pass (iliopsoas, both sides) |
| | run2_robust: after change 5.1 | 0.126 | 0.517 | 0.0047 | pass / pass / pass |
| | run3_timestep: after 5.2 | 0.126 | 0.517 | 0.0047 | pass / pass / pass (identical to run2) |
| | run4_switch: after 5.3 (first version) | 0.126 | 0.511 | 0.0048 | pass / pass / pass |
| | run5_pathtol: after 5.3 and 5.4 | 0.126 | 0.511 | 0.0048 | pass / pass / pass |
| | run6_final: after Station 6's changes | 0.126 | 0.511 | 0.0047 | pass / pass / pass |
| | **run7_damping: the final converter** | **0.126** | **0.511** | **0.0047** | **pass / pass / pass** |
| gait2354 | MyoConverter (Station 0, MuJoCo 2.3.7) | 1.64 | 1.20 | 0.0039 | — |
| | run1: after Station 4 | 0.236 | 0.220 | 0.0027 | fail (4) / fail (14) / pass |
| | run2_robust: after 5.1 | 0.130 | 0.167 | 0.0027 | fail (4) / fail (12) / pass |
| | run3_timestep: after 5.2 | 0.130 | 0.167 | 0.0027 | fail (4) / fail (12) / pass (identical to run2) |
| | run4_switch: after 5.3 (first version) | 0.050 | 0.109 | 0.0027 | pass / pass / pass |
| | run5_pathtol: after 5.3 and 5.4 | 0.050 | 0.109 | 0.0027 | pass / pass / pass |
| | run6_final: after Station 6's changes | 0.050 | 0.109 | 0.0027 | pass / pass / pass |
| | **run7_damping: the final converter** | **0.050** | **0.109** | **0.0027** | **pass / pass / pass** |
| Rajagopal2016 | MyoConverter (Station 1a port, MuJoCo 3.15) | 14.48 | 1.215 | 0.0112 | — |
| | run1 | — | — | — | W0 fail (the check, below) |
| | run2 | 0.368 | 0.217 | 0.0014 | not evaluated (MyoConverter column from run1's harness) |
| | run3: before change 6.10 | 0.368 | 0.217 | 0.0014 | pass / pass / pass |
| | **run4_damping: the final converter** | **0.368** | **0.217** | **0.0011** | **pass / pass / pass** |

Worst items in run5: length rect_fem_l (gait10 0.48 mm, gait2354 0.41 mm); moment arm vasti_l / vas_int_l about the
knee (2.2 mm RMS, 4.5% of its peak), the same in every run since Station 3 (the knee's moving path points); joint
moment hip flexion (0.0103 / 0.0086 N·m/kg, 1.7% / 1.2% of OpenSim's peak), about 80% of MyoConverter's. W0 passed
in every run, in both columns.

run6_final differs from run5_pathtol only where Station 6's change 6.3 gave the toes their own OpenSim inertia
(gait10dof18musc's joint moments 0.0048 → 0.0047 N·m/kg); paths are unchanged. run7_damping is the final converter
(change 6.10 removed the template's joint damping on joints a coupling drives; the gait models' numbers don't move at
this precision).

## Rajagopal 2016

Checked under the addendum and the two amendments in `station5/GATES.md`. The trial is opensim-models' Rajagopal
pipeline, rebuilt by `station5/make_rajagopal_trial.py` (OpenSim's own scale factors and IK setup; 136 frames,
0.45–1.8 s); the MyoConverter column is the Station 1a port's conversion.

- **run1** failed W0 on the check itself: the patella couplers' coordinates (knee_angle_beta) have no MuJoCo joint
  (the converter reproduces them through helper joints, change 2.4), and the IK file's column for them satisfies the
  coupler only to 2e-9. The amendment of 2026-10-13 (written before any W1–W3 number) makes coupled coordinates
  follow their constraint along the trial, as Stations 2 and 3 already did, and carries their inverse-dynamics force
  to the knee by the coupler's derivative, which is what the MuJoCo side's projection does.
- **run2**: ours passed W0, but the MyoConverter column was still the one run1 had stored with the old harness, so
  W1–W3 weren't evaluated. **run3** recomputed it.
- An earlier run of the amended check on an intermediate conversion (before the last change to the force refit,
  which doesn't affect W0–W3) was stopped when the converter changed and is not used (its W0 passed).

run4_damping (the final converter): lengths 0.37 mm mean RMS (MyoConverter 14.5 mm; its vasti wrap the wrong way
round the femoral condyles, 237 mm off), moment arms 0.22 mm (1.22 mm), joint moments 0.0011 N·m/kg (0.0112; worst
ours knee_angle_l 0.0027; MyoConverter's knee 0.058, carrying Station 2's 15.6 mm knee error). run3, before change
6.10, had 0.0014 (worst 0.0045): the template's joint damping on the patella's coupled joints added torque that
OpenSim doesn't have. The largest remaining errors are
the gastrocnemii (8.5 mm length, 10.9 mm knee moment arm), whose path wraps over two cylinders in a row in OpenSim
(Station 3 FINDINGS).

## What was wrong

Every miss in run1 was a hip flexor (iliopsoas in gait10dof18musc; iliacus and psoas, and a few gracilis pairs, in
gait2354). Each has a conditional path point that OpenSim uses only while hip flexion is between −90° and 45°. Walking
stays inside that range, so in walking OpenSim's point is simply a fixed point. Station 3's fit (change 3.1) moves the
point with hip flexion as one quartic, fitted with least squares over the whole hip range (−120° to 120°), where
OpenSim's path jumps where the point switches on and off (by 19.5 mm at −90°). The squared error of those jumps
pulled the quartic away from OpenSim's path everywhere, including the walking range: 1.2 mm of length error for
iliopsoas in walking, where MyoConverter's own model (which moves the point between its neighbours) was 0.2 mm off.

## The changes

Each change was made alone and both models were rerun after it, through every station it can affect.

- **5.1 Robust fit** (`xml/path_points/utils.py`). The conditional point is fitted with a soft-L1 loss with a 1 mm
  scale instead of least squares, so residuals far beyond 1 mm (the jumps) count roughly linearly and stop pulling the
  fit elsewhere. Candidates (the fit, upstream's model, the point fixed) are compared on the same objective.
  gait10dof18musc then passes W1–W3 (iliopsoas length 1.24 → 0.07 mm); gait2354 still fails on iliacus and psoas
  (length 1.0–1.2 mm, budget 0.6). Station 3 still passes (gait10 mean 0.91 → 0.81 mm). Station 4: gait10's
  iliopsoas_l fails F1 by 0.04% of Fmax (4.24% vs MyoConverter 3.70%; see 5.4).
- **5.2 Time step** (`conversion_steps/O2MStep2.py`, `O2MStep3.py`). MyoConverter's analysis functions set
  `opt.timestep` on the model they share with the conversion step (0.001 s in Step 2, 0.005 s in Step 3), and the
  value leaked into the saved cvt2 and cvt3; the template intends MuJoCo's 0.002 s. The steps now save the time
  step the model was loaded with. Only the `timestep` attribute of cvt2/cvt3 changes (checked by diffing every file
  of a full run against run2); every Station 2–5 number is identical. It matters in simulation: with MuJoCo's
  default (semi-implicit Euler) at 0.005 s, gait10dof18musc's first 50 ms of the walking replay below end about 105°
  from a converged simulation of the same model (RK4 at 0.5 ms; gait2354: 2–3°); at 0.002 s, 0.4° without ground
  forces and 6° with them (gait2354: under 1°).
- **5.3 Switch toward the neighbouring point** (`xml/path_points/utils.py`, `ConditionalPathPoint.py`, `xml/utils.py`,
  `optimization/utils/UtilsMujoco.py`). Two parts:
  - Samples where OpenSim's point is active weigh 10 times more in the fit (in the squared terms). There OpenSim's
    path is a fixed point, which a moving point can reproduce; the inactive side ends in a jump that no continuous
    point can follow.
  - A new candidate, when the point's neighbour on the path is a plain path point on the same body (at A): the point
    moves on the line from its own location C to A, P(q) = C + (A − C)·s(q). Where s = 1 the point sits on A and the
    local path is exactly OpenSim's straight segment, so s only has to switch between 0 (active) and 1 (inactive).
    A switch is too sharp for one quartic, so s is two quartics in sequence through a helper joint, as change 2.4
    does for the patella: helper h = p1(q), s = p2(h), fitted on the same weighted robust objective from the quartic
    closest to OpenSim's on/off pattern and three seeded random starts.
  - The helper sits on its own body under the world body, like change 2.4's. Its scale is free (the coupling is the
    same function for any scale), and it matters in dynamics: the body has the template's minimum inertia (0.001 kg
    m²) and the default joint damping, which reach the hip as (dh/dq)². The helper is scaled so |dh/dq| ≤ 0.1: at most
    1e-5 kg m² and 5e-4 N·m·s/rad added to the hip. A first version used 0.01; the chained-coupling motion test then
    failed for Rajagopal's patella (it lagged its spline by 2.4°, because a helper that is too cheap to move lets
    MuJoCo's soft equality on a heavier joint give way); with 0.1 it lags 0.10° (unscaled 0.08°). gait2354's walking
    joint moments are the same with slopes 0.01, 0.1 and 1 (to 3e-5 N·m/kg).
  - MyoConverter's `dependencyJointAng`, which poses the model in Steps 2–3, now follows chains of joint equalities
    (models without chains give exactly the previous result).
  - The candidates are compared on the weighted objective. The switch is chosen for iliopsoas (gait10), iliacus,
    psoas and medial gastrocnemius (gait2354); the single quartic stays for the others. gait2354 then passes:
    iliacus length 1.18 → 0.04 mm, every pair within budget. Station 3 still passes (means 0.81 → 0.86 mm and
    0.32 → 0.36 mm; MyoConverter 7.58 / 1.54 mm), Station 2 too (bodies identical).
    Station 4: gait10's iliopsoas_l fails F1 by more (4.55% vs 3.70%).
- **5.4 The force refit leaves out poses where the path is wrong** (`optimization/utils/force_refit.py`). Why
  iliopsoas_l failed F1: the hip's range is −120° to 120°, and beyond −90° (134 of the 1410 Station 2 poses; no
  human hip extends that far) OpenSim's point switches off and its path jumps by 19.5 mm. The converted path keeps the
  point on there (MuJoCo can't switch it off), so it is 19–21 mm off, and Station 4's refit (change 4.1), trained on
  random poses over the same range, bent the force–length curve to make up for those poses, at the cost of every
  other pose. Now training poses where the converted path is more than 5 mm from OpenSim's are left out (only the
  hip flexors have any: 48 of 400 for iliopsoas, 77–80 for iliacus and psoas; nothing else changed, checked muscle
  by muscle). Over the poses with hip flexion at −90° or more, the force error then falls from 3.6% to 2.3% of Fmax
  at full activation and from 2.7% to 0.6% when passive (gait10); MyoConverter is at 3.0% and 10.8%. Beyond −90° it
  rises (9.6% → 17.4%). Over all poses, two muscles now miss the F1/F2 tolerance (gait10 iliopsoas_l at full
  activation, 5.78% vs 3.70%; gait2354 iliacus_r passive, 4.42% vs 3.20%), in every case only because of the poses
  beyond −90°. They are listed with that reason (`station4/<model>/f_reasons.json`), as the gates allow, backed by
  `station4/tests/test_off_path_poses.py`: OpenSim's path jumps by more than 10× its largest change elsewhere; ours
  is more than 10 mm off in every pose beyond −90°; and over the other poses ours is within the tolerance (in fact
  better than MyoConverter for all six hip flexors, both activations). Without 5.4 the same test fails: iliopsoas_l
  stays 0.6% worse than MyoConverter even over the poses at −90° or more.

## Level 3: forward replay (reported only)

Both simulators start from the same state of the walking trial and get the same muscle excitations (OpenSim's CMC
result, open loop) and the same ground reaction forces; the table gives the time until any hip, knee or ankle angle
differs by more than 5° between OpenSim and MuJoCo, for the model's own time step and integrator ("as converted")
and for RK4 at 0.5 ms ("fine").

| Model | Column | With ground forces: as converted / fine | Without: as converted / fine |
| --- | --- | --- | --- |
| gait10dof18musc | MyoConverter (0.005 s) | 3 ms / 9 ms | 13 ms / 25 ms |
| | run2_robust (0.005 s, before 5.2) | 3 ms / 7 ms | 12 ms / 28 ms |
| | **run5_pathtol (0.002 s)** | **12 ms / 7 ms** | **25 ms / 28 ms** |
| gait2354 | MyoConverter (0.005 s) | 10 ms / 21 ms | 20 ms / 27 ms |
| | run2_robust (0.005 s, before 5.2) | 10 ms / 21 ms | 17 ms / 42 ms |
| | **run5_pathtol (0.002 s)** | **17 ms / 21 ms** | **41 ms / 43 ms** |

All columns, MyoConverter's included, part from OpenSim within tens of milliseconds, nearly always first at the
ankle. That is expected for an open-loop replay (GATES.md) and is mostly not the converter:
- The light foot is very sensitive: at the first frame the ankle accelerates at about 9000 rad/s² in MuJoCo, so a
  small difference in force becomes degrees within milliseconds.
- With muscles switched off on both sides (skeleton, gravity and ground forces only), the replays still part after
  17–22 ms (the feet spin), so the multibody side alone diverges this fast.
- Without ground forces, OpenSim and MuJoCo are about 14° apart after 50 ms whatever MuJoCo's integrator: that is
  the muscle models (MuJoCo's rigid-tendon muscle vs OpenSim's Millard equilibrium muscle), which no conversion of
  parameters can make identical in dynamics.
- Change 5.2 makes the as-converted simulation as accurate as the fine one (after 50 ms with ground forces, gait10:
  96° → 6° from OpenSim, fine 8°).

Joint damping (0.05 N·m·s/rad on every joint, from MyoConverter's template) and the template's minimum body inertia
(which raises the toes' inertia from 1e-4 to 1e-3 kg m²) are deliberate differences from OpenSim, included in W3
above (which uses the model exactly as converted). Without the damping, gait10dof18musc's joint-moment errors would
be lower by at most 5e-4 N·m/kg (knee); the toe inertia accounts for up to about 1.4e-3 N·m/kg.

## The check itself was checked

`station5/tests/test_gait_harness.py` (all pass on the current converter):
- OpenSim side = OpenSim's own InverseDynamicsTool: same generalized forces to 5e-9 N·m (the tool writes 8 decimals).
- Independent cross-check of the MuJoCo side: with OpenSim's knee splines replaced by the MuJoCo model's exact
  quartics, and OpenSim's physics restored in MuJoCo (no joint damping, massless virtual bodies, OpenSim's toe
  inertia), the joint moments agree to 1.6e-5 N·m/kg in every frame; the converted model as it is differs by up to
  0.063, so the cross-check is sensitive.
- Mutations: 10% more mass on tibia_r changes only its ancestors' joint moments; a renamed muscle and a renamed
  joint fail W0.

`station5/tests/test_switch_point.py` (change 5.3; passes on the current converter, fails 7 checks on the one before):
chained equalities followed in any order by `dependencyJointAng`; helper scaling leaves the function unchanged; in
the converted gait2354, every hip-flexor helper is on its own body, driven by its hip, slope ≤ 0.1, the keyframe
satisfies every equality, and `dependencyJointAng` poses all 50 driven joints exactly as Station 3's solver; in
motion (hip driven at 1 Hz), iliacus and psoas stay within 0.3 mm of their exact-pose lengths, as without the switch.

## Deviations and open items

- **gait2354 was not a hold-out for 5.3.** The active weight (10) was chosen after seeing both models' walking
  results. The weight matters but is not knife-edge (switch candidate, right side, local path only, walking RMS):
  weight 1 leaves gait2354's iliacus 0.45 mm / 1.08 mm (length / adduction moment arm, budgets 0.61 / 0.49) and
  fails; 3, 10 and 30 give 0.06 / 0.12, 0.04 / 0.06 and 0.01 / 0.01. Higher weights trade accuracy on random poses
  (beyond the walking range) for accuracy in the active range: random-pose length RMS 7.4, 8.0, 8.7, 9.6 mm for 1, 3,
  10, 30.
- **5.3 was rerun after a fix.** The first version (helper slope 0.01; `station5/*/run4_switch*`,
  `station3/*/run6_switch*`, `station2/*/run6_switch.json`) passed every Station 2, 3 and 5 gate, but the
  chained-coupling motion test (Station 2) then failed for Rajagopal's patella (above). The final version (slope
  0.1) was rerun through Stations 2–5 (`run7_switch`, `run6_switch` for Station 4, `run5_pathtol` here).
- **Two Station 4 misses are listed with a reason** (5.4). This is allowed by the gates, but it is a judgement:
  the converter is now better than MyoConverter in every pose a human hip can reach and worse in poses it can't.
  Open for Wafi to reverse (removing `f_reasons.json` turns them back into failures).
- **Level 3 harness fix.** When an open-loop replay blew up in MuJoCo 2.3.7 (MyoConverter's column, gait10), MuJoCo
  reset the whole state including the time, and the replay loop started over forever. The replay now stops at
  MuJoCo's instability warning (or any time reset) and reports "unstable at …". Found before any level 3 number was
  reported; replays that never became unstable give the same result either way.
- Level 3 shows that open-loop replay can't validate muscle dynamics at this level of detail; a closed-loop
  comparison (e.g. tracking the trial with a controller in both simulators) would be the next step, not done. So
  force–velocity behaviour, and the MuJoCo optimal length L0 that scales it (Station 4 open item), are still not
  validated.
