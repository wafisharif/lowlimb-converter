# Station 3: muscle paths (moment arms and lengths)

**Result: both gait models pass M0 and M1. Every scored muscle–coordinate pair is now as good as or better than
MyoConverter, and the average moment-arm error is 9× (gait10dof18musc) and 4× (gait2354) smaller.** Gates and
method: `station3/GATES.md` (posted to the plan doc before any Station 3 code existed). Per-run results:
`station3/<model>/<run>.json` and `<run>_m1.json`.

Both models were posed in the Station 2 poses (1410 / 1943). 24 / 158 muscle–coordinate pairs were scored (every
pair has a peak moment arm of at least 1 mm).

| Model | Converter | Mean RMS moment-arm error (mm) | Worst pair (mm, % of its peak) | Pairs ≤ 2 mm | Mean RMS length error (mm) | M1 |
| --- | --- | --- | --- | --- | --- | --- |
| gait10dof18musc | MyoConverter (Station 0, MuJoCo 2.3.7) | 7.58 | 69.6, 138% (rect_fem / knee) | 16 / 24 | 5.34 | — |
| | run1: after Station 2 | 7.64 | 69.2, 137% (rect_fem / knee) | 18 / 24 | 5.29 | fail (vasti 2 pairs) |
| | run4: after changes 3.1–3.3 | 0.91 | 3.4, 7% (vasti / knee) | 18 / 24 | 0.59 | pass (14 better, 10 same) |
| | run5_robust: after Station 5 change 5.1 | 0.81 | 3.4, 7% (vasti / knee) | 20 / 24 | 0.74 | pass (14 better, 10 same) |
| | **run7_switch: after 5.3 (current)** | **0.86** | **3.5, 7%** (vasti / knee) | **20 / 24** | **0.82** | **pass** (14 better, 10 same) |
| gait2354 | MyoConverter (Station 0, MuJoCo 2.3.7) | 1.54 | 78.4, 155% (rect_fem / knee) | 142 / 158 | 2.33 | — |
| | run1: after Station 2 | 1.53 | 78.0, 154% (rect_fem / knee) | 144 / 158 | 2.29 | fail (vas_int, grac 4 pairs) |
| | run4: after changes 3.1–3.3 | 0.33 | 4.7, 10% (iliacus / hip flexion) | 146 / 158 | 0.46 | pass (38 better, 120 same) |
| | run5_robust: after Station 5 change 5.1 | 0.32 | 4.7, 10% (iliacus / hip flexion) | 146 / 158 | 0.46 | pass (39 better, 119 same) |
| | **run7_switch: after 5.3 (current)** | **0.36** | **6.3, 14%** (iliacus / hip flexion) | **146 / 158** | **0.51** | **pass** (39 better, 119 same) |

"Same" means within 0.1 mm of MyoConverter. run1 failed M1 because Station 2's (more accurate) knee kinematics made
the old vasti and gracilis path approximations slightly worse; the Station 3 changes fixed that.

**Station 5 changed the conditional path points again** (changes 5.1 and 5.3, `station5/FINDINGS.md`): Station 3's
least-squares fit spread the error of OpenSim's on/off jumps over the whole hip range, including walking, where it
was 1.2 mm off in length (Station 5 gate W1). The fit now follows OpenSim's path closely wherever the point is on
(walking errors 0.04–0.07 mm) and concentrates the error beyond the jump, mostly in hip extension past −90° where no
human hip goes and OpenSim's path jumps by about 20 mm. On Station 3's random poses, which cover the whole range, that
shows as slightly higher means (lengths 0.59 → 0.82 mm and 0.46 → 0.51 mm, moment arms 0.91 → 0.86 and 0.33 →
0.36 mm); M1 still passes with every pair as good as or better than MyoConverter. run6_switch is the first version of
5.3 (helper scale 0.01; identical to run7 to 1e-8 mm).

## What was wrong

The checks pointed at the two kinds of OpenSim path point that MuJoCo has no equivalent for:

- **ConditionalPathPoint** (a point the muscle wraps over only while a coordinate is in some range; e.g. rectus
  femoris over the femoral condyles only in deep knee flexion). MyoConverter either keeps the point fixed and always
  active when it has no "anchor" (another plain point on the same body), or slides it linearly from the anchor to
  its location across the whole coordinate range. Rectus femoris has no anchor, so its path always ran over the
  condyles: at full knee extension (gait10, before Station 3) its length was 82 mm too long and its knee moment
  arm had the **wrong sign** (−58 mm vs OpenSim's +38 mm). That is the 69–78 mm RMS error above, in both models.
- **MovingPathPoint** (a point whose position is a spline of a coordinate, e.g. at the knee). Fitted with a quartic
  through the spline's knots over the knots' whole span, much wider than the knee can
  move (the same mistake Station 2 fixed for the knee joint itself; here the knots span −120° to +120° but the knee
  moves −120° to +10°), and, until Station 2's change 2.2, written with 4 significant figures.

MyoConverter's own Step 2 can't correct either: it only moves wrapping-object sites, and no muscle in these models
uses a wrapping object. Its own check does flag the errors, then skips every flagged group ("No wrapping information
extracted, skipped": 12 of 12 groups in gait10dof18musc, 20 of 20 in gait2354).

## The three changes

Each change was made alone and both models were rerun after it with the full pipeline (`run2_conditional`,
`run3_movingfit`, `run4_smallrange`).

| Change | gait10 mean / worst (mm) | gait2354 mean / worst (mm) |
| --- | --- | --- |
| run1 (none) | 7.64 / 69.2 | 1.53 / 78.0 |
| 3.1 conditional points fitted to OpenSim's path | 1.44 / 6.9 | 0.39 / 6.0 |
| 3.2 moving points fitted over the coordinate's range | 0.91 / 3.4 | 0.33 / 4.7 |
| 3.3 near-fixed axes set mid-range | 0.91 / 3.4 (≤ 0.0001 change) | 0.33 / 4.7 (≤ 0.04 change per pair) |

- **3.1 Conditional path points** (`xml/path_points/ConditionalPathPoint.py`, `xml/path_points/utils.py`,
  `xml/config.py`). The point becomes a moving point P(q), each axis a quartic of its condition coordinate q. MuJoCo
  can't switch a point on and off, but a point on the straight segment between the neighbouring active points
  changes nothing, so P is fitted so that the local path (previous point → P → next point) has OpenSim's length and
  moment arms, about every coordinate that moves those points. The fit uses its own training poses (seed 1:
  300 random poses plus a sweep of q), not the Station 2/3 poses; moment-arm samples straddling an on/off switch are
  left out. Upstream's model and "fixed at its location" are scored on the same objective, and the best is used, so
  it can't be worse than upstream on the training poses. It needs OpenSim at conversion time (already a dependency of
  Steps 2–3). All 8 / 12 conditional points in the two models are fitted this way.
- **3.2 Moving path points** (`xml/path_points/utils.py`). The same fit as Station 2's change 2.3: OpenSim's own spline,
  over the coordinate's range, largest error minimised, full precision. Largest position error per axis is now
  ≤ 0.55 mm.
- **3.3 Near-fixed axes** (`xml/path_points/MovingPathPoint.py`). An axis that moves less than 1 mm gets no joint; it was
  fixed at the minimum of its range and is now fixed mid-range, halving the largest position error. The measured
  effect is negligible (≤ 0.04 mm on any pair, mixed signs). Kept because it is the better constant and it makes the
  left and right gracilis identical (they differed by up to 0.08 mm before).

**Development and hold-out.** The four ways of fitting conditional points that were tried (fitting the point's
position with minimax or least squares, with or without letting it slide along the segment, and fitting the path)
were compared on gait10dof18musc only. gait2354's results for runs 2–4 were first looked at after each change was
final, so gait2354 is a held-out check of choices made on gait10. (run1 of gait2354 was seen before, but it only
shows the problems, not which fix works.)

## What is still off

- **Hip flexors:** iliacus/psoas (gait2354) and iliopsoas (gait10) have the largest remaining errors, 4.2–4.7 mm RMS
  moment arm (≈ 10% of peak; MyoConverter: 8.7–10.9 mm). Their conditional point switches on at −90° hip extension
  with a 23 mm jump in OpenSim's path, which no continuous point can follow.
- **Their lengths got slightly worse:** 0.3–0.5 mm more RMS length error than MyoConverter for these 6 muscles
  (e.g. iliacus_r 5.62 vs 5.17 mm), while their moment arms halved. The fit weighs moment arms and lengths equally;
  Station 4 (force–length) will show whether this matters, and the length weight is the knob if it does.
- **Near the on/off switches** single poses still differ by up to 15 mm (vasti, rectus femoris, at the knee angle
  where OpenSim's path jumps). This is reflected in the RMS numbers above.

## The check itself was checked

- **M0 on every run:** OpenSim's `computeMomentArm` agrees with a finite difference of its length on 100% of
  samples; MuJoCo's finite differences with h and 10h agree on 100%; equality residuals ≤ 5e-16.
- **Independent cross-check** (`station3/tests/test_paths_harness.py`): MuJoCo's analytic tendon Jacobian, with the
  chain rule through every dependent joint's polynomial, matches the harness's finite differences in every pose and
  pair to 4.7e-9 m.
- **Mutations:** moving one path site 5 mm changes only that muscle's errors (length by 4.96 mm); renaming a tendon
  fails M0; the muscle–coordinate pairs match the anatomy (e.g. gastrocnemius: knee and ankle; soleus: ankle only).
- The Station 2 kinematics are unchanged by these changes (every body's errors identical, all three models), and the
  Station 1b support report and its tests were updated to describe the new behaviour (S1, S2, S3 pass; deliberately
  mispredicting the conditional points fails S2).

## Found along the way

- **The pipeline silently reuses old results.** Steps 2 and 3 resume from any `_cvt2.xml` / `_cvt3.xml` already in
  the output folder (`O2MSteps.py`), so rerunning into the same folder after a converter change gives the old model
  back with no error. It cost one invalid run here (`run2` first attempt, discarded and rerun). The Station 1 and 3
  run scripts now refuse a non-empty output folder.
- MyoConverter's Step 2 also reuses cached OpenSim moment-arm data (`<muscle>.pkl`) found in the folder.
