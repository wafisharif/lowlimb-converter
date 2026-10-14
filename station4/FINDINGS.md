# Station 4: muscle forces

**Result: both gait models and Rajagopal 2016 pass F0, F1 and F2 (with the dated F0 amendment, below). Muscle
forces are now 3–5× (gait models) and 2.8× (Rajagopal 2016) closer to OpenSim's than MyoConverter's at full
activation, and 5–10× and 6.5× closer when passive. Two hip-flexor misses in the gait models are listed with a
test-backed reason (each comes only from poses with the hip extended beyond −90°, where OpenSim's own muscle path
jumps), and two in Rajagopal 2016 (each only against MyoConverter's luckier copy of a muscle whose mirror image it got
far worse; section below).**
Gates and method: `station4/GATES.md` (posted to the plan doc before any Station 4 code existed). Per-run results:
`station4/<model>/<run>.json` and `<run>_f.json`.

Each muscle was held still in the Station 2 poses (1410 / 1943) at activation 0, 0.25, 0.5 and 1, and its static force
compared with OpenSim's equilibrium tendon force. Errors are RMS over the poses, as % of the muscle's max isometric
force (Fmax).

| Model | Converter | a = 1: mean / worst (% Fmax) | a = 0: mean / worst | Muscles within 5% (a = 1) | F1 / F2 |
| --- | --- | --- | --- | --- | --- |
| gait10dof18musc | MyoConverter (Station 0, MuJoCo 2.3.7) | 10.1 / 19.9 | 13.2 / 28.2 | 5 / 18 | — |
| | run1: after Station 3 | 9.6 / 18.6 | 13.4 / 33.6 | 4 / 18 | fail / fail |
| | run3: after changes 4.1–4.2 | 2.9 / 4.1 | 1.2 / 3.1 | 18 / 18 | pass / pass (16 better, 2 same) |
| | run4: after change 4.3 | 3.0 / 4.1 | 1.2 / 3.0 | 18 / 18 | pass / pass (16 better, 2 same) |
| | run5_robust: after Station 5 change 5.1 | 3.0 / 4.2 | 1.2 / 3.3 | 18 / 18 | fail (iliopsoas_l by 0.04%) / pass |
| | run6_switch: after 5.3 | 3.0 / 4.6 | 1.2 / 3.5 | 18 / 18 | fail (iliopsoas_l) / pass |
| | run7_pathtol: after 5.4 | 3.2 / 5.8 | 1.3 / 4.5 | 16 / 18 | pass with 1 listed miss / pass |
| | **run8_guard: after 4.4 (current)** | **3.1 / 5.9** | **1.3 / 4.7** | **16 / 18** | **pass with 1 listed miss / pass** |
| gait2354 | MyoConverter (Station 0, MuJoCo 2.3.7) | 16.1 / 120.9 | 15.7 / 102.3 | 28 / 54 | — |
| | run1: after Station 3 | 15.4 / 117.2 | 16.3 / 109.9 | 29 / 54 | F0 fail (as gated); with the amendment fail / fail |
| | run3: after changes 4.1–4.2 | 3.5 / 19.3 | 2.8 / 22.2 | 46 / 54 | pass / pass (52 / 46 better, rest same) |
| | run4: after change 4.3 | 3.5 / 19.3 | 2.9 / 22.2 | 46 / 54 | pass / pass (51 / 47 better, rest same) |
| | run5_robust / run6_switch: after 5.1 / 5.3 | 3.5 / 19.3 | 2.9 / 22.2 | 46 / 54 | pass / pass |
| | run7_pathtol: after 5.4 | 3.5 / 19.3 | 2.9 / 22.2 | 46 / 54 | pass / pass with 1 listed miss |
| | **run8_guard: after 4.4 (current)** | **3.6 / 17.6** | **2.9 / 23.1** | **44 / 54** | **pass / pass with 1 listed miss** |
| Rajagopal2016 | MyoConverter (Station 1a port, MuJoCo 3.15) | 6.9 / 37.6 | 6.9 / 35.9 | 37 / 80 | — |
| | run1: before 4.4 | 3.1 / 6.8 | 1.4 / 11.3 | 71 / 80 | fail (11) / fail (2) |
| | run2_guard: first version of 4.4 | 19.1 / 731.9 | 18.0 / 742.7 | 72 / 80 | fail (4) / fail (3) |
| | run3_final: 4.4 with its safeguard | 2.5 / 7.1 | 1.1 / 4.6 | 74 / 80 | fail (2) / fail (1) |
| | run4_final: Step 3 kept only if it dominates | 2.5 / 7.1 | 1.1 / 4.6 | 74 / 80 | pass with 2 listed misses / pass |
| | **run5_damping: the final converter** | **2.5 / 7.1** | **1.1 / 4.6** | **74 / 80** | **pass with 2 listed misses / pass** |

"Same" means within 0.5% of Fmax of MyoConverter. Activations 0.25 and 0.5 are in the result files (gait models:
mean 1.9–2.6%; Rajagopal 2016: 1.7 / 2.3%). run8_guard (gait models) and run4_final (Rajagopal 2016) are the current
converter for the gait models (their final conversions differ from run8_guard's only in joint damping, change 6.10,
which static force doesn't see) and run5_damping for Rajagopal 2016. Listed misses are in each run's `_f.json` (the
run script reads `f_reasons.json`); for run4_final they were added after the run and scored in
`Rajagopal2016/run4_final_listed_f.json`.

## Rajagopal 2016, and change 4.4

Checked under the addendum in `station4/GATES.md`: the MyoConverter column is the Station 1a port's own conversion.
2189 poses, 80 Millard muscles.

**What was wrong (run1).** F1 failed on 11 lower-leg muscles (soleus, tibialis posterior and anterior, the long toe
extensors and flexors, peroneus longus) at full activation, by 0.5–1.4% of Fmax; F2 on iliacus (both sides). Rajagopal's
lower-leg muscles have long, compliant tendons: as activation rises the tendon stretches and the fibre's
force–length curve moves along the muscle–tendon length. A MuJoCo muscle has a rigid tendon, so one curve can't match
OpenSim at every activation. The refit (changes 4.1–4.3) fitted all four activation levels with equal weight, which
gave up some accuracy at full activation; MyoConverter's Step 3 fits full activation only. The iliacus misses were a
path error (change 3.5, Station 3): in a few poses the converted path went the long way round the pelvic brim.

**Change 4.4** (`src/myoconverter/optimization/utils/force_refit.py`): both ends of the activation range get a
guarantee. The same least squares is also run on the full-activation samples alone and on the passive samples alone;
while the combined fit is more than 0.25% of Fmax worse than that at either end (on the training poses), that end's
residuals get twice the weight, up to 256×. A muscle that already meets both (bifemsh in both gait models; sartorius and
vastus intermedius in gait2354) keeps exactly its old parameters. Two safeguards came from runs: the first version
(run2_guard) gave gastrocnemius lateralis lmax at its lower bound, so its passive force exploded just outside the fitted
lengths (600–700% of Fmax off; its 30% of training poses with the path a few mm off had been left out by change 5.4). A
weighted fit is now kept only if its largest error over every training sample with an OpenSim force, including those
left-out poses, is within 25% of Fmax of the equal-weight fit's; otherwise the equal-weight fit is used. And Step 3's
particle-swarm result, which used to be kept when its plain sum over the levels was lower, is now kept only if it is at
least as good at every level (run3_final kept it for extensor digitorum longus left, whose right side got the refit).
Test: `station4/tests/test_level_guard.py`.

On the gait models 4.4 trades sub-maximal accuracy for the ends: gait10dof18musc at full activation 3.2 → 3.1%, at
0.25 and 0.5 1.7 → 1.9% and 2.0 → 2.2%; gait2354 3.5 → 3.6% at full activation (the mean moved with two short
hip rotators, quadratus femoris and gemelli, whose rigid-tendon fit is 9–23% off at every level), 2.3 → 2.5% and
2.3 → 2.6% in between. Both still pass F1 and F2.

**The two listed misses** (run4_final and run5_damping, identical): vastus lateralis right (1.98% vs MyoConverter's
1.45% of Fmax) and soleus left (2.30% vs 1.80%), at full activation. MyoConverter's Step 3 is unseeded, so its error for
one muscle is a single draw: its mirror-image copies of these muscles, identical in OpenSim (same parameters, path
points on the mirrored bodies at the mirrored locations), score 7.87% and 2.31% in the same conversion. Ours are
deterministic and symmetric (1.98 / 1.99% and 2.30 / 2.31%) and within the tolerance of the mean of MyoConverter's two
copies. `station4/tests/test_mirror_muscles.py` checks all of this (numbers in `Rajagopal2016/mirror_muscles.json`); the
reasons are in `Rajagopal2016/f_reasons.json`.

## Station 5 changes: the hip flexors

Station 5 refitted the conditional path points of the hip flexors (iliopsoas; iliacus and psoas) so that they follow
OpenSim's path where the point is on, which includes all of walking (changes 5.1, 5.3; `station5/FINDINGS.md`). The
cost is in hip extension beyond −90°, where OpenSim's point switches off and its path jumps by about 20 mm: the
converted path keeps the point on there, about 20 mm off. That is 134 of the 1410 / 1943 Station 2 poses (the hip's
range is ±120°), and it raised gait10's iliopsoas_l above MyoConverter's at full activation (F1). The refit of change
4.1 trained on random poses over the same range and bent the force–length curve to make up for those poses, at the
cost of all the others. Change 5.4 leaves training poses where the converted path is more than 5 mm off out of the
refit (only the six hip flexors have any). Result, over the poses with hip flexion at −90° or more, every hip flexor
is now better than MyoConverter at both activations (iliopsoas_l: 2.3% vs 3.0% of Fmax active, 0.6% vs 10.8%
passive; was 3.6% and 2.7%), and worse beyond −90° (17% vs 7.5%). Over all poses two muscles miss the tolerance:
gait10 iliopsoas_l active (5.78% vs 3.70%) and gait2354 iliacus_r passive (4.42% vs 3.20%). Both are listed in
`station4/<model>/f_reasons.json` with the reason and the test that shows it, `station4/tests/test_off_path_poses.py`
(OpenSim's path jumps at −90°; ours is more than 10 mm off in every pose beyond; within the tolerance, in fact better
than MyoConverter, over all the other poses; the same test fails without change 5.4). This is a judgement that the
gates allow; it is open for Wafi to reverse.

## What was wrong

MyoConverter's Step 3 fits four parameters of each MuJoCo muscle (lmin, lmax, fpmax and a peak-force scale) with a
particle swarm, at full activation only, after setting the muscle's operating range from a rough estimate (tendon at
slack length, no pennation). Three problems:

- **It is fragile.** Station 2's geometry changes moved some muscle lengths by tenths of a millimetre, and gluteus
  maximus's passive force error went from 2% to 34% of Fmax (gait10, run1). Left and right copies of the same muscle
  end up with different parameters (e.g. vasti_l 4.2% vs vasti_r 14.5% training error).
- **It never looks at passive or sub-maximal force,** and the operating range it sets is never fitted. Passive errors
  average 13–16% of Fmax.
- **It can't fit muscles whose force shape MuJoCo can't follow:** quadratus femoris, gemelli and the internal obliques
  in gait2354 are off by 59–121% of Fmax (RMS).

## The changes

- **4.1 Deterministic refit** (`optimization/utils/force_refit.py`, called at the end of Step 3 in
  `conversion_steps/O2MStep3.py`). After the particle swarm, every muscle's static-force parameters (operating range,
  lmin, lmax, fpmax, peak force) are refitted with least squares to OpenSim's equilibrium force at activations 0,
  0.25, 0.5 and 1, on 400 training poses of the coordinates that change the muscle's length (seed 2, not the Station
  2–4 poses), from four starting points (Step 3's result and three fixed ones; see 4.3). Step 3's result is scored on
  the same objective and kept if better
  (it never was: the refit won for all 18 + 54 muscles). The MuJoCo force formula used for fitting is a vectorised copy of
  MuJoCo's own (`mju_muscleGain`/`mju_muscleBias`), checked equal to 3e-16 of Fmax. Adds 11–13 s (gait10dof18musc) and 70–83 s (gait2354) to a conversion (2 cores).
- **4.2 Train only on valid equilibria.** Training samples where OpenSim has no equilibrium (the fiber clamped at its
  minimum length while the tendon pulls, see the amendment) are left out, as in the check itself. Effect ≤ 0.03% Fmax.
- **4.3 Results that don't depend on the particle swarm, and the same for left and right** (`force_refit.py`).
  Converting gait10dof18musc with the new `osim2mujoco` command (MyoConverter's validation plots off) gave
  different muscle parameters from the station runs (plots on), with the same static errors (within 0.0005% of Fmax).
  The cause: Step 3's particle swarm is unseeded, so its result can change from run to run (it did between two
  tool runs, and between the tool and the station runs, although repeated station runs had matched), and the
  refit's first starting point was that result; static force leaves some parameters free (below), so
  different starts end in different, equally good, parameter sets. The same thing made left and right copies of a
  muscle differ. The refit now starts only from fixed points, and a weak penalty (a parameter 1 unit from the
  estimate costs as much as 1% of Fmax of RMS error) pulls it towards a rigid-tendon estimate: normalised length =
  (tendon length − tendon slack length) / (optimal fiber length × cos pennation), MuJoCo's default lmin, lmax and
  fpmax, peak force = Fmax. The weight was chosen on both models (no hold-out) from 0, 0.003, 0.01 and 0.03, as the
  value that made every limb muscle's left and right copies identical without moving any mean error by more than
  0.06% of Fmax (0.03 failed F1 on gait10dof18musc). Effect: means within 0.06% of Fmax of run3 (largest per-muscle
  change: internal obliques, −2.6% at a = 1 and +1.1% at a = 0); left/right pairs with different parameters 4 → 0 of 9
  (gait10dof18musc) and 15 → 3 of 27 (gait2354; see below).

The work container was lost on 2026-10-11 before Station 4 was committed; runs 1–3 (and the amended run1) were redone
from scratch in a new container from the same code and reproduced every number in this file and in the logs. Since
change 4.3 the result no longer depends on the particle swarm: a tool conversion whose Step 3 result differed from
run4's (e.g. hamstrings_r training error 10.61% vs 8.22% of Fmax) gave a cvt3 identical to run4's, apart from a
`content_type` attribute MuJoCo's XML writer adds to mesh assets in one of the two.

## Gate amendment

run1 failed F0 on gait2354 for a reason outside both converters: at some extreme poses (1.3% for soleus_l at full
activation) a muscle is shorter than its tendon slack length plus its minimum fiber length, OpenSim clamps the fiber,
and it reports a tendon force that its fiber doesn't produce, so there is no static OpenSim force to compare with. A
dated amendment (in `GATES.md`, the original text unchanged, posted to the plan doc) leaves those samples out of both
columns. Its first wording also dropped clamped samples of slack muscles, where both forces are zero and valid
(13.9% of gait10's samples); that was corrected the same day, before the amendment was used to gate anything.
Final effect: 0 samples left out in gait10dof18musc, 288 of 419,688 (0.07%) in gait2354.

## What is still off

- **Muscles with very short fibers:** quadratus femoris (19–22% of Fmax at a = 0 and 1), gemelli (11–13%) and the
  internal obliques (12–13%) in gait2354. Their OpenSim passive force rises far more steeply than MuJoCo's passive curve
  (quadratic, then linear) can, so no parameter choice fits it. They were at 59–121% before.
- **Rigid tendons:** MuJoCo force is linear in activation and OpenSim's isn't, so errors at a = 1 (3.0–3.5% mean) are
  larger than at a = 0.25–0.5 (1.6–2.2%). An elastic-tendon muscle would need a custom MuJoCo plugin (Station 0).
- **Static only:** force–velocity behaviour is not checked here, and Station 5 couldn't check it either (its
  open-loop replay parts from OpenSim within milliseconds for reasons that aren't force–velocity; station5/FINDINGS.md).
- **Static force doesn't pin down every parameter.** In the poses a muscle can reach, many parameter sets give the
  same static force. Before change 4.3, left and right copies of 4 of 9 muscles (gait10dof18musc) and 15 of 27
  (gait2354) ended up with different parameters although their static errors agreed: e.g. glut_max1's optimal length
  L0 (MuJoCo: lengthrange width / range width) was 0.86 m on the right and 0.24 m on the left. Change 4.3 resolves the
  free directions towards a rigid-tendon estimate; the 3 pairs that still differ (erector spinae, internal and
  external obliques) are trunk muscles whose training data really differ: both copies are moved by the same lumbar
  coordinates, and the same random lumbar values bend the trunk towards one copy and away from the other. L0 matters
  beyond static force: it also scales MuJoCo's force–velocity curve. L0 relative to OpenSim's optimal fiber length ×
  cos(pennation) is now 1.12–1.73 (gait10dof18musc, median 1.24) and 1.41–2.35 (gait2354, median 1.66); before 4.3
  1.06–6.10; MyoConverter 0.44–2.53. A ratio above 1 is expected with an elastic tendon (part of a length change goes
  into the tendon), but these values aren't validated (still open after Station 5).
- Station 3's open item (hip-flexor lengths 0.3–0.5 mm worse than MyoConverter) was closed by Station 5 (above).

## The check itself was checked

- **F0 on every run:** all OpenSim forces finite; equilibrium residual within 1e-6 Fmax on ≥ 99% of the samples that
  have an equilibrium.
- **Independent cross-check** (`station4/tests/test_forces_harness.py`): the harness's MuJoCo force equals MuJoCo's own
  muscle functions at the actuator length exactly (difference 0), and the converter's vectorised copy agrees to
  3e-16 Fmax. OpenSim's held activation (0.01 when asked for 0) is what MuJoCo gets.
- **Mutations:** raising one muscle's peak force 10% changes only that muscle's errors (its signed error at a = 1
  rises by 4.8% of Fmax); renaming a muscle fails F0.
- Stations 2 and 3 are unaffected (the refit only touches force parameters).
