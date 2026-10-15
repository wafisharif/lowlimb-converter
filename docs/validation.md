# Validation

## Method

The converter was developed in stages ("stations"), with each station testing more of the model than the last. Pass/fail criteria
for each station were written and published before any results existed. When a station failed, the fix went in
as a separate change and the station was rerun; failed runs stay in the record. Criteria that changed after seeing results
get a dated amendment. Every run is logged in {repo}`EXPERIMENTS.md`.

Results are always compared with MyoConverter on the same model, poses, and comparison code.

| Station | Checks | Criteria |
| --- | --- | --- |
| 0 | MyoConverter reproduces its own published results | {repo}`baseline/GATES.md` |
| 1 | It runs on current OpenSim/MuJoCo/NumPy, and failures can be predicted | {repo}`station1/GATES.md` |
| 2 | Bones are in the same place in every pose | {repo}`station2/GATES.md` |
| 3 | Muscle lengths and moment arms | {repo}`station3/GATES.md` |
| 4 | Static muscle forces | {repo}`station4/GATES.md` |
| 5 | Lengths, moment arms and joint moments along real walking trials | {repo}`station5/GATES.md` |
| 6 | MyoConverter's documented limitations, on all 56 OpenSim example models | {repo}`station6/GATES.md` |

Models: gait10dof18musc (10 DOF, 18 muscles), gait2354 (23 DOF, 54 muscles), Rajagopal 2016 (80 leg muscles, 40 wrap
cylinders), RajagopalLaiUhlrich2023 (Station 2 only), and for Station 6 every model in
[opensim-models](https://github.com/opensim-org/opensim-models) at `d9b05d4`.

## Results

MyoConverter's numbers in brackets. Lower is better.

### Bones: worst position error, 1,400–2,200 poses

| Model | Error |
| --- | --- |
| gait10dof18musc | 0.22 mm (0.88) |
| gait2354 | 0.22 mm (0.89) |
| Rajagopal 2016 | 0.15 mm, 0.04° (15.6 mm, 0.89°) |
| RajagopalLaiUhlrich2023 | 0.42 mm, 0.07° (MyoConverter can't convert it) |

Limit: 1 mm and 0.5°. All pass. {repo}`station2/FINDINGS.md`

### Muscle paths: mean moment-arm error

| Model | Error |
| --- | --- |
| gait10dof18musc | 0.86 mm (7.58) |
| gait2354 | 0.36 mm (1.54) |
| Rajagopal 2016 | 0.46 mm (1.64) |

No muscle–joint pair may be worse than MyoConverter's beyond a tolerance. All pass. {repo}`station3/FINDINGS.md`

### Muscle forces: mean error, % of max isometric force

| Model | Full activation | Passive |
| --- | --- | --- |
| gait10dof18musc | 3.1% (10.1) | 1.3% (13.2) |
| gait2354 | 3.6% (16.1) | 2.9% (15.7) |
| Rajagopal 2016 | 2.5% (6.9) | 1.1% (6.9) |

No muscle may be worse than MyoConverter's beyond a tolerance unless there is a tested reason. Four muscles are listed
as exceptions: two hip flexors, which only miss with the hip extended past −90° where OpenSim's own path jumps, and two
Rajagopal muscles, which only miss against MyoConverter's lucky copy of a muscle whose mirror image it got much worse.
{repo}`station4/FINDINGS.md`

### Walking: OpenSim's example trials

| Model | Muscle length | Moment arm | Joint moments |
| --- | --- | --- | --- |
| gait10dof18musc | 0.13 mm (5.27) | 0.51 mm (7.61) | 0.0047 N·m/kg (0.0075) |
| gait2354 | 0.05 mm (1.64) | 0.11 mm (1.20) | 0.0027 N·m/kg (0.0039) |
| Rajagopal 2016 | 0.37 mm (14.48) | 0.22 mm (1.22) | 0.0011 N·m/kg (0.0112) |

Nothing may be worse than MyoConverter beyond a tolerance, and joint moments must be within 0.02 N·m/kg. All pass.
{repo}`station5/FINDINGS.md`

### All 56 OpenSim models

| Check | Result |
| --- | --- |
| Support report predicts which files convert | 56/56, for both converters |
| Files that convert | 54/56 (MyoConverter 40/56) |
| Bones | pass |
| Mass properties exact | 54/54 (MyoConverter wrong on 32 of 40) |
| Limit forces within 1% of peak | worst 0.22%, 24 forces in 9 files |
| Path actuators and springs | lengths within 0.19 mm, forces within 1.6% |
| Contact geometry within 1e-6 m | worst 3e-14 m, 66 pairs |
| Contact dynamics on unseen drops | fail, see {doc}`limitations` |

{repo}`station6/FINDINGS.md`

Every station can be rerun from scratch ({doc}`developers`), and the quick versions run on GitHub Actions on every
push.
