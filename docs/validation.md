# How it's validated

## The method

The converter was built and checked in stations, each a harder test than the last. Each station has **gates**:
pass/fail rules written down, and posted publicly, before any result they apply to was seen. When a gate failed, the
fix was made as its own change and the station was rerun under a new name; failed runs are kept, not overwritten. A
gate that had to change after a result was seen gets a dated amendment saying why. Every run, failed ones included,
is logged in {repo}`EXPERIMENTS.md` (intended, actual, deviation).

Every number is compared with MyoConverter's result for the same model, on the same poses, with the same code.

| Station | Question | Gates |
| --- | --- | --- |
| 0 | Does MyoConverter reproduce its own published results? | {repo}`baseline/GATES.md` |
| 1 | Does it run on a current stack, and can we predict what it will fail on? | {repo}`station1/GATES.md` |
| 2 | Are the bones in the same place in every pose? | {repo}`station2/GATES.md` |
| 3 | Do the muscles take the same paths (lengths and moment arms)? | {repo}`station3/GATES.md` |
| 4 | Do the muscles pull with the same static force? | {repo}`station4/GATES.md` |
| 5 | Along a real walking trial, do lengths, moment arms and the joint moments needed to walk match? | {repo}`station5/GATES.md` |
| 6 | Does it get past MyoConverter's documented limitations, on every model OpenSim distributes? | {repo}`station6/GATES.md` |

## Models

- **gait10dof18musc**: 10 degrees of freedom, 18 muscles (MyoConverter's own test model).
- **gait2354**: 23 degrees of freedom, 54 muscles.
- **Rajagopal 2016**: a full-body model with 80 leg muscles and 40 wrapping cylinders.
- **RajagopalLaiUhlrich2023**: Rajagopal's successor, with polynomial knees (Station 2 only).
- Station 6: all 56 model files in [opensim-models](https://github.com/opensim-org/opensim-models) (@ `d9b05d4`).

## Results

Final converter, against MyoConverter on the same model. Lower is better.

### Bones (Station 2): worst body position error over 1400–2200 poses

| Model | osim2mujoco | MyoConverter |
| --- | --- | --- |
| gait10dof18musc | **0.22 mm** | 0.88 mm |
| gait2354 | **0.22 mm** | 0.89 mm |
| Rajagopal 2016 | **0.15 mm, 0.04°** | 15.6 mm, 0.89° |
| RajagopalLaiUhlrich2023 | **0.42 mm, 0.07°** | couldn't be converted |

Gate: within 1 mm and 0.5° in every pose. All pass. Details: {repo}`station2/FINDINGS.md`.

### Muscle paths (Station 3): mean moment-arm error

| Model | osim2mujoco | MyoConverter |
| --- | --- | --- |
| gait10dof18musc | **0.86 mm** | 7.58 mm |
| gait2354 | **0.36 mm** | 1.54 mm |
| Rajagopal 2016 | **0.46 mm** | 1.64 mm |

Gate: no muscle–joint pair worse than MyoConverter's beyond a tolerance. All pass. Details:
{repo}`station3/FINDINGS.md`.

### Muscle forces (Station 4): mean static-force error, % of max force

| Model | osim2mujoco, full / passive | MyoConverter, full / passive |
| --- | --- | --- |
| gait10dof18musc | **3.1% / 1.3%** | 10.1% / 13.2% |
| gait2354 | **3.6% / 2.9%** | 16.1% / 15.7% |
| Rajagopal 2016 | **2.5% / 1.1%** | 6.9% / 6.9% |

Gate: no muscle worse than MyoConverter's beyond a tolerance, or listed with a test-backed reason. All pass, with four
listed muscles: two hip flexors (only in hip extension beyond −90°, where OpenSim's own path jumps) and two Rajagopal
muscles (only against MyoConverter's luckier copy of a muscle whose mirror image it got far worse). Details:
{repo}`station4/FINDINGS.md`.

### Walking (Station 5): along OpenSim's own walking trials

| Model | Muscle length | Moment arm | Joint moments needed to walk |
| --- | --- | --- | --- |
| gait10dof18musc | **0.13 mm** (MyoConverter 5.27) | **0.51 mm** (7.61) | **0.0047 N·m/kg** (0.0075) |
| gait2354 | **0.05 mm** (1.64) | **0.11 mm** (1.20) | **0.0027 N·m/kg** (0.0039) |
| Rajagopal 2016 | **0.37 mm** (14.48) | **0.22 mm** (1.22) | **0.0011 N·m/kg** (0.0112) |

Gate: every muscle, muscle–joint pair and joint no worse than MyoConverter's beyond a tolerance, and joint moments
within 0.02 N·m/kg. All pass. Open-loop replay of recorded muscle excitations (reported only) parts from OpenSim within milliseconds in
every case, MyoConverter's included. Details: {repo}`station5/FINDINGS.md`.

### Every OpenSim model (Station 6)

| Gate | What | Result |
| --- | --- | --- |
| L0 | The support report predicts every file's outcome | **pass**: 56/56, for both converters |
| L1 | Every file converts, except those with an out-of-scope element | **pass**: 54/56 (MyoConverter 40/56) |
| L2 | Bones, on the newly converted files | **pass** |
| L3 | Mass properties exact (1e-9) | **pass**: 54/54 (MyoConverter fails 32 of its 40) |
| L4 | Limit forces within 1% of peak | **pass**: 24 forces in 9 files, worst 0.22% |
| L5 | Path actuators and springs | **pass**: lengths within 0.19 mm, forces within 1.6% |
| L6 | Contact geometry within 1e-6 m | **pass**: 66 pairs, worst 3e-14 m |
| L7 | Contact dynamics on drops the calibration never saw | **fail**: see {doc}`limitations` |

Details: {repo}`station6/FINDINGS.md`.

## Rerunning the checks

Each station has a script that reruns it from scratch; see {doc}`developers`. GitHub Actions reruns Stations 2–6's
quick checks on every push.
