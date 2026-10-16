# osim2mujoco

OpenSim → MuJoCo conversion for lower-limb gait models, with a validation report for every
converted model. It's a specialised fork of [MyoConverter](https://github.com/MyoHub/myoconverter), and an independent
project (not affiliated with the OpenSim or MuJoCo teams). Documentation: <https://osim2mujoco.readthedocs.io>.
The command is `o2m` (`osim2mujoco` also works). Until 2026-10-14 the project was called lowlimb-converter (its command
`lowlimb-convert` still works); results and logs from before then use that name.

> **Status: Stations 2–5 pass for both gait models and Rajagopal 2016 (RajagopalLaiUhlrich2023: Station 2), and Station
> 6 takes the converter past MyoConverter's documented limitations on all 56 model files OpenSim distributes (gates
> L0–L6 pass; L7, contact dynamics on unseen drops, fails and is an open item).** The pipeline runs on Python 3.12,
> OpenSim 4.6, MuJoCo 3.15 and NumPy 2 with no Docker (verified on Linux; the lockfile includes Apple Silicon wheels,
> but the native macOS run is not verified yet). Compared with MyoConverter's conversions of the same models:
> - every body is within 0.22 mm of OpenSim in every tested pose for the gait models (MyoConverter: 0.89 mm) and
>   0.15 mm for Rajagopal 2016 (MyoConverter: 15.6 mm at the knee);
> - muscle moment arms are 4–9× closer to OpenSim's on average over random poses for the gait models and 3.6× for
>   Rajagopal 2016 (0.46 vs 1.64 mm), whose wrapping paths now take the same side of each wrap object as OpenSim's;
> - static muscle forces are 3–5× closer at full activation and 5–10× closer when passive for the gait models
>   (Rajagopal 2016: 2.8× and 6.5×);
> - along OpenSim's own walking data, muscle lengths are 33–42× closer (Rajagopal 2016: 39×), moment arms 11–15×
>   closer (5.6×), and the joint moments needed to walk 1.4–1.6× closer (10×);
> - 54 of OpenSim's 56 model files convert (MyoConverter: 40), with contact geometry, limit forces, path actuators
>   and springs, four more joint types and exact mass properties, each checked against OpenSim (contact dynamics are
>   calibrated but don't yet follow OpenSim on tilted or sliding drops).
>
> Details: [`station2/FINDINGS.md`](station2/FINDINGS.md), [`station3/FINDINGS.md`](station3/FINDINGS.md),
> [`station4/FINDINGS.md`](station4/FINDINGS.md), [`station5/FINDINGS.md`](station5/FINDINGS.md),
> [`station6/FINDINGS.md`](station6/FINDINGS.md), and where this fork stands on each of MyoConverter's documented
> limitations: [`LIMITATIONS.md`](LIMITATIONS.md). Four muscle-force results miss the Station 4 tolerance against
> MyoConverter; each is listed with a test-backed reason (two hip flexors only in hip extension beyond −90°, where
> OpenSim's own path jumps; two Rajagopal muscles only against MyoConverter's luckier copy of a muscle whose mirror
> image it got far worse). Open-loop replay of muscle excitations (Station 5 level 3, reported only) parts from OpenSim
> within milliseconds in every column, MyoConverter's included.
> Plan: [project pipeline doc](https://claude.ai/code/artifact/41e46ee4-a7c3-42bc-82d8-dd94af3ef801) ·
> log: [`EXPERIMENTS.md`](EXPERIMENTS.md) · gates: [`baseline/GATES.md`](baseline/GATES.md), [`station1/GATES.md`](station1/GATES.md),
> [`station2/GATES.md`](station2/GATES.md), [`station3/GATES.md`](station3/GATES.md), [`station4/GATES.md`](station4/GATES.md),
> [`station5/GATES.md`](station5/GATES.md), [`station6/GATES.md`](station6/GATES.md)

## Quick start

Python 3.11–3.13 (tested on 3.12, which the lockfile is for). OpenSim publishes wheels for Linux x86-64, macOS 15 or
newer and Windows; only Linux has been tested end to end so far (CI now also runs the quick test on macOS).

```bash
git clone https://github.com/wafisharif/osim2mujoco.git && cd osim2mujoco
python3.12 -m venv .venv && source .venv/bin/activate
pip install --require-hashes -r env/requirements.lock   # the exact, tested versions
pip install --no-deps .                                   # adds the o2m command
```

Then, for an OpenSim model with its `Geometry` folder next to it:

```bash
o2m check    model.osim                         # what converts, and how (seconds)
o2m convert  model.osim out/ --validate         # convert, then compare with OpenSim (minutes)
o2m validate model.osim out/model_cvt3.xml report/   # the comparison alone, for any converted model
o2m validate model.osim out/model_cvt3.xml report/ --ik walk_ik.mot --grf walk_grf.xml
                                                            # ... plus a walking trial of your own (OpenSim IK + GRF)
```

`convert` refuses a model it can't handle (and says why) before doing anything, and refuses an output folder that
isn't empty: MyoConverter's pipeline would otherwise silently reuse old results found there. The converted model is
`out/<model>_cvt3.xml`. `--steps 1` stops after the XML conversion (seconds; muscles not yet fitted).

The validation report (`report.md`, with `report.json` and detailed results) compares the MuJoCo model with OpenSim in
about 1000–2000 poses: every body's position and orientation, every muscle's length and moment arms, and every muscle's
static force at four activation levels; and, where the model has them, its mass properties, limit forces, path actuators
and springs, and contact geometry and drops (level "elements"). These are the same checks used to develop the converter
(Stations 2–4 and 6 below). Give it a walking trial (`--ik` with OpenSim's inverse-kinematics result, `--grf` with the
ground reaction forces as an OpenSim ExternalLoads file; optionally `--start`/`--end`, and `--controls` with recorded
muscle excitations for an open-loop replay) and it also runs Station 5's walking check: muscle lengths and moment arms
along the trial, and the joint moments needed to walk with the measured forces. It does not check how muscle force
changes with speed.

The gait models in the examples come with MyoConverter (`models/osim/` in its repo). Python API:
`osim2mujoco.convert.convert(...)` and `osim2mujoco.validate.validate(...)`.

## What this will be

- Validated end to end on a short list of standard gait models (gait10dof18musc as a test bed, gait2354, Rajagopal
  2016, RajagopalLaiUhlrich2023). Since Station 6 it also converts, and checks element by element, almost every model
  OpenSim distributes; anything it can't handle is rejected with a clear error before converting.
- Where it stands on each of MyoConverter's own documented limitations: [`LIMITATIONS.md`](LIMITATIONS.md).
- Every converted model ships with a report comparing it to OpenSim at three levels: body kinematics,
  muscle moment arms and forces, and replayed walking (muscle lengths, joint moments).
- Every number is reported next to MyoConverter's number for the same model.

## Findings so far

- Converted models from MyoConverter (pinned to MuJoCo 2.3.7) do not load in current MuJoCo (3.x):
  `option/@collision` was removed.
- Neither gait model contains markers, so MyoConverter's Step 1 kinematic check compares zero points
  and reports NaN. Body kinematics were never actually validated for these models.
- MyoConverter's moment-arm optimisation (Step 2) did not run for any muscle group of either gait model: it only moves
  wrapping-object sites, and these models have none. Rectus femoris's knee moment arm had the wrong sign at full
  extension (69–78 mm RMS error), because its conditional path point was kept always active. Fixed by fitting
  conditional and moving path points to OpenSim's path (Station 3).
- Rerunning MyoConverter's pipeline into a folder that already holds results silently resumes from the old cvt2/cvt3.
- MyoConverter's muscle-force step (a particle swarm at full activation only) left mean errors of 10–16% of max
  isometric force, and 59–121% for a few short-fibered muscles. A deterministic refit against OpenSim's force at four
  activation levels brings the means to 1.2–3.5% (Station 4). Static force doesn't determine every MuJoCo muscle
  parameter (e.g. optimal length, which also scales force–velocity); that is still not validated.
- Along a real walking trial, the converted hip flexors were 1.0–1.2 mm off in length where MyoConverter was 0.2 mm
  off: their conditional path points were fitted so that OpenSim's on/off jumps pulled the fit away from OpenSim's
  path everywhere. Fixed with a robust fit, and a point that switches toward its neighbour through a helper joint
  (Station 5). MyoConverter also saved its models with the time step its analysis last set (5 ms) instead of 2 ms,
  which made forward simulation of the gait models inaccurate; fixed.
- MuJoCo muscles have rigid tendons, and native elastic tendons are not available
  ([`station0/elastic_tendon/FINDINGS.md`](station0/elastic_tendon/FINDINGS.md)).
- Porting to MuJoCo 3 / NumPy 2 took four small fixes (one silent-risk API change: `actuator_moment` became sparse).
  Compiled models match the baseline except for documented, physics-neutral differences, plus a 0.015–0.020%
  total-mass difference on massless virtual bodies, which comes from MuJoCo 2.3.7's XML writer dropping `boundmass`.
- MyoConverter never checked body kinematics for these models. Checked now: its gait conversions are within
  0.89 mm and 0.01°, limited by a quartic fit placed over the wrong knee range and by numbers written with
  4 significant figures. Both fixed (0.22 mm). Rajagopal 2016's knee was 15.6 mm off, because joints with an
  offset child frame rotated about the wrong point. Fixed too.
- No quartic can follow Rajagopal 2016's patella spline within 0.5° (best possible: 0.64°), and MuJoCo couples
  joints with quartics only. Fixed by two quartics in sequence through a helper joint (0.036°).
- MyoConverter doesn't convert gravity (OpenSim's 9.80665 m/s² became MuJoCo's default 9.81) and drops torque
  actuators' `optimal_force` (Rajagopal 2016's lumbar and arm actuators came out 10× too weak). Both fixed.
- Rajagopal 2016's wrapping paths: MyoConverter forced every wrap onto one side of its object, so the vasti went the
  wrong way round the femoral condyles (127 mm too long in flexion) and iliacus and psoas sometimes the long way round
  the pelvic brim (up to 180 mm). Unconstrained wraps now take the shorter side and a constrained wrap keeps its side
  site only where that matches OpenSim's path better (Station 3).
- A rigid-tendon MuJoCo muscle can't follow how a long, compliant tendon shifts the force–length curve with
  activation; the force refit now guarantees both ends of the activation range (full and passive) and lets the middle
  pay (Station 4).
- MyoConverter can't read most of OpenSim's own example models (element names with `::`, ASCII STL and OBJ meshes,
  omitted coordinate properties, and more), drops contact, limit forces and path actuators, and inflated small bodies'
  inertia; all of these are converted and checked now (Station 6).
- RajagopalLaiUhlrich2023 couldn't be converted at all: its knees use a `PolynomialFunction`. They now convert
  exactly (degree ≤ 4 polynomials are MuJoCo's own coupling), and it passes Station 2.
- MyoConverter never read an OpenSim model's `<defaults>` element (its documented limitation 3), although OpenSim
  fills omitted properties from it. Now applied as OpenSim does.

## Running the ported pipeline (Station 1a)

Python 3.12; on macOS, version 15 or newer (OpenSim's wheel requires it). No Docker needed.

```bash
git clone https://github.com/MyoHub/myoconverter.git upstream      # supplies the OpenSim models
git -C upstream checkout cadf38059367a51239e6dc28c9fbe8b8fbd5149f
python3.12 -m venv .venv && .venv/bin/pip install --require-hashes -r env/requirements.lock
bash station1/run_station1.sh gait10dof18musc upstream ../station1_run/gait10dof18musc .venv/bin/python
```

The output folder must be new or empty (the pipeline would otherwise resume from old results). The script converts the model, then checks gates P1–P5 and exits non-zero if any fails. P3–P5 check that the results
match the Station 0 baseline, which only holds for the port itself (tag `station1a`): from Station 2 on, the converter
is changed on purpose. On later commits, run it with `STRUCTURAL_ONLY=1` to check P1–P2.

## Kinematic check (Station 2)

```bash
git clone --filter=blob:none https://github.com/opensim-org/opensim-models.git ../opensim-models   # Rajagopal 2016
git -C ../opensim-models checkout d9b05d470b1a481c222372c85b75772faf8f7792
bash station2/run_station2.sh myrun upstream ../opensim-models ../station2_work .venv/bin/python
```

It reruns the XML step for each model, poses OpenSim and MuJoCo identically (1000 random poses plus a sweep of every
coordinate) and compares every body. Results go to `station2/<model>/myrun.json`. The work folder must be outside the
repo, because the converted Rajagopal model isn't ours to redistribute. `MODELS="gait10dof18musc"` runs one model.

## Checking a model before converting it (Station 1b)

```bash
.venv/bin/python -m osim2mujoco.support path/to/model.osim --geometry path/to/Geometry \
    --md report.md --json report.json
```

Every element gets one status: converted, approximated, skipped, ignored or unsupported, with the reason and the
converter source it comes from. Exit code 0 means the XML step is predicted to convert, 1 that it will fail
(the report names the first element it fails on), 2 an error reading the file. Without `--geometry`, mesh files
aren't checked for existence. Reports for four models are in [`station1/support/`](station1/support/).

## Muscle-path check (Station 3)

```bash
bash station3/run_station3.sh myrun upstream ../station3_work .venv/bin/python
```

Runs the full pipeline on both gait models into a fresh folder, then compares every muscle's moment arm about every
joint it crosses, and its length, with OpenSim in the Station 2 poses. Results go to `station3/<model>/myrun.json`
and the comparison with MyoConverter (from `station3/<model>/baseline_myoconverter.json`) to `myrun_m1.json`.
About 15 minutes on two cores.

## Muscle-force check (Station 4)

```bash
bash station4/run_station4.sh myrun upstream ../station4_work .venv/bin/python
```

Runs the full pipeline on both gait models into a fresh folder, holds each muscle still in the Station 2 poses at
activation 0, 0.25, 0.5 and 1, and compares its force with OpenSim's equilibrium force. Results go to
`station4/<model>/myrun.json` and the comparison with MyoConverter to `myrun_f.json`. About 20 minutes on two cores.

## Walking check (Station 5)

```bash
git clone --filter=blob:none --no-checkout https://github.com/opensim-org/opensim-models.git ../opensim-models
git -C ../opensim-models sparse-checkout set Pipelines/Gait10dof18musc Pipelines/Gait2354_Simbody
git -C ../opensim-models checkout d9b05d470b1a481c222372c85b75772faf8f7792
bash station5/run_station5.sh myrun upstream ../opensim-models ../station5_work .venv/bin/python
```

Runs the full pipeline on both gait models into a fresh folder, then replays OpenSim's own walking data in both
simulators: muscle lengths and moment arms over the gait cycle, and the joint moments needed to walk with the
measured ground forces (gates W0–W3), plus an open-loop replay of the recorded muscle excitations (level 3, reported
only). Results go to `station5/<model>/myrun.json`, `myrun_w.json` and `myrun_replay*.json`. About 35 minutes on two
cores.

## Past MyoConverter's limitations (Station 6)

```bash
git clone --filter=blob:none https://github.com/opensim-org/opensim-models.git ../opensim-models
git -C ../opensim-models checkout d9b05d470b1a481c222372c85b75772faf8f7792
bash station6/run_station6.sh myrun upstream ../opensim-models ../station6_work .venv/bin/python
```

Runs the support report and the XML step on every one of the 56 model files in opensim-models (and the Station 1a
port, for MyoConverter's column), then gates L2–L7 on the converted files: kinematics, mass properties, limit forces,
path actuators and springs, contact geometry, and contact dynamics on WalkerModel drops. Results go to
`station6/results/myrun/`. Several hours on two cores (`SKIP_COVERAGE_MC=1` skips MyoConverter's column).
`o2m validate` runs the same element checks (level "elements") on any converted model.

## Reproducing the Station 0 baseline

Requires Docker. On Apple Silicon, the image runs under amd64 emulation.

```bash
git clone https://github.com/MyoHub/myoconverter.git upstream
git -C upstream checkout cadf38059367a51239e6dc28c9fbe8b8fbd5149f
docker build --platform linux/amd64 -f docker/Dockerfile.baseline -t myoconverter:baseline-cadf380 upstream
docker run --rm --platform linux/amd64 \
  --mount type=bind,src="$PWD/baseline",target=/app/data \
  myoconverter:baseline-cadf380 bash /app/data/run_station0.sh
```

`docker/Dockerfile.baseline` replaces upstream's Dockerfile, which no longer builds because Debian 11
reached end of life. Upstream's `conda_env.yml`, which holds the actual dependency pins, is unchanged.

## Layout

| Path | What |
| --- | --- |
| `src/myoconverter/` | Vendored MyoConverter @ cadf380 with our port fixes (each modified file says what changed; see `VENDORED.md`) |
| `osim2mujoco/` | Our own code: the `o2m` command (`cli.py`), conversion wrapper (`convert.py`), validation report (`validate.py`), element-support report (`support.py`, Station 1b) and the checks behind Stations 2–6 (`checks/`) |
| `tests/` | Tests of the installable tool |
| `docs/` | The documentation site (Sphinx; built on Read the Docs from `.readthedocs.yaml`) |
| `env/` | Modern-stack requirements and the universal hashed lockfile |
| `station1/` | Station 1a/1b: gates, run script, results per model, tests, findings |
| `station1/support/` | Support reports for four models, S3 results, model provenance |
| `station2/` | Station 2: gates, OpenSim and MuJoCo sides of the kinematic check, run script, tests, findings, results per model and run |
| `station3/` | Station 3: gates, OpenSim and MuJoCo sides of the muscle-path check, M1 comparison, run script, tests, findings, results |
| `station4/` | Station 4: gates, OpenSim and MuJoCo sides of the muscle-force check, F1–F2 comparison, run script, tests, findings, results |
| `station5/` | Station 5: gates, OpenSim and MuJoCo sides of the walking check (paths, inverse dynamics, forward replay), W1–W3 comparison, run script, tests, findings, results |
| `station6/` | Station 6: gates, coverage of OpenSim's 56 model files, gates L1–L7, run script, tests, findings, results per run |
| `LIMITATIONS.md` | MyoConverter's ten documented limitations, and where this fork stands on each |
| `baseline/` | Station 0: run scripts, gates, extracted metrics, outputs for each model |
| `baseline/upstream_shipped/` | Metrics extracted from the outputs MyoConverter ships, for the reproducibility check |
| `station0/elastic_tendon/` | Whether MuJoCo supports elastic tendons: test and findings |
| `docker/` | Baseline image definitions (Station 0) |
| `.github/workflows/` | Clean-machine checks: Station 0 (Docker, lockfile), Station 2 + P1–P2, Stations 3–6 (Linux, on push), the tool (Linux and macOS, on push), the docs (on push), Station 1a (manual) |
| `EXPERIMENTS.md` | Every run: intended vs actual vs deviation |

## Credit and license

Apache 2.0 (see `LICENSE` and `NOTICE`). Builds on MyoConverter and O2MConverter; please cite their
papers too (see `CITATION.cff`).
