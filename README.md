# lowlimb-converter

OpenSim → MuJoCo conversion for lower-limb gait models, with a validation report for every
converted model. It's a specialised fork of [MyoConverter](https://github.com/MyoHub/myoconverter).

> **Status: Station 4 (muscle forces) done for both gait models; Rajagopal 2016 is held at Station 2.** The pipeline
> runs on Python 3.12, OpenSim 4.6, MuJoCo 3.15 and NumPy 2 with no Docker (verified on Linux; the lockfile includes
> Apple Silicon wheels, but the native macOS run is not verified yet). For both gait models, compared with
> MyoConverter's conversions of the same models:
> - every body is within 0.22 mm of OpenSim in every tested pose (MyoConverter: 0.89 mm);
> - muscle moment arms are 5–8× closer to OpenSim's on average (worst muscle 3.4–4.7 mm RMS, MyoConverter 70–78 mm);
> - static muscle forces are 3–5× closer at full activation and 5–11× closer when passive (mean 3.0–3.5% and
>   1.2–2.9% of max isometric force; MyoConverter 10–16%).
>
> Details: [`station2/FINDINGS.md`](station2/FINDINGS.md), [`station3/FINDINGS.md`](station3/FINDINGS.md),
> [`station4/FINDINGS.md`](station4/FINDINGS.md). Rajagopal 2016's patella is still 0.64° off, over the 0.5° Station 2
> gate. Walking-level validation is next (Station 5).
> Plan: [project pipeline doc](https://claude.ai/code/artifact/41e46ee4-a7c3-42bc-82d8-dd94af3ef801) ·
> log: [`EXPERIMENTS.md`](EXPERIMENTS.md) · gates: [`baseline/GATES.md`](baseline/GATES.md), [`station1/GATES.md`](station1/GATES.md),
> [`station2/GATES.md`](station2/GATES.md), [`station3/GATES.md`](station3/GATES.md), [`station4/GATES.md`](station4/GATES.md)

## Quick start

Python 3.11–3.13 (tested on 3.12, which the lockfile is for). OpenSim publishes wheels for Linux x86-64, macOS 15 or
newer and Windows; only Linux has been tested end to end so far (CI now also runs the quick test on macOS).

```bash
git clone https://github.com/wafisharif/lowlimb-converter.git && cd lowlimb-converter
python3.12 -m venv .venv && source .venv/bin/activate
pip install --require-hashes -r env/requirements.lock   # the exact, tested versions
pip install --no-deps .                                   # adds the lowlimb-convert command
```

Then, for an OpenSim model with its `Geometry` folder next to it:

```bash
lowlimb-convert check    model.osim                         # what converts, and how (seconds)
lowlimb-convert convert  model.osim out/ --validate         # convert, then compare with OpenSim (minutes)
lowlimb-convert validate model.osim out/model_cvt3.xml report/   # the comparison alone, for any converted model
```

`convert` refuses a model it can't handle (and says why) before doing anything, and refuses an output folder that
isn't empty: MyoConverter's pipeline would otherwise silently reuse old results found there. The converted model is
`out/<model>_cvt3.xml`. `--steps 1` stops after the XML conversion (seconds; muscles not yet fitted).

The validation report (`report.md`, with `report.json` and detailed results) compares the MuJoCo model with OpenSim
in about 1000–2000 poses: every body's position and orientation, every muscle's length and moment arms, and every
muscle's static force at four activation levels. These are the same checks used to develop the converter
(Stations 2–4 below). It does not yet cover walking (replayed gait, joint moments), or how muscle force changes with
speed.

The gait models in the examples come with MyoConverter (`models/osim/` in its repo). Python API:
`lowlimb_converter.convert.convert(...)` and `lowlimb_converter.validate.validate(...)`.

## What this will be

- Supports a short list of standard gait models (gait10dof18musc as a test bed, gait2354, Rajagopal 2016),
  not every OpenSim model. Unsupported models are rejected with a clear error.
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
  parameter (e.g. optimal length, which also scales force–velocity), which Station 5 must check.
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
  joints with quartics only.
- MyoConverter doesn't convert gravity (OpenSim's 9.80665 m/s² became MuJoCo's default 9.81) and drops torque
  actuators' `optimal_force` (Rajagopal 2016's lumbar and arm actuators came out 10× too weak). Both fixed.
- RajagopalLaiUhlrich2023 can't be converted yet: its knees use a `PolynomialFunction`, which the converter rejects.

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
.venv/bin/python -m lowlimb_converter.support path/to/model.osim --geometry path/to/Geometry \
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
| `lowlimb_converter/` | Our own code: the `lowlimb-convert` command (`cli.py`), conversion wrapper (`convert.py`), validation report (`validate.py`), element-support report (`support.py`, Station 1b) and the checks behind Stations 2–4 (`checks/`) |
| `tests/` | Tests of the installable tool |
| `env/` | Modern-stack requirements and the universal hashed lockfile |
| `station1/` | Station 1a/1b: gates, run script, results per model, tests, findings |
| `station1/support/` | Support reports for four models, S3 results, model provenance |
| `station2/` | Station 2: gates, OpenSim and MuJoCo sides of the kinematic check, run script, tests, findings, results per model and run |
| `station3/` | Station 3: gates, OpenSim and MuJoCo sides of the muscle-path check, M1 comparison, run script, tests, findings, results |
| `station4/` | Station 4: gates, OpenSim and MuJoCo sides of the muscle-force check, F1–F2 comparison, run script, tests, findings, results |
| `baseline/` | Station 0: run scripts, gates, extracted metrics, outputs for each model |
| `baseline/upstream_shipped/` | Metrics extracted from the outputs MyoConverter ships, for the reproducibility check |
| `station0/elastic_tendon/` | Whether MuJoCo supports elastic tendons: test and findings |
| `docker/` | Baseline image definitions (Station 0) |
| `.github/workflows/` | Clean-machine checks: Station 0 (Docker, lockfile), Station 2 + P1–P2, Station 3 and Station 4 (Linux, on push), the tool (Linux and macOS, on push), Station 1a (manual) |
| `EXPERIMENTS.md` | Every run: intended vs actual vs deviation |

## Credit and license

Apache 2.0 (see `LICENSE` and `NOTICE`). Builds on MyoConverter and O2MConverter; please cite their
papers too (see `CITATION.cff`).
