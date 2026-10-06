# lowlimb-converter

OpenSim → MuJoCo conversion for lower-limb gait models, with a validation report for every
converted model. It's a specialised fork of [MyoConverter](https://github.com/MyoHub/myoconverter).

> **Status: Station 1b done (port to a modern stack + element-support report).** MyoConverter's pipeline now runs on
> Python 3.12, OpenSim 4.6, MuJoCo 3.15 and NumPy 2 with no Docker. Verified on Linux; the lockfile includes the Apple
> Silicon wheels, but the native macOS run is not verified yet. For both gait models it reproduces the Station 0
> baseline to floating-point precision ([`station1/FINDINGS.md`](station1/FINDINGS.md)). A support report now says,
> before converting, what will be converted, approximated or dropped, and whether conversion will fail.
> No accuracy improvements yet; those start in Station 2.
> Plan: [project pipeline doc](https://claude.ai/code/artifact/41e46ee4-a7c3-42bc-82d8-dd94af3ef801) ·
> log: [`EXPERIMENTS.md`](EXPERIMENTS.md) · gates: [`baseline/GATES.md`](baseline/GATES.md), [`station1/GATES.md`](station1/GATES.md)

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
- MyoConverter's moment-arm optimisation (Step 2) did not run for any muscle group of either gait model.
  Rectus femoris has the largest moment-arm error in both.
- MuJoCo muscles have rigid tendons, and native elastic tendons are not available
  ([`station0/elastic_tendon/FINDINGS.md`](station0/elastic_tendon/FINDINGS.md)).
- Porting to MuJoCo 3 / NumPy 2 took four small fixes (one silent-risk API change: `actuator_moment` became sparse).
  Compiled models match the baseline except for documented, physics-neutral differences, plus a 0.015–0.020%
  total-mass difference on massless virtual bodies, which comes from MuJoCo 2.3.7's XML writer dropping `boundmass`.
- Gravity is not converted: OpenSim models use 9.80665 m/s², converted models get MuJoCo's default 9.81.
- Rajagopal 2016's torque actuators (lumbar, arms) convert 10× too weak: the converter drops `optimal_force`.
- RajagopalLaiUhlrich2023 can't be converted yet: its knees use a `PolynomialFunction`, which the converter rejects.

## Running the ported pipeline (Station 1a)

Python 3.12; on macOS, version 15 or newer (OpenSim's wheel requires it). No Docker needed.

```bash
git clone https://github.com/MyoHub/myoconverter.git upstream      # supplies the OpenSim models
git -C upstream checkout cadf38059367a51239e6dc28c9fbe8b8fbd5149f
python3.12 -m venv .venv && .venv/bin/pip install --require-hashes -r env/requirements.lock
bash station1/run_station1.sh gait10dof18musc upstream station1/gait10dof18musc .venv/bin/python
```

The script converts the model, then checks gates P1–P5 and exits non-zero if any fails.

## Checking a model before converting it (Station 1b)

```bash
.venv/bin/python -m lowlimb_converter.support path/to/model.osim --geometry path/to/Geometry \
    --md report.md --json report.json
```

Every element gets one status: converted, approximated, skipped, ignored or unsupported, with the reason and the
converter source it comes from. Exit code 0 means the XML step is predicted to convert, 1 that it will fail
(the report names the first element it fails on), 2 an error reading the file. Without `--geometry`, mesh files
aren't checked for existence. Reports for four models are in [`station1/support/`](station1/support/).

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
| `lowlimb_converter/` | Our own code: `support.py`, the element-support report (Station 1b) |
| `env/` | Modern-stack requirements and the universal hashed lockfile |
| `station1/` | Station 1a/1b: gates, run script, results per model, tests, findings |
| `station1/support/` | Support reports for four models, S3 results, model provenance |
| `baseline/` | Station 0: run scripts, gates, extracted metrics, outputs for each model |
| `baseline/upstream_shipped/` | Metrics extracted from the outputs MyoConverter ships, for the reproducibility check |
| `station0/elastic_tendon/` | Whether MuJoCo supports elastic tendons: test and findings |
| `docker/` | Baseline image definitions (Station 0) |
| `.github/workflows/` | Clean-machine checks: Station 0 (Docker, lockfile) and Station 1a (Linux on push; macOS on manual run) |
| `EXPERIMENTS.md` | Every run: intended vs actual vs deviation |

## Credit and license

Apache 2.0 (see `LICENSE` and `NOTICE`). Builds on MyoConverter and O2MConverter; please cite their
papers too (see `CITATION.cff`).
