# lowlimb-converter

OpenSim → MuJoCo conversion for lower-limb gait models, with a validation report for every
converted model. It's a specialised fork of [MyoConverter](https://github.com/MyoHub/myoconverter).

> **Status: Station 0 (baseline).** Nothing here is ready to use yet. We are measuring how
> MyoConverter performs today, unmodified, so every later change can be compared against it.
> Plan: [project pipeline doc](https://claude.ai/code/artifact/41e46ee4-a7c3-42bc-82d8-dd94af3ef801) ·
> log: [`EXPERIMENTS.md`](EXPERIMENTS.md) · gates: [`baseline/GATES.md`](baseline/GATES.md)

## What this will be

- Supports a short list of standard gait models (gait10dof18musc as a test bed, gait2354, Rajagopal 2016),
  not every OpenSim model. Unsupported models are rejected with a clear error.
- Every converted model ships with a report comparing it to OpenSim at three levels: body kinematics,
  muscle moment arms and forces, and replayed walking (muscle lengths, joint moments).
- Every number is reported next to MyoConverter's number for the same model.

## Baseline findings so far

- Converted models from MyoConverter (pinned to MuJoCo 2.3.7) do not load in current MuJoCo (3.x):
  `option/@collision` was removed.
- Neither gait model contains markers, so MyoConverter's Step 1 kinematic check compares zero points
  and reports NaN. Body kinematics were never actually validated for these models.
- MyoConverter's moment-arm optimisation (Step 2) did not run for any muscle group of either gait model.
  Rectus femoris has the largest moment-arm error in both.
- MuJoCo muscles have rigid tendons, and native elastic tendons are not available
  ([`station0/elastic_tendon/FINDINGS.md`](station0/elastic_tendon/FINDINGS.md)).

## Reproducing the baseline

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
| `baseline/` | Station 0: run scripts, gates, extracted metrics, outputs for each model |
| `baseline/upstream_shipped/` | Metrics extracted from the outputs MyoConverter ships, for the reproducibility check |
| `station0/elastic_tendon/` | Whether MuJoCo supports elastic tendons: test and findings |
| `docker/` | Baseline image definition |
| `EXPERIMENTS.md` | Every run: intended vs actual vs deviation |

## Credit and license

Apache 2.0 (see `LICENSE` and `NOTICE`). Builds on MyoConverter and O2MConverter; please cite their
papers too (see `CITATION.cff`).
