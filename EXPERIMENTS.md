# Experiment log

Newest first. Never delete rows; correct a wrong result with a new row.
Plan doc: https://claude.ai/code/artifact/41e46ee4-a7c3-42bc-82d8-dd94af3ef801

| Date | Model | Station | Intended | Actual | Deviation and why | Commit |
| --- | --- | --- | --- | --- | --- | --- |
| 2026-10-05 | gait10dof18musc | 1 (gate check) | Joint, muscle, keyframe gates per `baseline/check_counts.py` | PASS all three: 10/10 independent joints match OpenSim coordinate names; 18/18 muscle names match; keyframe eq. violation 0.0. MuJoCo has 38 joints = 10 free + 28 followers (4 knee translations, 24 moving-path-point joints) | Gate checked independently with mujoco 2.3.7 (pip, aarch64) + .osim XML parse; containerised re-check runs with gait2354 | (this commit) |
| 2026-10-05 | gait10dof18musc | 0 | Baseline run, upstream example settings | Completed. cvt1/2/3 + PDF. Wall ≈112 s incl. PDF (20:54:42–20:56:34 UTC, amd64 emulated on arm64 Mac); cvt3 at +57 s. 2 warnings: ConditionalPathPoint rect_fem_r/l-P2 had no anchor, treated as normal path point. Step 2 skipped MA optimisation for 6 muscles (errors below threshold) | Model notes say "not intended to be used in research": test bed / CI only. `time` output not captured (went to terminal, not run.log); wall time taken from log + file timestamps | (this commit) |
| 2026-10-05 | gait2354 | 0 | Baseline run, upstream settings | Not run in first session (command skipped); rerun via `baseline/run_station0.sh` | — | — |
| 2026-10-05 | Rajagopal | 0 | Baseline run | Not run | Not bundled with upstream; baseline deferred to model 3 | — |
| 2026-10-05 | all | 0 | Build upstream docker/Dockerfile | Failed: Debian 11 security repo 404s | Debian 11 EOL Aug 31 2026. Built own `docker/Dockerfile.baseline` on condaforge/miniforge3 (digest in `baseline/env/base_image_digest.txt`); upstream `conda_env.yml` unchanged. Container confirms mujoco 2.3.7, opensim 4.4.1 (`baseline/env/versions.txt`) | upstream cadf380 |
