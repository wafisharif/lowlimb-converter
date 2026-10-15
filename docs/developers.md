# Development

## Layout

| Path | |
| --- | --- |
| `osim2mujoco/` | The CLI, conversion wrapper, validation report, support report and checks |
| `src/myoconverter/` | MyoConverter at `cadf380` with this project's changes ({repo}`VENDORED.md`) |
| `stationN/` | Criteria, run script, tests, findings and results for each station |
| `tests/` | Tests of the installed tool |
| `env/` | Requirements and lockfile |
| `docs/` | This site |
| `EXPERIMENTS.md` | Log of every run |

## Tests

Tests are plain scripts that print `ok`/`FAIL` per check and exit non-zero on failure. Usage is in each file's
docstring.

```bash
python tests/test_tool.py upstream ../scratch/tool
python station2/tests/test_chained_coupling.py src ../opensim-models/Models/Rajagopal/Rajagopal2016.osim GEOMETRY ../scratch/s2
python station6/tests/test_more_files.py src OLD_SRC ../opensim-models ../scratch/s6
```

Tests for converter changes also take the previous converter (`OLD_SRC`, e.g. from `git archive 49140ff src`) and
check that it fails.

## Rerunning stations

```bash
bash station2/run_station2.sh myrun upstream ../opensim-models ../station2_work .venv/bin/python
bash station3/run_station3.sh myrun upstream ../station3_work .venv/bin/python
bash station4/run_station4.sh myrun upstream ../station4_work .venv/bin/python
bash station5/run_station5.sh myrun upstream ../opensim-models ../station5_work .venv/bin/python
bash station6/run_station6.sh myrun upstream ../opensim-models ../station6_work .venv/bin/python
```

`upstream` is MyoConverter at `cadf380`, and `../opensim-models` is opensim-models at `d9b05d4`. Keep work folders
outside the repository, since some converted models can't be redistributed. Stations 3–5 take 15–35 minutes on two
cores; Station 6 takes several hours.

## CI

| Workflow | Runs |
| --- | --- |
| `tool` | Install from the lockfile, convert and validate gait10dof18musc (full on Linux, quick on macOS) |
| `station2-kinematics` to `station5-walking` | Stations 2–5 on the gait models |
| `station6-limitations` | Tests for every Station 6 change, and the support report on all 56 models |
| `docs` | Builds this site, failing on warnings |

All run on pushes to `main`. Stations 0 and 1a, and the full Station 6 run, are manual.

## Changing the converter

From {repo}`CONTRIBUTING.md`:

1. Write a station's pass/fail criteria before seeing any of its results. If one turns out wrong, add a dated
   amendment rather than editing it.
2. One change per commit, with the affected stations rerun under a new label.
3. Log every run in `EXPERIMENTS.md`, failures included. Never delete a row.
4. Mark every modified file in `src/myoconverter/` with what changed and why (Apache 2.0 requires it).
5. Every new check needs a test showing it catches a deliberately broken case.

## Building the docs

```bash
pip install -r docs/requirements.txt
sphinx-build -W -b html docs docs/_build/html
```

OpenSim and MuJoCo aren't needed; the API reference mocks them.
