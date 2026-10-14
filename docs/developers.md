# For developers

## Repository layout

| Path | What |
| --- | --- |
| `osim2mujoco/` | The project's own code: the command (`cli.py`), conversion wrapper (`convert.py`), validation report (`validate.py`), support report (`support.py`) and the checks (`checks/`) |
| `src/myoconverter/` | MyoConverter @ `cadf380`, vendored, with every changed file marked ({repo}`VENDORED.md`) |
| `stationN/` | One folder per station: `GATES.md`, run script, tests, `FINDINGS.md`, results per run |
| `tests/` | Tests of the installable tool |
| `env/` | Requirements and the hashed lockfile |
| `docs/` | This documentation |
| `EXPERIMENTS.md` | Every run, newest first |
| `LIMITATIONS.md` | MyoConverter's documented limitations, and the evidence for where this project stands on each |

## Running the tests

Each test is a plain script that prints `ok` / `FAIL` per check and exits non-zero if any check failed. Most take the
converter's source folder and a scratch folder as arguments; the docstring at the top of each file has its usage.

```bash
python tests/test_tool.py upstream ../scratch/tool                     # the command, end to end
python station2/tests/test_chained_coupling.py src ../opensim-models/Models/Rajagopal/Rajagopal2016.osim GEOMETRY ../scratch/s2
python station6/tests/test_more_files.py src OLD_SRC ../opensim-models ../scratch/s6
```

Tests of a converter change take the converter before it too (`OLD_SRC`, e.g. `git archive 49140ff src`) and check
that it fails there, so each change is shown to be needed.

## Rerunning a station

Each station reruns from scratch into a new, labelled set of results:

```bash
bash station2/run_station2.sh myrun upstream ../opensim-models ../station2_work .venv/bin/python
bash station3/run_station3.sh myrun upstream ../station3_work .venv/bin/python
bash station4/run_station4.sh myrun upstream ../station4_work .venv/bin/python
bash station5/run_station5.sh myrun upstream ../opensim-models ../station5_work .venv/bin/python
bash station6/run_station6.sh myrun upstream ../opensim-models ../station6_work .venv/bin/python
```

`upstream` is a clone of MyoConverter at `cadf380` (it supplies the gait models), and `../opensim-models` a clone of
opensim-models at `d9b05d4`. Work folders must be outside the repository: some converted models aren't ours to
redistribute. Stations 3–5 take 15–35 minutes on two cores; Station 6 takes several hours. The README has each
station's setup.

## Continuous integration

GitHub Actions runs on every push to `main`:

| Workflow | What |
| --- | --- |
| `tool` | Installs the package from the lockfile and converts and validates gait10dof18musc (Linux, full; macOS, quick) |
| `station2-kinematics` | Station 2 and the structural gates |
| `station3-paths`, `station4-forces`, `station5-walking` | Stations 3–5 on the gait models |
| `station6-limitations` | The tests of every Station 6 change, and the support report on all 56 files (the full gated run is manual) |
| `docs` | Builds this documentation with warnings as errors |

Stations 0 and 1a run manually (Station 0 needs Docker).

## Changing the converter

The project's rules ({repo}`CONTRIBUTING.md`):

1. **Gates before results.** A station's pass/fail rules are written before any of its results are seen. A rule that
   turns out to be wrong gets a dated amendment, never a silent edit.
2. **One change at a time,** each its own commit, with the affected stations rerun under a new label.
3. **Log everything** in `EXPERIMENTS.md`, failed runs included. Rows are never deleted; a wrong result is corrected
   by a new row.
4. **Mark vendored changes.** Every modified file in `src/myoconverter/` starts with a note saying what changed and
   why (Apache 2.0).
5. **Show the check can fail.** A new check comes with a test that breaks something on purpose and shows the check
   catches it.

## Building these docs

```bash
pip install -r docs/requirements.txt
sphinx-build -W -b html docs docs/_build/html
```

The build doesn't need OpenSim or MuJoCo: the API reference mocks them. Read the Docs builds from
`.readthedocs.yaml` on every push.
