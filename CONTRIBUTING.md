# Contributing

Thanks for helping. This project changes a converter whose output people use in research, so every change to it is
measured.

## Reporting a problem

Open an issue with the OpenSim model (or a link to it), the command you ran, and the output of
`o2m check model.osim`. If the problem is in a converted model, attach `report.md` from
`o2m validate`.

## Changing the converter

The project follows a gated plan ([pipeline doc](https://claude.ai/code/artifact/41e46ee4-a7c3-42bc-82d8-dd94af3ef801)):

1. **Gates before results.** Pass/fail criteria for a station are written (in `stationN/GATES.md`) before any code
   for that station runs. A gate that turns out to be wrong is amended with a dated note, never silently edited.
2. **One change at a time.** Each converter change is its own commit, with a before/after run of the affected
   station(s) on both gait models (`stationN/run_stationN.sh <label> ...`), and the results committed under that
   label.
3. **Log everything.** Every run gets a row in `EXPERIMENTS.md`: intended, actual, deviation and why. Rows are
   never deleted; a wrong result is corrected by a new row.
4. **Mark vendored changes.** Files in `src/myoconverter/` are MyoConverter's (Apache 2.0). Every modified file
   starts with a short "Modified by osim2mujoco" note saying what changed and why.
5. **Show the check can fail.** New checks come with a test that breaks the model on purpose and shows the check
   catches it (see `station*/tests/`).

## Running the checks

See the README: each station's section has the command. CI runs Stations 2–4 and the tool tests on every push to
`main`.
