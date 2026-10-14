#!/bin/bash
# Station 6: past MyoConverter's limitations (station6/GATES.md). Runs gates L0-L7 and writes the results to
# station6/results/<label>/. Exit code 0 only if every gate passes.
# Usage: station6/run_station6.sh <label> <upstream_clone_dir> <opensim_models_clone_dir> <work_dir> [python]
#   upstream_clone_dir: MyoConverter clone at cadf380 (gait models, for the L2 regression and Station 1b's S2)
#   opensim_models_clone_dir: opensim-org/opensim-models clone at d9b05d4 (all 56 model files)
#   work_dir: scratch folder outside the repo (conversions of models whose licence doesn't allow committing them)
# Environment: SKIP_COVERAGE_MC=1 skips the MyoConverter column of L0-L1 (slow; results are committed).
set -uo pipefail
LABEL=$1; UP=$(cd "$2" && pwd); OSM=$(cd "$3" && pwd); WORK=$4; PY=${5:-python}
REPO=$(cd "$(dirname "$0")/.." && pwd)
mkdir -p "$WORK"; WORK=$(cd "$WORK" && pwd)
case "$WORK/" in "$REPO"/*) echo "work_dir must be outside the repo"; exit 2 ;; esac
RES="$REPO/station6/results/$LABEL"; mkdir -p "$RES"
fail=0
export PYTHONPATH="$REPO${PYTHONPATH:+:$PYTHONPATH}"

$PY -c "import opensim, mujoco, numpy; print('opensim', opensim.__version__, '| mujoco', mujoco.__version__, '| numpy', numpy.__version__)" || exit 1

echo "== L0-L1: every opensim-models file, our converter"
$PY "$REPO/station6/coverage.py" "$OSM" "$WORK/coverage" ours "$RES/coverage_ours.json" > "$RES/coverage_ours.txt" 2>&1 \
  || fail=1
tail -1 "$RES/coverage_ours.txt"
# the files this run used, with their SHA-256 (station6/GATES.md)
$PY -c "import json,sys; r=json.load(open(sys.argv[1]))['results']; print('file\tsha256'); [print(x['file']+'\t'+x['sha256']) for x in r]" \
  "$RES/coverage_ours.json" > "$REPO/station6/models.tsv" || fail=1
if [ -z "${SKIP_COVERAGE_MC:-}" ]; then
  echo "== L0-L1: every opensim-models file, MyoConverter (Station 1a port)"
  $PY "$REPO/station6/coverage.py" "$OSM" "$WORK/coverage" myoconverter "$RES/coverage_myoconverter.json" \
    > "$RES/coverage_myoconverter.txt" 2>&1 || fail=1
  tail -1 "$RES/coverage_myoconverter.txt"
fi
echo "== L0: support report S1 on every file (and S2 on the gait models)"
mapfile -t ALL < <(cd "$OSM" && find . -name "*.osim" | sort | sed "s#^\./#$OSM/#")
rm -rf "$WORK/s1"
$PY "$REPO/station1/tests/test_support_report.py" "$UP" "$WORK/s1" "${ALL[@]}" > "$RES/support_s1_s2.txt" 2>&1 || fail=1
tail -1 "$RES/support_s1_s2.txt"

echo "== L1-L7 on the converted files"
$PY "$REPO/station6/gates.py" "$OSM" "$RES/coverage_ours.json" "$RES" "$WORK/gates" ${SKIP_COVERAGE_MC:+--no-mc} \
  || fail=1

[ $fail -eq 0 ] && echo "STATION 6 ($LABEL): ALL GATES PASS" || echo "STATION 6 ($LABEL): GATE FAILURE"
exit $fail
