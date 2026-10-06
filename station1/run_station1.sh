#!/bin/bash
# Station 1a: convert one model with the ported pipeline on the modern stack, then check gates P1-P5
# (station1/GATES.md). Exit code 0 only if every gate passes.
# Usage: station1/run_station1.sh <gait10dof18musc|gait2354> <upstream_clone_dir> <output_dir> [python]
#   upstream_clone_dir: MyoConverter clone at cadf380 (source of the OpenSim models)
#   python: interpreter with env/requirements.lock installed (default: python)
set -uo pipefail
MODEL=$1; UP=$(cd "$2" && pwd); OUT=$3; PY=${4:-python}
REPO=$(cd "$(dirname "$0")/.." && pwd)
case $MODEL in
  gait10dof18musc) OSIM=models/osim/Gait10dof18musc/gait10dof18musc.osim ;;
  gait2354)        OSIM=models/osim/Gait2354Simbody/gait2354.osim ;;
  *) echo "unknown model $MODEL"; exit 2 ;;
esac
mkdir -p "$OUT"; OUT=$(cd "$OUT" && pwd)
$PY -c "import opensim, mujoco, numpy; print('opensim', opensim.GetVersionAndDate()); print('mujoco', mujoco.__version__); print('numpy', numpy.__version__)" | tee "$OUT/versions.txt" || exit 1
fail=0
echo "== P1 RUNS: converting $MODEL ..."
start=$(date +%s)
$PY "$REPO/station1/run_port.py" "$MODEL" "$UP" "$OUT" > "$OUT/run.log" 2>&1; rc=$?
echo "exit_code=$rc wall_seconds=$(( $(date +%s) - start ))" | tee "$OUT/timing.txt"
caught=$(grep -c "An error has been caught" "$OUT/run.log")
steps=$(grep -cE "Finished step [123] (conversion|validation)" "$OUT/run.log")
if [ $rc -eq 0 ] && [ "$caught" -eq 0 ] && [ "$steps" -eq 6 ]; then echo "P1 RUNS: PASS (6/6 steps, 0 caught errors)"
else echo "P1 RUNS: FAIL (exit $rc, steps $steps/6, caught errors $caught)"; tail -30 "$OUT/run.log"; exit 1; fi
leaks=$(cat "$OUT"/*_cvt*.xml | grep -c "np\.")
[ "$leaks" -eq 0 ] && echo "XML numpy-repr leak check: PASS" || { echo "XML numpy-repr leak check: FAIL ($leaks lines)"; fail=1; }
echo "== P2 STRUCTURAL (OpenSim + current MuJoCo):"
(cd "$UP" && $PY "$REPO/baseline/check_counts.py" "$OSIM" "$OUT") > "$OUT/gates.txt" 2>&1
grep -E "GATE" "$OUT/gates.txt"
[ "$(grep -c 'GATE: *PASS' "$OUT/gates.txt")" -eq 3 ] || fail=1
$PY "$REPO/baseline/extract_metrics.py" "$OUT" > /dev/null || { echo "metric extraction failed"; exit 1; }
echo "== P3-P5 vs Station 0 baseline:"
$PY "$REPO/station1/compare_to_baseline.py" "$OUT" "$REPO/baseline/$MODEL" > "$OUT/vs_baseline.txt"; rc=$?
cut -c1-160 "$OUT/vs_baseline.txt"; [ $rc -eq 0 ] || fail=1
[ $fail -eq 0 ] && echo "STATION 1a: ALL GATES PASS for $MODEL" || echo "STATION 1a: GATE FAILURE for $MODEL"
exit $fail
