#!/bin/bash
# Station 3: muscle paths (station3/GATES.md). Runs the full pipeline with the current converter on each gait model,
# then compares every muscle's moment arms and length against OpenSim, for our output and MyoConverter's.
# Exit code 0 only if M0 and M1 pass for every model run.
# Usage: station3/run_station3.sh <label> <upstream_clone_dir> <work_dir> [python] [python_mujoco237]
#   label:      results go to station3/<model>/<label>.json and <label>_m1.json (e.g. run1)
#   upstream_clone_dir: MyoConverter clone at cadf380 (gait models)
#   work_dir:   scratch folder outside the repo for conversions and OpenSim reference data
#   python:     interpreter with env/requirements.lock installed (default: python)
#   python_mujoco237: interpreter with mujoco==2.3.7 + numpy, for the MyoConverter column (computed once per model;
#                     without it, an existing station3/<model>/baseline_myoconverter.json is used)
# Environment: MODELS="gait10dof18musc gait2354" (default: both).
set -uo pipefail
LABEL=$1; UP=$(cd "$2" && pwd); WORK=$3; PY=${4:-python}; PY237=${5:-}
MODELS=${MODELS:-"gait10dof18musc gait2354"}
REPO=$(cd "$(dirname "$0")/.." && pwd)
mkdir -p "$WORK"; WORK=$(cd "$WORK" && pwd)
case "$WORK/" in "$REPO"/*) echo "work_dir must be outside the repo"; exit 2 ;; esac

fail=0
$PY -c "import opensim, mujoco, numpy; print('opensim', opensim.__version__, '| mujoco', mujoco.__version__, '| numpy', numpy.__version__)" || exit 1
for NAME in $MODELS; do
  case $NAME in
    gait10dof18musc) OSIM="$UP/models/osim/Gait10dof18musc/gait10dof18musc.osim" ;;
    gait2354)        OSIM="$UP/models/osim/Gait2354Simbody/gait2354.osim" ;;
    *) echo "unknown model $NAME"; exit 2 ;;
  esac
  # A fresh folder per run: the pipeline resumes Steps 2-3 from any cvt2/cvt3 already in its output folder
  # (O2MSteps.py), which would silently reuse an older model.
  OUT="$WORK/$LABEL/$NAME"; RES="$REPO/station3/$NAME"
  if [ -e "$OUT" ]; then echo "$OUT already exists; use a new label or remove it"; exit 2; fi
  mkdir -p "$OUT" "$RES"
  echo "== $NAME: full pipeline (current converter)"
  start=$(date +%s)
  $PY "$REPO/station1/run_port.py" "$NAME" "$UP" "$OUT" > "$OUT/run.log" 2>&1 \
    || { echo "pipeline failed"; tail -20 "$OUT/run.log"; fail=1; continue; }
  echo "   done in $(( $(date +%s) - start )) s"
  echo "== $NAME: OpenSim lengths and moment arms"
  $PY "$REPO/station3/osim_paths.py" "$OSIM" "$OUT/paths_reference.npz" 2>&1 | grep -v "^\[" || { fail=1; continue; }
  echo "== $NAME: ours ($LABEL)"
  $PY "$REPO/station3/mujoco_paths.py" "$OUT/${NAME}_cvt3.xml" "$OUT/paths_reference.npz" "$RES/$LABEL.json" || fail=1
  if [ ! -f "$RES/baseline_myoconverter.json" ]; then
    if [ -z "$PY237" ]; then echo "no MyoConverter column: pass python_mujoco237"; fail=1; continue; fi
    echo "== $NAME: MyoConverter's own output (MuJoCo 2.3.7)"
    $PY237 "$REPO/station3/mujoco_paths.py" "$REPO/baseline/$NAME/${NAME}_cvt3.xml" "$OUT/paths_reference.npz" \
      "$RES/baseline_myoconverter.json" || fail=1
  fi
  echo "== $NAME: M1"
  if [ -f "$RES/m1_reasons.json" ]; then
    $PY "$REPO/station3/gate_m1.py" "$RES/$LABEL.json" "$RES/baseline_myoconverter.json" "$RES/${LABEL}_m1.json" \
      "$RES/m1_reasons.json" || fail=1
  else
    $PY "$REPO/station3/gate_m1.py" "$RES/$LABEL.json" "$RES/baseline_myoconverter.json" "$RES/${LABEL}_m1.json" || fail=1
  fi
done
[ $fail -eq 0 ] && echo "STATION 3 ($LABEL): ALL GATES PASS" || echo "STATION 3 ($LABEL): GATE FAILURE"
exit $fail
