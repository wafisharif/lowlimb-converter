#!/bin/bash
# Station 4: muscle forces (station4/GATES.md). Runs the full pipeline with the current converter on each gait model,
# then compares every muscle's static force with OpenSim's at several activations, for ours and MyoConverter's.
# Exit code 0 only if F0, F1 and F2 pass for every model run.
# Usage: station4/run_station4.sh <label> <upstream_clone_dir> <work_dir> [python] [python_mujoco237]
#   label:      results go to station4/<model>/<label>.json and <label>_f.json (e.g. run1)
#   upstream_clone_dir: MyoConverter clone at cadf380 (gait models)
#   work_dir:   scratch folder outside the repo for conversions and OpenSim reference data
#   python:     interpreter with env/requirements.lock installed (default: python)
#   python_mujoco237: interpreter with mujoco==2.3.7 + numpy, for the MyoConverter column (computed once per model;
#                     without it, an existing station4/<model>/baseline_myoconverter.json is used)
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
  OUT="$WORK/$LABEL/$NAME"; RES="$REPO/station4/$NAME"
  if [ -e "$OUT" ]; then echo "$OUT already exists; use a new label or remove it"; exit 2; fi
  mkdir -p "$OUT" "$RES"
  echo "== $NAME: full pipeline (current converter)"
  start=$(date +%s)
  $PY "$REPO/station1/run_port.py" "$NAME" "$UP" "$OUT" > "$OUT/run.log" 2>&1 \
    || { echo "pipeline failed"; tail -20 "$OUT/run.log"; fail=1; continue; }
  echo "   done in $(( $(date +%s) - start )) s"
  echo "== $NAME: OpenSim static forces"
  $PY "$REPO/station4/osim_forces.py" "$OSIM" "$OUT/forces_reference.npz" 2>&1 | grep -v "^\[" || { fail=1; continue; }
  echo "== $NAME: ours ($LABEL)"
  $PY "$REPO/station4/mujoco_forces.py" "$OUT/${NAME}_cvt3.xml" "$OUT/forces_reference.npz" "$RES/$LABEL.json" || fail=1
  if [ ! -f "$RES/baseline_myoconverter.json" ]; then
    if [ -z "$PY237" ]; then echo "no MyoConverter column: pass python_mujoco237"; fail=1; continue; fi
    echo "== $NAME: MyoConverter's own output (MuJoCo 2.3.7)"
    $PY237 "$REPO/station4/mujoco_forces.py" "$REPO/baseline/$NAME/${NAME}_cvt3.xml" "$OUT/forces_reference.npz" \
      "$RES/baseline_myoconverter.json" || fail=1
  fi
  echo "== $NAME: F1-F2"
  if [ -f "$RES/f_reasons.json" ]; then
    $PY "$REPO/station4/gate_f.py" "$RES/$LABEL.json" "$RES/baseline_myoconverter.json" "$RES/${LABEL}_f.json" \
      "$RES/f_reasons.json" || fail=1
  else
    $PY "$REPO/station4/gate_f.py" "$RES/$LABEL.json" "$RES/baseline_myoconverter.json" "$RES/${LABEL}_f.json" || fail=1
  fi
done
[ $fail -eq 0 ] && echo "STATION 4 ($LABEL): ALL GATES PASS" || echo "STATION 4 ($LABEL): GATE FAILURE"
exit $fail
