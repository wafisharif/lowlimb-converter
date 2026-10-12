#!/bin/bash
# Station 5: walking (station5/GATES.md). Runs the full pipeline with the current converter on each gait model, then
# replays OpenSim's walking data in OpenSim and MuJoCo: muscle lengths and moment arms along the gait cycle, and the
# joint moments needed to walk with the measured ground reaction forces, for our output and MyoConverter's.
# Then level 3 (reported only, never fails the run): open-loop forward replay of OpenSim's CMC muscle excitations in
# both simulators, with the measured ground reaction forces (as gated) and, as a diagnostic, without them.
# Exit code 0 only if W0-W3 pass for every model run.
# Usage: station5/run_station5.sh <label> <upstream_clone_dir> <opensim_models_clone_dir> <work_dir> [python] [python_mujoco237]
#   label:      results go to station5/<model>/<label>.json and <label>_w.json (e.g. run1), level 3 to
#               <label>_replay.json and <label>_replay_nogrf.json
#   upstream_clone_dir:       MyoConverter clone at cadf380 (gait models)
#   opensim_models_clone_dir: opensim-org/opensim-models clone at d9b05d4 (walking data, Pipelines/)
#   work_dir:   scratch folder outside the repo for conversions and OpenSim reference data
#   python:     interpreter with env/requirements.lock installed (default: python)
#   python_mujoco237: interpreter with mujoco==2.3.7 + numpy, for the MyoConverter column (computed once per model;
#                     without it, an existing station5/<model>/baseline_myoconverter.json is used)
# Environment: MODELS="gait10dof18musc gait2354" (default: both).
set -uo pipefail
LABEL=$1; UP=$(cd "$2" && pwd); OSM=$(cd "$3" && pwd); WORK=$4; PY=${5:-python}; PY237=${6:-}
MODELS=${MODELS:-"gait10dof18musc gait2354"}
REPO=$(cd "$(dirname "$0")/.." && pwd)
mkdir -p "$WORK"; WORK=$(cd "$WORK" && pwd)
case "$WORK/" in "$REPO"/*) echo "work_dir must be outside the repo"; exit 2 ;; esac

fail=0
$PY -c "import opensim, mujoco, numpy; print('opensim', opensim.__version__, '| mujoco', mujoco.__version__, '| numpy', numpy.__version__)" || exit 1
for NAME in $MODELS; do
  case $NAME in
    gait10dof18musc)
      OSIM="$UP/models/osim/Gait10dof18musc/gait10dof18musc.osim"
      IK="$OSM/Pipelines/Gait10dof18musc/OutputReference/IK/subject01_walk_IK.mot"
      GRF="$OSM/Pipelines/Gait10dof18musc/ExperimentalData/subject01_walk_grf.xml"
      RANGE=""
      CMC="$OSM/Pipelines/Gait10dof18musc/OutputReference/CMC/ResultsCMC/walk_subject_controls.sto" ;;
    gait2354)
      OSIM="$UP/models/osim/Gait2354Simbody/gait2354.osim"
      IK="$OSM/Pipelines/Gait2354_Simbody/subject01_walk1_ik.mot"
      GRF="$OSM/Pipelines/Gait2354_Simbody/subject01_walk1_grf.xml"
      RANGE="--start 0.4 --end 1.6"
      CMC="$OSM/Pipelines/Gait2354_Simbody/OutputReference/ResultsCMC/subject01_walk1_controls.sto" ;;
    *) echo "unknown model $NAME"; exit 2 ;;
  esac
  # A fresh folder per run: the pipeline resumes Steps 2-3 from any cvt2/cvt3 already in its output folder
  # (O2MSteps.py), which would silently reuse an older model.
  OUT="$WORK/$LABEL/$NAME"; RES="$REPO/station5/$NAME"
  if [ -e "$OUT" ]; then echo "$OUT already exists; use a new label or remove it"; exit 2; fi
  mkdir -p "$OUT" "$RES"
  echo "== $NAME: full pipeline (current converter)"
  start=$(date +%s)
  $PY "$REPO/station1/run_port.py" "$NAME" "$UP" "$OUT" > "$OUT/run.log" 2>&1 \
    || { echo "pipeline failed"; tail -20 "$OUT/run.log"; fail=1; continue; }
  echo "   done in $(( $(date +%s) - start )) s"
  echo "== $NAME: OpenSim walking reference (paths, inverse dynamics)"
  # shellcheck disable=SC2086
  $PY "$REPO/station5/osim_gait.py" "$OSIM" "$IK" "$GRF" "$OUT/gait_reference.npz" $RANGE 2>&1 | grep -v "^\[" \
    || { fail=1; continue; }
  echo "== $NAME: ours ($LABEL)"
  $PY "$REPO/station5/mujoco_gait.py" "$OUT/${NAME}_cvt3.xml" "$OUT/gait_reference.npz" "$RES/$LABEL.json" || fail=1
  if [ ! -f "$RES/baseline_myoconverter.json" ]; then
    if [ -z "$PY237" ]; then echo "no MyoConverter column: pass python_mujoco237"; fail=1; continue; fi
    echo "== $NAME: MyoConverter's own output (MuJoCo 2.3.7)"
    $PY237 "$REPO/station5/mujoco_gait.py" "$REPO/baseline/$NAME/${NAME}_cvt3.xml" "$OUT/gait_reference.npz" \
      "$RES/baseline_myoconverter.json" || fail=1
  fi
  echo "== $NAME: W1-W3"
  if [ -f "$RES/w_reasons.json" ]; then
    $PY "$REPO/station5/gate_w.py" "$RES/$LABEL.json" "$RES/baseline_myoconverter.json" "$RES/${LABEL}_w.json" \
      "$RES/w_reasons.json" || fail=1
  else
    $PY "$REPO/station5/gate_w.py" "$RES/$LABEL.json" "$RES/baseline_myoconverter.json" "$RES/${LABEL}_w.json" || fail=1
  fi
  echo "== $NAME: level 3, forward replay (reported only)"
  for V in replay replay_nogrf; do
    EXTRA=""; [ "$V" = replay_nogrf ] && EXTRA="--grf-off"
    # shellcheck disable=SC2086
    $PY "$REPO/station5/osim_replay.py" "$OSIM" "$IK" "$GRF" "$CMC" "$OUT/${V}_reference.npz" $EXTRA 2>&1 \
      | grep -v "^\[" || { echo "   (OpenSim replay failed; level 3 not reported)"; continue; }
    $PY "$REPO/station5/mujoco_replay.py" "$OUT/${NAME}_cvt3.xml" "$OUT/${V}_reference.npz" "$RES/${LABEL}_${V}.json" \
      || echo "   (MuJoCo replay failed)"
    if [ ! -f "$RES/baseline_myoconverter_${V}.json" ] && [ -n "$PY237" ]; then
      $PY237 "$REPO/station5/mujoco_replay.py" "$REPO/baseline/$NAME/${NAME}_cvt3.xml" "$OUT/${V}_reference.npz" \
        "$RES/baseline_myoconverter_${V}.json" || echo "   (MyoConverter replay failed)"
    fi
  done
done
[ $fail -eq 0 ] && echo "STATION 5 ($LABEL): ALL GATES PASS" || echo "STATION 5 ($LABEL): GATE FAILURE"
exit $fail
