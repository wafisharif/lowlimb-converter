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
# Environment: OURS_CONVERTED=<folder> uses that full-pipeline conversion (by the same converter) instead of
#   converting again (one model per run). MODELS="gait10dof18musc gait2354" (default: both); add Rajagopal2016 with OSM=<opensim-models clone at
#   d9b05d4>. Its MyoConverter column (no Station 0 output exists) is the Station 1a port's own conversion
#   (station1/run_myoconverter_port.sh; MC_CONVERTED=<its output folder> reuses one), made once if
#   station3/Rajagopal2016/baseline_myoconverter.json is missing.
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
    Rajagopal2016)
      [ -n "${OSM:-}" ] || { echo "Rajagopal2016 needs OSM=<opensim-models clone>"; exit 2; }
      OSM=$(cd "$OSM" && pwd); OSIM="$OSM/Models/Rajagopal/Rajagopal2016.osim"
      # meshes: the repo-wide Geometry/ folder, overlaid with the model's own (station1/support/README.md)
      GEOM="$WORK/rajagopal_geometry"
      if [ ! -d "$GEOM" ]; then
        mkdir -p "$GEOM" && cp -R "$OSM/Geometry/." "$GEOM/" && cp -R "$OSM/Models/Rajagopal/Geometry/." "$GEOM/" || exit 1
      fi ;;
    *) echo "unknown model $NAME"; exit 2 ;;
  esac
  case $NAME in Rajagopal2016) PORT_ARGS=("$OSM" "" "$GEOM") ;; *) PORT_ARGS=("$UP" "") ;; esac
  # A fresh folder per run: the pipeline resumes Steps 2-3 from any cvt2/cvt3 already in its output folder
  # (O2MSteps.py), which would silently reuse an older model.
  OUT="$WORK/$LABEL/$NAME"; RES="$REPO/station3/$NAME"
  if [ -e "$OUT" ]; then echo "$OUT already exists; use a new label or remove it"; exit 2; fi
  mkdir -p "$OUT" "$RES"
  echo "== $NAME: full pipeline (current converter)"
  start=$(date +%s)
  PORT_ARGS[1]="$OUT"
  if [ -n "${OURS_CONVERTED:-}" ]; then
    # an earlier full-pipeline conversion by the same converter (e.g. shared by Stations 3-5), copied in
    cp -R "$OURS_CONVERTED/." "$OUT/" && echo "   reused $OURS_CONVERTED" > "$OUT/run.log" || { fail=1; continue; }
  else
    $PY "$REPO/station1/run_port.py" "$NAME" "${PORT_ARGS[@]}" > "$OUT/run.log" 2>&1 \
      || { echo "pipeline failed"; tail -20 "$OUT/run.log"; fail=1; continue; }
  fi
  echo "   done in $(( $(date +%s) - start )) s"
  echo "== $NAME: OpenSim lengths and moment arms"
  # Rajagopal2016: OpenSim moment arms as -dL/dq (station3/GATES.md, amendment 2026-10-12: wrapped paths)
  MA_DEF=""; [ "$NAME" = Rajagopal2016 ] && MA_DEF="--moment-arm length-derivative"
  $PY "$REPO/station3/osim_paths.py" "$OSIM" "$OUT/paths_reference.npz" $MA_DEF 2>&1 | grep -v "^\[" \
    || { fail=1; continue; }
  echo "== $NAME: ours ($LABEL)"
  $PY "$REPO/station3/mujoco_paths.py" "$OUT/${NAME}_cvt3.xml" "$OUT/paths_reference.npz" "$RES/$LABEL.json" || fail=1
  if [ ! -f "$RES/baseline_myoconverter.json" ] && [ "$NAME" = Rajagopal2016 ]; then
    echo "== $NAME: MyoConverter's own conversion (Station 1a port, MuJoCo 3.15)"
    if [ -n "${MC_CONVERTED:-}" ]; then MC=$(cd "$MC_CONVERTED" && pwd)   # an earlier run of the port, reused
    else
      MC="$WORK/$LABEL/${NAME}_myoconverter"
      bash "$REPO/station1/run_myoconverter_port.sh" "$NAME" "$OSM" "$MC" "$GEOM" "$PY" > "$MC.log" 2>&1 \
        || { echo "MyoConverter port failed"; tail -20 "$MC.log"; fail=1; continue; }
      MC="$MC/converted"
    fi
    $PY "$REPO/station3/mujoco_paths.py" "$MC/${NAME}_cvt3.xml" "$OUT/paths_reference.npz" \
      "$RES/baseline_myoconverter.json" || fail=1
  fi
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
