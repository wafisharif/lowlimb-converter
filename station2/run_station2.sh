#!/bin/bash
# Station 2: kinematic check (station2/GATES.md). Reruns the XML step with the current converter for each
# model, then compares every body against OpenSim. Exit code 0 only if K0-K2 pass for every model run.
# Usage: station2/run_station2.sh <label> <upstream_clone_dir> <opensim_models_clone_dir> <work_dir> [python] [python_mujoco237]
#   label:      results go to station2/<model>/<label>.json (e.g. run1, run2_frames)
#   upstream_clone_dir:       MyoConverter clone at cadf380 (gait models)
#   opensim_models_clone_dir: opensim-org/opensim-models clone at d9b05d4 (Rajagopal2016)
#   work_dir:   scratch folder for conversions and OpenSim reference poses (Rajagopal outputs are not committed)
#   python:     interpreter with env/requirements.lock installed (default: python)
#   python_mujoco237: optional interpreter with mujoco==2.3.7 + numpy; if given, also checks MyoConverter's own
#                     Station 0 output for the gait models (baseline/<model>/<model>_cvt3.xml)
# Environment: MODELS="gait10dof18musc gait2354 Rajagopal2016" (default: all three). Without Rajagopal2016,
#   opensim_models_clone_dir may be "-".
set -uo pipefail
LABEL=$1; UP=$(cd "$2" && pwd); WORK=$4; PY=${5:-python}; PY237=${6:-}
MODELS=${MODELS:-"gait10dof18musc gait2354 Rajagopal2016"}
if [[ " $MODELS " == *" Rajagopal2016 "* ]]; then OSM=$(cd "$3" && pwd) || exit 2; fi
REPO=$(cd "$(dirname "$0")/.." && pwd)
mkdir -p "$WORK"; WORK=$(cd "$WORK" && pwd)
case "$WORK/" in "$REPO"/*) echo "work_dir must be outside the repo (Rajagopal outputs must not be committed)"; exit 2 ;; esac

fail=0
run_model() {  # name osim geometry
  local NAME=$1 OSIM=$2 GEOM=$3 OUT="$WORK/$1" RES="$REPO/station2/$1"
  mkdir -p "$OUT" "$RES"
  echo "== $NAME: XML step"
  $PY "$REPO/station2/convert_xml_step.py" "$OSIM" "$GEOM" "$OUT" > "$OUT/convert.log" 2>&1 \
    || { echo "conversion failed"; tail -20 "$OUT/convert.log"; fail=1; return; }
  echo "== $NAME: OpenSim reference poses"
  $PY "$REPO/station2/osim_reference.py" "$OSIM" "$OUT/reference.npz" 2>&1 | grep -v "^\[" || { fail=1; return; }
  echo "== $NAME: K0-K2 ($LABEL)"
  $PY "$REPO/station2/mujoco_compare.py" "$OUT/${NAME}_cvt1.xml" "$OUT/reference.npz" "$RES/$LABEL.json" || fail=1
  if [ -n "$PY237" ] && [ -f "$REPO/baseline/$NAME/${NAME}_cvt3.xml" ] && [ ! -f "$RES/baseline_myoconverter.json" ]; then
    echo "== $NAME: MyoConverter's own output (MuJoCo 2.3.7, reported only)"
    $PY237 "$REPO/station2/mujoco_compare.py" "$REPO/baseline/$NAME/${NAME}_cvt3.xml" "$OUT/reference.npz" \
      "$RES/baseline_myoconverter.json"
  fi
}

$PY -c "import opensim, mujoco, numpy; print('opensim', opensim.__version__, '| mujoco', mujoco.__version__, '| numpy', numpy.__version__)" || exit 1
for MODEL in $MODELS; do
  case $MODEL in
    gait10dof18musc) run_model gait10dof18musc "$UP/models/osim/Gait10dof18musc/gait10dof18musc.osim" \
                       "$UP/models/osim/Gait10dof18musc/Geometry" ;;
    gait2354)        run_model gait2354 "$UP/models/osim/Gait2354Simbody/gait2354.osim" \
                       "$UP/models/osim/Gait2354Simbody/Geometry" ;;
    Rajagopal2016)
      # meshes: the repo-wide Geometry/ folder, overlaid with the model's own (same as station1/support/README.md)
      GEOM_RAJ="$WORK/rajagopal_geometry"
      if [ ! -d "$GEOM_RAJ" ]; then
        mkdir -p "$GEOM_RAJ" && cp -R "$OSM/Geometry/." "$GEOM_RAJ/" && cp -R "$OSM/Models/Rajagopal/Geometry/." "$GEOM_RAJ/" || exit 1
      fi
      run_model Rajagopal2016 "$OSM/Models/Rajagopal/Rajagopal2016.osim" "$GEOM_RAJ" ;;
    *) echo "unknown model $MODEL"; exit 2 ;;
  esac
done
[ $fail -eq 0 ] && echo "STATION 2 ($LABEL): ALL GATES PASS" || echo "STATION 2 ($LABEL): GATE FAILURE"
exit $fail
