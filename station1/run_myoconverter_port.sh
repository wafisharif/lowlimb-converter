#!/bin/bash
# MyoConverter's own conversion of a model it never shipped one for (Rajagopal 2016), for the "MyoConverter" column of
# Stations 3-5: the full pipeline with the Station 1a port (commit 1670ae3 = upstream @ cadf380 plus the four
# MuJoCo 3 / NumPy 2 fixes, shown equal to upstream's own output to 1e-16 on both gait models; station1/FINDINGS.md),
# with the same settings as station1/run_port.py except MyoConverter's own validation plots and PDF, which are off: they
# read the converted models and never write them (O2MStep1.vlt1_*, O2MStep2.compMomentArmResults,
# O2MStep3.compMuscleForceResults), and for Rajagopal 2016 they take hours.
# Usage: station1/run_myoconverter_port.sh <model> <models_clone_dir> <output_dir> <geometry_dir> [python]
#   (arguments as for station1/run_port.py; output_dir must not exist yet)
set -euo pipefail
NAME=$1; MODELS_DIR=$(cd "$2" && pwd); OUT=$3; GEOM=$(cd "$4" && pwd); PY=${5:-python}
REPO=$(cd "$(dirname "$0")/.." && pwd)
if [ -e "$OUT" ]; then echo "$OUT already exists"; exit 2; fi
mkdir -p "$OUT/port_src"; OUT=$(cd "$OUT" && pwd)
git -C "$REPO" archive 1670ae3 src | tar -x -C "$OUT/port_src"
case $NAME in
  Rajagopal2016|RajagopalLaiUhlrich2023) OSIM_REL="Models/Rajagopal/$NAME.osim" ;;
  *) echo "unknown model $NAME (the gait models' MyoConverter column is baseline/, Station 0)"; exit 2 ;;
esac
cd "$MODELS_DIR"
"$PY" - "$OUT/port_src/src" "$OSIM_REL" "$GEOM" "$OUT/converted" <<'PYEOF'
import sys
sys.path.insert(0, sys.argv[1])
from myoconverter.O2MPipeline import O2MPipeline
O2MPipeline(sys.argv[2], sys.argv[3], sys.argv[4], convert_steps=[1, 2, 3], muscle_list=None, osim_data_overwrite=True,
            conversion=True, validation=False, speedy=False, generate_pdf=False, add_ground_geom=True,
            treat_as_normal_path_point=False)
PYEOF
ls "$OUT/converted/${NAME}_cvt3.xml"
