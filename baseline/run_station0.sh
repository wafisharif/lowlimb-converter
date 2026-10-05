#!/bin/bash
# Station 0 baseline: runs inside myoconverter:baseline-cadf380.
# Uses the env's python explicitly (interactive shells auto-activate conda 'base').
set -uo pipefail
PY=/opt/conda/envs/myoconverter/bin/python
cd /app
$PY -c "import mujoco, opensim; print('mujoco', mujoco.__version__); print('opensim', opensim.GetVersionAndDate())" > /app/data/env/versions_container.txt
mkdir -p /app/data/gait2354
start=$(date +%s)
$PY /app/data/run_baseline.py gait2354 > /app/data/gait2354/run.log 2>&1
rc=$?
end=$(date +%s)
echo "gait2354 exit_code=$rc wall_seconds=$((end-start)) start_utc=$(date -u -d @$start +%FT%TZ)" | tee /app/data/gait2354/timing.txt
for spec in "Gait10dof18musc/gait10dof18musc.osim gait10dof18musc" "Gait2354Simbody/gait2354.osim gait2354"; do
  set -- $spec
  $PY /app/data/check_counts.py models/osim/$1 /app/data/$2 > /app/data/$2/gates.txt 2>&1
  echo "== $2"; cat /app/data/$2/gates.txt | grep -E "GATE|OpenSim  |MuJoCo   "
done
echo STATION0_DONE
