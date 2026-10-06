#!/bin/bash
# Station 0 baseline. Runs inside myoconverter:baseline-cadf380.
# Calls the env's python directly (interactive shells auto-activate conda 'base').
# gait10dof18musc was already converted on 2026-10-05; this converts gait2354 (skipped if done),
# then runs structural gates, metric extraction and the reproducibility check on both models.
set -uo pipefail
PY=/opt/conda/envs/myoconverter/bin/python
D=/app/data
cd /app
$PY -c "import mujoco, opensim; print('mujoco', mujoco.__version__); print('opensim', opensim.GetVersionAndDate())" > $D/env/versions_container.txt

mkdir -p $D/gait2354
if ls $D/gait2354/*_cvt3.xml >/dev/null 2>&1; then
  echo "gait2354 already converted; skipping conversion"
else
  echo "Converting gait2354 (expect 15-30+ min)..."
  start=$(date +%s)
  $PY $D/run_baseline.py gait2354 > $D/gait2354/run.log 2>&1
  rc=$?
  end=$(date +%s)
  echo "gait2354 exit_code=$rc wall_seconds=$((end-start)) start_utc=$(date -u -d @$start +%FT%TZ)" | tee $D/gait2354/timing.txt
fi

for spec in "Gait10dof18musc gait10dof18musc gait10dof18musc" "Gait2354Simbody gait2354 gait2354"; do
  set -- $spec   # $1 upstream folder, $2 our folder, $3 osim/xml stem
  echo "================ $2"
  $PY $D/check_counts.py models/osim/$1/$3.osim $D/$2 > $D/$2/gates.txt 2>&1
  grep -E "GATE" $D/$2/gates.txt
  $PY $D/extract_metrics.py $D/$2 > /dev/null 2> $D/$2/extract_metrics.err && echo "metrics: written" || echo "metrics: ERROR (see extract_metrics.err)"
  $PY $D/compare_to_upstream.py $D/$2 $D/upstream_shipped/$2 models/mjc/$1/$3_cvt3.xml > $D/$2/reproducibility.txt 2>&1
  cut -c1-140 $D/$2/reproducibility.txt
done
echo STATION0_DONE
