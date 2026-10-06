#!/bin/bash
# Station 0 clean-machine check (run by .github/workflows/station0-baseline.yml inside the
# image built from docker/Dockerfile.locked). Converts gait10dof18musc from scratch, then runs
# the structural gates (check_counts.py) and reproducibility gates R1-R3 (compare_to_upstream.py).
# Exits non-zero unless the pinned versions are present and all 6 gates PASS.
source /opt/conda/etc/profile.d/conda.sh
conda activate myoconverter || { echo "conda activate failed"; exit 1; }
set -uo pipefail
D=/app/data; M=gait10dof18musc
cd /app
python -c "import mujoco, opensim; print('mujoco', mujoco.__version__); print('opensim', opensim.GetVersionAndDate())" | tee $D/env/versions_ci.txt
grep -q "^mujoco 2.3.7$" $D/env/versions_ci.txt && grep -q "^opensim version 4.4.1" $D/env/versions_ci.txt \
  || { echo "FAIL: versions differ from the baseline pins"; exit 1; }

rm -rf $D/$M && mkdir -p $D/$M
start=$(date +%s)
python $D/run_baseline.py $M > $D/$M/run.log 2>&1 || { tail -40 $D/$M/run.log; echo "FAIL: conversion"; exit 1; }
echo "$M wall_seconds=$(( $(date +%s) - start ))" | tee $D/$M/timing.txt

python $D/check_counts.py models/osim/Gait10dof18musc/$M.osim $D/$M > $D/$M/gates.txt 2>&1
python $D/extract_metrics.py $D/$M > /dev/null
python $D/compare_to_upstream.py $D/$M $D/upstream_shipped/$M models/mjc/Gait10dof18musc/${M}_cvt3.xml > $D/$M/reproducibility.txt 2>&1
grep GATE $D/$M/gates.txt; cut -c1-160 $D/$M/reproducibility.txt

n_pass=$(( $(grep -c "GATE: *PASS" $D/$M/gates.txt) + $(grep -cE "^R[123]_[a-z_]+: PASS" $D/$M/reproducibility.txt) ))
echo "gates passed: $n_pass / 6"
[ "$n_pass" -eq 6 ] || { echo "FAIL: not all gates passed"; exit 1; }
echo "CLEAN-MACHINE CHECK PASSED"
