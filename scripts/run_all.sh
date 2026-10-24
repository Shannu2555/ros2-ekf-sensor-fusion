#!/bin/bash
# All runs behind the README numbers. Each run is a fresh Gazebo instance, 5 laps.
# Usage: scripts/run_all.sh <out_dir>
OUT=${1:-$HOME/fusion_logs}
HERE=$(cd "$(dirname "$0")" && pwd)
bash "$HERE/run_scenario.sh" floor    measured floor               "$OUT"
bash "$HERE/run_scenario.sh" slippery measured slippery            "$OUT"
bash "$HERE/run_scenario.sh" slippery exact    slippery_exact_calib "$OUT"
# sanity check: what Gazebo gives you with perfect geometry and no slip
bash "$HERE/run_scenario.sh" floor    exact    floor_exact_calib    "$OUT"
