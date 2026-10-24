#!/bin/bash
# One headless run: Gazebo server + bridge + fusion stack. Writes <out_dir>/<tag>.csv plus logs.
# Usage: scripts/run_scenario.sh <floor|slippery> <measured|exact> <tag> <out_dir> [workspace]
set -e
WORLD=${1:-floor}
CALIB=${2:-measured}
TAG=${3:-$WORLD}
OUT=$(realpath -m "${4:-$HOME/fusion_logs}")
WS=${5:-$HOME/fusion_ws}
REPO=${REPO:-$(cd "$(dirname "$0")/.." && pwd)}

source /opt/ros/humble/setup.bash
mkdir -p "$WS/src" "$OUT"
rsync -a --delete --exclude .git --exclude results --exclude docs "$REPO/" "$WS/src/ekf_fusion/"
cd "$WS" && colcon build --symlink-install >/dev/null 2>&1 || colcon build --symlink-install
source "$WS/install/setup.bash"
export ROS_DOMAIN_ID=$((RANDOM % 100))

ros2 launch ekf_fusion sim.launch.py world:=$WORLD gz_extra:=-s > "$OUT/sim_$TAG.log" 2>&1 &
SIM=$!
sleep 12
ros2 launch ekf_fusion fusion.launch.py tag:=$TAG calibration:=$CALIB out_dir:=$OUT \
  > "$OUT/fusion_$TAG.log" 2>&1 || true
kill -INT $SIM 2>/dev/null || true
sleep 3
pkill -f "ign gazebo" 2>/dev/null || true
pkill -f parameter_bridge 2>/dev/null || true
pkill -f robot_state_publisher 2>/dev/null || true
echo "done: $OUT/$TAG.csv ($(wc -l < "$OUT/$TAG.csv") rows)"
