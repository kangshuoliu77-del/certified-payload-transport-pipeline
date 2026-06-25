#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEMO="${1:-8}"

case "$DEMO" in
  1|demo1) DEMO_NAME="demo1" ;;
  2|demo2) DEMO_NAME="demo2" ;;
  3|demo3) DEMO_NAME="demo3" ;;
  4|demo4) DEMO_NAME="demo4" ;;
  5|demo5) DEMO_NAME="demo5" ;;
  6|demo6) DEMO_NAME="demo6" ;;
  7|demo7) DEMO_NAME="demo7" ;;
  8|demo8) DEMO_NAME="demo8" ;;
  *)
    echo "Unknown demo: $DEMO" >&2
    echo "Use 1-8 or demo1-demo8." >&2
    exit 2
    ;;
esac

export ROS_LOG_DIR="${ROS_LOG_DIR:-$ROOT_DIR/log/ros}"
mkdir -p "$ROS_LOG_DIR"

"$ROOT_DIR/scripts/stop_demo_processes.sh"

source /opt/ros/humble/setup.bash
if [ ! -f "$ROOT_DIR/install/setup.bash" ]; then
  echo "Missing install/setup.bash. Build first:" >&2
  echo "  cd $ROOT_DIR" >&2
  echo "  source /opt/ros/humble/setup.bash" >&2
  echo "  colcon build --packages-select swarm_random_payload" >&2
  exit 3
fi
source "$ROOT_DIR/install/setup.bash"

SHARE_DIR="$(ros2 pkg prefix --share swarm_random_payload)"
CASE_FILE="$SHARE_DIR/data/${DEMO_NAME}_case.json"

if [ ! -f "$CASE_FILE" ]; then
  echo "Missing case file: $CASE_FILE" >&2
  exit 4
fi

echo "[presentation] starting stable RViz cinematic demo: $DEMO_NAME"
echo "[presentation] fixed overview camera, realistic drone mesh, no physics fall/drop transient"

ros2 launch swarm_random_payload cinematic_payload_demo.launch.py \
  case_file:="$CASE_FILE" \
  dt:=0.035 \
  frames_per_tick:=2 \
  initial_hold_seconds:=1.5 \
  final_hold_seconds:=1.5 \
  drone_mesh_scale:=0.38 \
  show_cinematic_hud:=true \
  follow_position_alpha:=0.08 \
  follow_yaw_alpha:=0.05 \
  follow_lock_yaw:=false
