#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEMO="${1:-1}"

case "$DEMO" in
  1|demo1) CASE_FILE="$ROOT_DIR/data/demo1_case.json" ;;
  2|demo2) CASE_FILE="$ROOT_DIR/data/demo2_case.json" ;;
  3|demo3) CASE_FILE="$ROOT_DIR/data/demo3_case.json" ;;
  4|demo4) CASE_FILE="$ROOT_DIR/data/demo4_case.json" ;;
  5|demo5) CASE_FILE="$ROOT_DIR/data/demo5_case.json" ;;
  6|demo6) CASE_FILE="$ROOT_DIR/data/demo6_case.json" ;;
  7|demo7) CASE_FILE="$ROOT_DIR/data/demo7_case.json" ;;
  8|demo8) CASE_FILE="$ROOT_DIR/data/demo8_case.json" ;;
  *)
    echo "Unknown demo: $DEMO" >&2
    echo "Use 1-8 or demo1-demo8." >&2
    exit 2
    ;;
esac

source /opt/ros/humble/setup.bash
if [ ! -f "$ROOT_DIR/install/setup.bash" ]; then
  echo "Missing install/setup.bash. Build first:" >&2
  echo "  cd $ROOT_DIR" >&2
  echo "  source /opt/ros/humble/setup.bash" >&2
  echo "  colcon build --packages-select swarm_random_payload" >&2
  exit 3
fi
source "$ROOT_DIR/install/setup.bash"

ros2 launch swarm_random_payload case_payload_demo.launch.py \
  case_file:="$CASE_FILE" \
  dt:=0.03 \
  frames_per_tick:=3 \
  initial_hold_seconds:=1.0 \
  final_hold_seconds:=1.0
