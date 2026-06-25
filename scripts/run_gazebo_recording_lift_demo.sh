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

CASE_FILE="$ROOT_DIR/data/${DEMO_NAME}_case.json"
WORLD_FILE="$ROOT_DIR/out/gazebo/${DEMO_NAME}_gazebo_recording_lift.sdf"
QP_TRAJECTORY_FILE="$ROOT_DIR/out/gazebo/${DEMO_NAME}_paper_qp_trajectory.json"
RECORDING_TRAJECTORY_FILE="$ROOT_DIR/out/gazebo/${DEMO_NAME}_recording_uniform_trajectory.json"
POSE_STREAM_FILE="$ROOT_DIR/out/gazebo/${DEMO_NAME}_recording_lift_pose_stream.csv"
PLUGIN_LIB="$ROOT_DIR/build/gazebo_pose_replay/libPayloadPoseReplaySystem.so"
REPLAY_RATE="${REPLAY_RATE:-13.3}"
GAZEBO_PID=""

cleanup() {
  if [ -n "$GAZEBO_PID" ] && kill -0 "$GAZEBO_PID" 2>/dev/null; then
    kill -INT "$GAZEBO_PID" 2>/dev/null || true
    wait "$GAZEBO_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

if [ ! -f "$CASE_FILE" ]; then
  echo "Missing case file: $CASE_FILE" >&2
  exit 3
fi

"$ROOT_DIR/scripts/stop_demo_processes.sh"

source /opt/ros/humble/setup.bash
mkdir -p "$(dirname "$WORLD_FILE")"

echo "[gazebo-recording-lift] exporting Gazebo world for ${DEMO_NAME}"
python3 "$ROOT_DIR/tools/export_gazebo_scene.py" \
  --case "$CASE_FILE" \
  --out "$WORLD_FILE" \
  --map-scale 0.02 \
  --mesh-scale 1.00 \
  --drone-altitude 1.65 \
  --obstacle-height 2.00

if [ ! -f "$QP_TRAJECTORY_FILE" ] || [ "$QP_TRAJECTORY_FILE" -ot "$CASE_FILE" ]; then
  echo "[gazebo-recording-lift] exporting paper-QP trajectory cache at visual altitude 1.65m"
  python3 "$ROOT_DIR/tools/export_gazebo_trajectory.py" \
    --case "$CASE_FILE" \
    --out "$QP_TRAJECTORY_FILE" \
    --map-scale 0.02 \
    --altitude 1.65 \
    --pickup-altitude 0.62 \
    --payload-drop 0.36 \
    --payload-ground-height 0.12 \
    --event-steps 6 \
    --start-hold-steps 8 \
    --frame-stride 5 \
    --visual-rate 30.0 \
    --vehicle-mass 1.6 \
    --payload-mass 0.6 \
    --hover-rotor-speed 52.0 \
    --acceleration-limit 3.2
fi

if [ ! -f "$RECORDING_TRAJECTORY_FILE" ] || [ "$RECORDING_TRAJECTORY_FILE" -ot "$QP_TRAJECTORY_FILE" ]; then
  echo "[gazebo-recording-lift] freezing cached QP trajectory into uniform recording path"
  python3 "$ROOT_DIR/tools/freeze_uniform_recording_trajectory.py" \
    --input "$QP_TRAJECTORY_FILE" \
    --out "$RECORDING_TRAJECTORY_FILE" \
    --frames 960
else
  echo "[gazebo-recording-lift] reusing frozen uniform recording path: $RECORDING_TRAJECTORY_FILE"
fi

echo "[gazebo-recording-lift] exporting pose stream with damped payload lift"
python3 "$ROOT_DIR/tools/export_gazebo_pose_stream.py" \
  --trajectory "$RECORDING_TRAJECTORY_FILE" \
  --case "$CASE_FILE" \
  --out "$POSE_STREAM_FILE" \
  --rate "$REPLAY_RATE" \
  --rotor-speed 52.0 \
  --camera-target-z-offset 0.85 \
  --camera-target-ahead-distance 0.75 \
  --camera-target-smoothing 0.035 \
  --camera-target-yaw-smoothing 0.06 \
  --yaw-smoothing 0.45 \
  --highlight-lookahead-fraction 0.90 \
  --payload-lift-physics \
  --payload-physics-stiffness 6.5 \
  --payload-physics-damping 2.8 \
  --payload-ground-height 0.12 \
  --start-hold-seconds 1.0 \
  --cable-switch-height 0.90

echo "[gazebo-recording-lift] building Gazebo in-process replay plugin"
"$ROOT_DIR/scripts/build_gazebo_pose_replay_plugin.sh" >/dev/null

python3 - "$WORLD_FILE" "$PLUGIN_LIB" "$POSE_STREAM_FILE" "$REPLAY_RATE" <<'PY'
from pathlib import Path
import sys

world = Path(sys.argv[1])
plugin = Path(sys.argv[2]).resolve()
trajectory = Path(sys.argv[3]).resolve()
rate = sys.argv[4]
text = world.read_text(encoding="utf-8")
block = f"""
    <plugin filename="{plugin}" name="payload_demo::PayloadPoseReplaySystem">
      <trajectory>{trajectory}</trajectory>
      <rate>{rate}</rate>
      <loop>true</loop>
    </plugin>
"""
if "payload_demo::PayloadPoseReplaySystem" not in text:
    text = text.replace("  </world>", block + "  </world>", 1)
world.write_text(text, encoding="utf-8")
PY

echo "[gazebo-recording-lift] opening $WORLD_FILE"
env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u ALL_PROXY -u all_proxy \
  ign gazebo -r "$WORLD_FILE" &
GAZEBO_PID="$!"

wait "$GAZEBO_PID"
