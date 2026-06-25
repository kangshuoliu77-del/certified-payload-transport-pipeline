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
WORLD_FILE="$ROOT_DIR/out/gazebo/${DEMO_NAME}_gazebo_cinematic.sdf"
TRAJECTORY_FILE="$ROOT_DIR/out/gazebo/${DEMO_NAME}_paper_qp_trajectory.json"
GAZEBO_PID=""
REPLAY_PID=""

cleanup() {
  if [ -n "$REPLAY_PID" ] && kill -0 "$REPLAY_PID" 2>/dev/null; then
    kill -INT "$REPLAY_PID" 2>/dev/null || true
    wait "$REPLAY_PID" 2>/dev/null || true
  fi
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

echo "[gazebo-cinematic] exporting stable Gazebo replay world for ${DEMO_NAME}"
python3 "$ROOT_DIR/tools/export_gazebo_scene.py" \
  --case "$CASE_FILE" \
  --out "$WORLD_FILE" \
  --map-scale 0.02 \
  --mesh-scale 1.00 \
  --drone-altitude 1.65 \
  --obstacle-height 2.00

if python3 - "$TRAJECTORY_FILE" "$CASE_FILE" <<'PY'
import json
import sys
from pathlib import Path

trajectory = Path(sys.argv[1])
case_file = Path(sys.argv[2])
if not trajectory.exists() or trajectory.stat().st_mtime < case_file.stat().st_mtime:
    raise SystemExit(1)
data = json.loads(trajectory.read_text(encoding="utf-8"))
ok = (
    data.get("replay_frames")
    and abs(float(data.get("map_scale", -1.0)) - 0.02) < 1e-9
    and abs(float(data.get("altitude", -1.0)) - 1.65) < 1e-9
    and abs(float(data.get("pickup_altitude", -1.0)) - 0.62) < 1e-9
    and abs(float(data.get("payload_drop", -1.0)) - 0.36) < 1e-9
    and data.get("schema") == "gazebo_paper_qp_trajectory_v5"
    and int(data.get("event_steps", -1)) == 6
    and int(data.get("start_hold_steps", -1)) == 8
    and int(data.get("frame_stride", -1)) == 5
    and abs(float(data.get("visual_rate", -1.0)) - 30.0) < 1e-9
)
raise SystemExit(0 if ok else 1)
PY
then
  echo "[gazebo-cinematic] reusing paper-QP trajectory cache: $TRAJECTORY_FILE"
else
  echo "[gazebo-cinematic] exporting paper-QP trajectory cache at visual altitude 1.65m"
  python3 "$ROOT_DIR/tools/export_gazebo_trajectory.py" \
    --case "$CASE_FILE" \
    --out "$TRAJECTORY_FILE" \
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

echo "[gazebo-cinematic] opening $WORLD_FILE"
echo "[gazebo-cinematic] replay starts automatically after Gazebo loads"
env -u http_proxy -u https_proxy -u HTTP_PROXY -u HTTPS_PROXY -u ALL_PROXY -u all_proxy \
  ign gazebo -r "$WORLD_FILE" &
GAZEBO_PID="$!"

python3 "$ROOT_DIR/tools/replay_gazebo_trajectory.py" \
  --trajectory "$TRAJECTORY_FILE" \
  --world payload_transport_scene \
  --rate 270.0 \
  --start-delay 5.0 \
  --payload-drop 0.36 \
  --payload-ground-height 0.12 \
  --payload-mass 0.6 \
  --rope-stiffness 55.0 \
  --rope-damping 4.0 \
  --payload-air-damping 0.35 \
  --payload-physics-substeps 1 \
  --drone-mesh-scale 1.00 \
  --rotor-speed 52.0 \
  --frame-step 3 \
  --substeps 1 \
  --max-replay-step 0.0 \
  --max-interpolation-substeps 1 \
  --camera-target-z-offset 0.85 \
  --camera-target-ahead-distance 0.75 \
  --camera-target-smoothing 0.16 \
  --camera-target-yaw-smoothing 0.20 \
  --yaw-smoothing 0.45 \
  --highlight-lookahead-fraction 0.90 \
  --loop &
REPLAY_PID="$!"

wait "$GAZEBO_PID"
