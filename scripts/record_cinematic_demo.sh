#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEMO="${1:-8}"
DURATION_SECONDS="${2:-45}"
START_DELAY_SECONDS="${START_DELAY_SECONDS:-6}"
RECORD_DIR="${ROOT_DIR}/recordings"
STAMP="$(date +%Y%m%d_%H%M%S)"

export ROS_LOG_DIR="${ROS_LOG_DIR:-$ROOT_DIR/log/ros}"
mkdir -p "$ROS_LOG_DIR"

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

OUT_FILE="${RECORD_DIR}/${DEMO_NAME}_${STAMP}.mp4"

mkdir -p "$RECORD_DIR"

source /opt/ros/humble/setup.bash
if [ ! -f "$ROOT_DIR/install/setup.bash" ]; then
  echo "Missing install/setup.bash. Build first:" >&2
  echo "  cd $ROOT_DIR" >&2
  echo "  source /opt/ros/humble/setup.bash" >&2
  echo "  colcon build --packages-select swarm_random_payload" >&2
  exit 3
fi
source "$ROOT_DIR/install/setup.bash"

LAUNCH_PID=""
REC_PID=""

cleanup() {
  if [ -n "$REC_PID" ] && kill -0 "$REC_PID" 2>/dev/null; then
    kill -INT "$REC_PID" 2>/dev/null || true
    wait "$REC_PID" 2>/dev/null || true
  fi
  if [ -n "$LAUNCH_PID" ] && kill -0 "$LAUNCH_PID" 2>/dev/null; then
    kill -INT "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

echo "[demo] starting cinematic demo ${DEMO}"
"$ROOT_DIR/scripts/run_cinematic_demo.sh" "$DEMO" &
LAUNCH_PID="$!"

echo "[record] waiting ${START_DELAY_SECONDS}s for RViz to load"
sleep "$START_DELAY_SECONDS"

if command -v wf-recorder >/dev/null 2>&1 && [ "${XDG_SESSION_TYPE:-}" = "wayland" ]; then
  echo "[record] using wf-recorder -> $OUT_FILE"
  wf-recorder -f "$OUT_FILE" &
  REC_PID="$!"
elif command -v ffmpeg >/dev/null 2>&1 && [ -n "${DISPLAY:-}" ]; then
  RECORD_RESOLUTION="${RECORD_RESOLUTION:-1920x1080}"
  RECORD_OFFSET="${RECORD_OFFSET:-0,0}"
  echo "[record] using ffmpeg x11grab ${RECORD_RESOLUTION}+${RECORD_OFFSET} -> $OUT_FILE"
  ffmpeg -y \
    -video_size "$RECORD_RESOLUTION" \
    -framerate 30 \
    -f x11grab \
    -i "${DISPLAY}+${RECORD_OFFSET}" \
    -codec:v libx264 \
    -preset veryfast \
    -pix_fmt yuv420p \
    "$OUT_FILE" &
  REC_PID="$!"
else
  echo "[record] no supported screen recorder found." >&2
  echo "Install one of these, then run this script again:" >&2
  echo "  sudo apt install ffmpeg" >&2
  echo "  sudo apt install wf-recorder   # Wayland sessions" >&2
  echo "The demo is still running; press Ctrl-C when done." >&2
  wait "$LAUNCH_PID"
  exit 5
fi

sleep "$DURATION_SECONDS"
echo "[record] stopping after ${DURATION_SECONDS}s"
cleanup
trap - EXIT INT TERM

echo "[OK] wrote $OUT_FILE"
