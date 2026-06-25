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
WORLD_FILE="$ROOT_DIR/out/gazebo/${DEMO_NAME}_gazebo_scene.sdf"

if [ ! -f "$CASE_FILE" ]; then
  echo "Missing case file: $CASE_FILE" >&2
  exit 3
fi

python3 "$ROOT_DIR/tools/export_gazebo_scene.py" \
  --case "$CASE_FILE" \
  --out "$WORLD_FILE"

if ! command -v ign >/dev/null 2>&1; then
  echo "Ignition/Gazebo command 'ign' was not found." >&2
  echo "Install Gazebo Fortress or run only the RViz cinematic demo." >&2
  exit 4
fi

echo "[gazebo] opening $WORLD_FILE"
ign gazebo "$WORLD_FILE"
