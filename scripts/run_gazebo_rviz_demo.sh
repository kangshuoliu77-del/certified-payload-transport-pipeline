#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEMO="${1:-8}"

GAZEBO_PID=""
RVIZ_PID=""

cleanup() {
  if [ -n "$RVIZ_PID" ] && kill -0 "$RVIZ_PID" 2>/dev/null; then
    kill -INT "$RVIZ_PID" 2>/dev/null || true
    wait "$RVIZ_PID" 2>/dev/null || true
  fi
  if [ -n "$GAZEBO_PID" ] && kill -0 "$GAZEBO_PID" 2>/dev/null; then
    kill -INT "$GAZEBO_PID" 2>/dev/null || true
    wait "$GAZEBO_PID" 2>/dev/null || true
  fi
}
trap cleanup EXIT INT TERM

"$ROOT_DIR/scripts/run_gazebo_scene.sh" "$DEMO" &
GAZEBO_PID="$!"

sleep 3

"$ROOT_DIR/scripts/run_cinematic_demo.sh" "$DEMO" &
RVIZ_PID="$!"

wait "$RVIZ_PID"
