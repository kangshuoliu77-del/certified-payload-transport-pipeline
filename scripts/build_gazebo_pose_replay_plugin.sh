#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SRC="$ROOT_DIR/gazebo_pose_replay/PayloadPoseReplaySystem.cc"
OUT_DIR="$ROOT_DIR/build/gazebo_pose_replay"
OUT="$OUT_DIR/libPayloadPoseReplaySystem.so"

mkdir -p "$OUT_DIR"

g++ -std=c++17 -O2 -fPIC -shared "$SRC" \
  -o "$OUT" \
  -I/usr/include/ignition/gazebo6 \
  -I/usr/include/ignition/msgs8 \
  -I/usr/include/ignition/transport11 \
  -I/usr/include/ignition/math6 \
  -I/usr/include/ignition/plugin1 \
  -I/usr/include/ignition/common4 \
  -I/usr/include/ignition/utils1 \
  -I/usr/include/ignition/sdformat12 \
  -I/usr/include/ignition/cmake2 \
  -L/usr/lib/x86_64-linux-gnu \
  -lignition-gazebo6 \
  -lignition-plugin1 \
  -lignition-math6 \
  -lignition-common4

echo "$OUT"
