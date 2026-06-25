#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

python3 tools/rebuild_cases.py \
  --case demo1 \
  --case demo2 \
  --case demo3 \
  --case demo4 \
  --case demo5 \
  --case demo6 \
  --case demo7 \
  --case demo8
