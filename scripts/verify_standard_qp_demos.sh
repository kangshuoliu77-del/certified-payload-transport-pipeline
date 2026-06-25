#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT_DIR"

python3 tools/verify_cases.py \
  data/demo1_case.json \
  data/demo2_case.json \
  data/demo3_case.json \
  data/demo4_case.json \
  data/demo5_case.json \
  data/demo6_case.json \
  data/demo7_case.json \
  data/demo8_case.json
