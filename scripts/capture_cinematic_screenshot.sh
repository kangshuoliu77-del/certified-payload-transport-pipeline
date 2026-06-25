#!/usr/bin/env bash
set -eo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT_DIR="${ROOT_DIR}/screenshots"
STAMP="$(date +%Y%m%d_%H%M%S)"
OUT_FILE="${1:-${OUT_DIR}/cinematic_rviz_${STAMP}.png}"
mkdir -p "$(dirname "$OUT_FILE")"

TMP_WINDOWS="$(mktemp)"
xwininfo -root -tree > "$TMP_WINDOWS"
WINDOW_ID="$(awk '/cinematic_payload_demo\.rviz.*RViz/ {print $1; exit}' "$TMP_WINDOWS")"
rm -f "$TMP_WINDOWS"
if [ -z "$WINDOW_ID" ]; then
  echo "Could not find an RViz window using cinematic_payload_demo.rviz." >&2
  echo "Start one first:" >&2
  echo "  ./scripts/run_cinematic_demo.sh 8" >&2
  exit 2
fi

WINDOW_ID="$WINDOW_ID" python3 - <<'PY' || true
import os

try:
    from Xlib import X, display
    from Xlib.protocol import event
except Exception:
    raise SystemExit(0)

d = display.Display()
root = d.screen().root
window_id = int(os.environ["WINDOW_ID"], 16)
win = d.create_resource_object("window", window_id)
win.configure(stack_mode=X.Above)
try:
    net_active_window = d.intern_atom("_NET_ACTIVE_WINDOW")
    root.send_event(
        event.ClientMessage(
            window=win,
            client_type=net_active_window,
            data=(32, [1, X.CurrentTime, 0, 0, 0]),
        ),
        event_mask=X.SubstructureRedirectMask | X.SubstructureNotifyMask,
    )
except Exception:
    pass
try:
    win.set_input_focus(X.RevertToParent, X.CurrentTime)
except Exception:
    pass
d.sync()
PY

sleep 0.4
TMP_XWD="$(mktemp --suffix=.xwd)"
xwd -id "$WINDOW_ID" -silent -out "$TMP_XWD"

TMP_XWD="$TMP_XWD" OUT_FILE="$OUT_FILE" python3 - <<'PY'
from pathlib import Path
import os
import struct

import numpy as np
from PIL import Image

src = Path(os.environ["TMP_XWD"])
dst = Path(os.environ["OUT_FILE"])
payload = src.read_bytes()
header = struct.unpack(">25I", payload[:100])
header_size = header[0]
width = header[4]
height = header[5]
bytes_per_line = header[12]
ncolors = header[19]
offset = header_size + ncolors * 12

raw = np.frombuffer(payload, dtype=np.uint8, count=bytes_per_line * height, offset=offset).reshape(
    (height, bytes_per_line)
)

if bytes_per_line >= width * 4:
    px = raw[:, : width * 4].reshape((height, width, 4))
    rgb = px[:, :, [2, 1, 0]]
elif bytes_per_line >= width * 3:
    px = raw[:, : width * 3].reshape((height, width, 3))
    rgb = px[:, :, [2, 1, 0]]
else:
    raise RuntimeError(f"Unsupported XWD layout: width={width} bytes_per_line={bytes_per_line}")

Image.fromarray(rgb, "RGB").save(dst)
print(f"[OK] wrote {dst}")
PY

rm -f "$TMP_XWD"
