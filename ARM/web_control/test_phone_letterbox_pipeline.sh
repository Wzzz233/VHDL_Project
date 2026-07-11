#!/bin/sh
# Verify GStreamer's phone-frame normalization by inspecting one raw BGRx frame.
#
# Default: 640x480 -> 1280x720, with 160-pixel side borders.
# Top-border case: LETTERBOX_CASE=top uses 800x400 -> 1280x720, with
# 40-pixel top and bottom borders.
#
# GST_LAUNCH_CMD may contain a whitespace-separated launcher plus arguments.
# Example using the RK3568 SDK through qemu-aarch64:
#
#   SDK_ROOT=/home/wzzz/RK3568J_SDK/debian/binary
#   export QEMU_LD_PREFIX="$SDK_ROOT"
#   export GST_PLUGIN_SYSTEM_PATH_1_0="$SDK_ROOT/usr/lib/aarch64-linux-gnu/gstreamer-1.0"
#   export GST_PLUGIN_SCANNER_1_0="$SDK_ROOT/usr/lib/aarch64-linux-gnu/gstreamer1.0/gstreamer-1.0/gst-plugin-scanner"
#   export GST_REGISTRY_1_0=/tmp/gst-registry-phone-letterbox.bin
#   GST_LAUNCH_CMD="/usr/bin/qemu-aarch64 -L $SDK_ROOT $SDK_ROOT/usr/bin/gst-launch-1.0" \
#     PYTHON=/root/miniconda3/bin/python \
#     ./test_phone_letterbox_pipeline.sh

set -eu

GST_LAUNCH_CMD=${GST_LAUNCH_CMD:-gst-launch-1.0}
PYTHON=${PYTHON:-python3}
LETTERBOX_CASE=${LETTERBOX_CASE:-side}

case "$LETTERBOX_CASE" in
    side)
        SRC_WIDTH=640
        SRC_HEIGHT=480
        ;;
    top)
        SRC_WIDTH=800
        SRC_HEIGHT=400
        ;;
    *)
        echo "LETTERBOX_CASE must be 'side' or 'top'" >&2
        exit 2
        ;;
esac

CLEAN_OUTPUT=0
if [ -z "${OUTPUT+x}" ]; then
    OUTPUT=$(mktemp "${TMPDIR:-/tmp}/phone_letterbox_XXXXXX.bgrx")
    CLEAN_OUTPUT=1
fi

cleanup()
{
    if [ "$CLEAN_OUTPUT" -eq 1 ]; then
        rm -f "$OUTPUT"
    fi
}
trap cleanup EXIT HUP INT TERM

run_gst_launch()
{
    # Intentional splitting lets the override include qemu and its arguments.
    # Paths in GST_LAUNCH_CMD therefore must not contain whitespace.
    # shellcheck disable=SC2086
    set -- $GST_LAUNCH_CMD "$@"
    "$@"
}

run_gst_launch -q \
    videotestsrc num-buffers=1 pattern=white ! \
    "video/x-raw,width=${SRC_WIDTH},height=${SRC_HEIGHT},framerate=1/1,pixel-aspect-ratio=1/1" ! \
    videoconvert ! \
    videoscale add-borders=true ! \
    video/x-raw,format=BGRx,width=1280,height=720,pixel-aspect-ratio=1/1 ! \
    filesink "location=${OUTPUT}" sync=false

"$PYTHON" - "$OUTPUT" "$LETTERBOX_CASE" <<'PY'
from pathlib import Path
import sys

WIDTH = 1280
HEIGHT = 720
STRIDE = WIDTH * 4
EXPECTED_SIZE = STRIDE * HEIGHT

path = Path(sys.argv[1])
case = sys.argv[2]
frame = path.read_bytes()
if len(frame) != EXPECTED_SIZE:
    raise SystemExit(
        f"[FAIL] {path}: expected {EXPECTED_SIZE} bytes, got {len(frame)}"
    )


def bgr(x, y):
    offset = y * STRIDE + x * 4
    return tuple(frame[offset:offset + 3])


def require_black(label, points):
    bad = [(point, bgr(*point)) for point in points if max(bgr(*point)) > 4]
    if bad:
        raise SystemExit(f"[FAIL] {label} border is not black: {bad}")


def require_white(label, points):
    bad = [(point, bgr(*point)) for point in points if min(bgr(*point)) < 240]
    if bad:
        raise SystemExit(f"[FAIL] {label} image is not white: {bad}")


if case == "side":
    border = [
        (0, 0), (159, 0), (0, 360), (159, 719),
        (1120, 0), (1279, 360), (1120, 719), (1279, 719),
    ]
    center = [(160, 0), (640, 0), (640, 360), (1119, 719)]
    require_black("side", border)
    require_white("center", center)
    padding = "160-pixel left/right"
else:
    border = [
        (0, 0), (640, 0), (1279, 39),
        (0, 680), (640, 719), (1279, 719),
    ]
    center = [(0, 40), (640, 40), (640, 360), (1279, 679)]
    require_black("top/bottom", border)
    require_white("center", center)
    padding = "40-pixel top/bottom"

print(
    f"[PASS] {path}: {WIDTH}x{HEIGHT} BGRx, {padding} padding is black, "
    "active image is white"
)
PY
