#!/usr/bin/env bash
set -euo pipefail

if ! command -v gst-inspect-1.0 >/dev/null 2>&1; then
    echo "missing gst-inspect-1.0" >&2
    exit 1
fi

required_elements=(
    rtspsrc
    rtph264depay
    h264parse
    appsink
    jpegenc
    videoconvert
    videoscale
)

failed=0
for element in "${required_elements[@]}"; do
    if gst-inspect-1.0 "${element}" >/dev/null 2>&1; then
        echo "[PASS] GStreamer element ${element}"
    else
        echo "[FAIL] GStreamer element ${element}" >&2
        failed=1
    fi
done

if gst-inspect-1.0 mppvideodec >/dev/null 2>&1; then
    echo "[PASS] Rockchip MPP decoder mppvideodec"
else
    echo "[FAIL] Rockchip MPP decoder mppvideodec" >&2
    failed=1
fi

exit "${failed}"
