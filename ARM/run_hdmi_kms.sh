#!/usr/bin/env bash
set -euo pipefail

DEVICE="/dev/fpga_dma0"
DRM_CARD="/dev/dri/card0"
INPUT_DEV=""
FPS="10"
PIXEL_ORDER="bgr565"
CONNECTOR_ID=""
TIMEOUT_MS="5000"
STATS_INTERVAL="1"
COPY_BUFFERS="3"
QUEUE_DEPTH="2"
IO_MODE="mmap"

usage() {
  cat <<EOF
Usage: $0 [options]
  --device <path>         FPGA device (default: ${DEVICE})
  --drm-card <path>       DRM card (default: ${DRM_CARD})
  --connector-id <id>     Optional KMS connector id
  --input <event>         Optional /dev/input/eventX
  --fps <num>             Target FPS (default: ${FPS})
  --pixel-order <mode>    bgr565|rgb565 (default: ${PIXEL_ORDER})
  --timeout-ms <ms>       Frame timeout (default: ${TIMEOUT_MS})
  --stats-interval <sec>  Stats interval (default: ${STATS_INTERVAL})
  --copy-buffers <num>    Copy ring size (default: ${COPY_BUFFERS})
  --queue-depth <num>     appsrc queue depth (default: ${QUEUE_DEPTH})
  --io-mode <mode>        mmap|copy (default: ${IO_MODE})
  -h, --help              Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --device) DEVICE="$2"; shift 2 ;;
    --drm-card) DRM_CARD="$2"; shift 2 ;;
    --connector-id) CONNECTOR_ID="$2"; shift 2 ;;
    --input) INPUT_DEV="$2"; shift 2 ;;
    --fps) FPS="$2"; shift 2 ;;
    --pixel-order) PIXEL_ORDER="$2"; shift 2 ;;
    --timeout-ms) TIMEOUT_MS="$2"; shift 2 ;;
    --stats-interval) STATS_INTERVAL="$2"; shift 2 ;;
    --copy-buffers) COPY_BUFFERS="$2"; shift 2 ;;
    --queue-depth) QUEUE_DEPTH="$2"; shift 2 ;;
    --io-mode) IO_MODE="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if ! command -v gst-inspect-1.0 >/dev/null 2>&1; then
  echo "gst-inspect-1.0 not found" >&2
  exit 2
fi

for plugin in appsrc videoconvert kmssink; do
  if ! gst-inspect-1.0 "$plugin" >/dev/null 2>&1; then
    echo "Missing GStreamer plugin: $plugin" >&2
    exit 2
  fi
done

if [[ ! -c "$DEVICE" ]]; then
  echo "Device not found: $DEVICE" >&2
  exit 3
fi

if [[ ! -e "$DRM_CARD" ]]; then
  echo "DRM card not found: $DRM_CARD" >&2
  exit 3
fi

if [[ ! -x ./fpga_hdmi_display ]]; then
  echo "fpga_hdmi_display not found. Build it first:" >&2
  echo "  make displayapp" >&2
  exit 4
fi

CMD=(./fpga_hdmi_display
  --device "$DEVICE"
  --drm-card "$DRM_CARD"
  --fps "$FPS"
  --pixel-order "$PIXEL_ORDER"
  --timeout-ms "$TIMEOUT_MS"
  --stats-interval "$STATS_INTERVAL"
  --copy-buffers "$COPY_BUFFERS"
  --queue-depth "$QUEUE_DEPTH"
  --io-mode "$IO_MODE")

if [[ -n "$CONNECTOR_ID" ]]; then
  CMD+=(--connector-id "$CONNECTOR_ID")
fi

if [[ -n "$INPUT_DEV" ]]; then
  CMD+=(--input "$INPUT_DEV")
fi

echo "Launching: ${CMD[*]}"
exec "${CMD[@]}"
