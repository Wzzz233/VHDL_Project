#!/usr/bin/env bash
set -euo pipefail

DEVICE="/dev/fpga_dma0"
DRM_CARD="/dev/dri/card0"
PED_MODEL=""
LABELS=""
CONNECTOR_ID=""
FPS="15"
PIXEL_ORDER="bgr565"
SWAP16="1"
TIMEOUT_MS="5000"
STATS_INTERVAL="1"
COPY_BUFFERS="3"
QUEUE_DEPTH="1"
MIN_PERSON_CONF="0.35"
CV_EVERY_N="3"
SCENE_SMOOTH="10"

usage() {
  cat <<EOF
Usage: $0 --ped-model <path> --labels <path> [options]
  --device <path>            FPGA device (default: ${DEVICE})
  --drm-card <path>          DRM card (default: ${DRM_CARD})
  --ped-model <path>         Pedestrian YOLO RKNN model (required)
  --labels <path>            Labels file with person class (required)
  --connector-id <id>        Optional KMS connector id
  --fps <num>                Target FPS (default: ${FPS})
  --pixel-order <mode>       bgr565|rgb565 (default: ${PIXEL_ORDER})
  --swap16 <0|1>             Swap bytes per 16-bit pixel (default: ${SWAP16})
  --timeout-ms <ms>          Slot wait timeout (default: ${TIMEOUT_MS})
  --stats-interval <sec>     Stats interval (default: ${STATS_INTERVAL})
  --copy-buffers <num>       Copy/display buffers (default: ${COPY_BUFFERS})
  --queue-depth <num>        appsrc queue depth (default: ${QUEUE_DEPTH})
  --min-person-conf <v>      Person confidence threshold (default: ${MIN_PERSON_CONF})
  --cv-every-n <n>           Run scene CV every N frames (default: ${CV_EVERY_N})
  --scene-smooth <n>         Stable ROI hold TTL in CV samples (default: ${SCENE_SMOOTH})
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --device) DEVICE="$2"; shift 2 ;;
    --drm-card) DRM_CARD="$2"; shift 2 ;;
    --ped-model) PED_MODEL="$2"; shift 2 ;;
    --labels) LABELS="$2"; shift 2 ;;
    --connector-id) CONNECTOR_ID="$2"; shift 2 ;;
    --fps) FPS="$2"; shift 2 ;;
    --pixel-order) PIXEL_ORDER="$2"; shift 2 ;;
    --swap16) SWAP16="$2"; shift 2 ;;
    --timeout-ms) TIMEOUT_MS="$2"; shift 2 ;;
    --stats-interval) STATS_INTERVAL="$2"; shift 2 ;;
    --copy-buffers) COPY_BUFFERS="$2"; shift 2 ;;
    --queue-depth) QUEUE_DEPTH="$2"; shift 2 ;;
    --min-person-conf) MIN_PERSON_CONF="$2"; shift 2 ;;
    --cv-every-n) CV_EVERY_N="$2"; shift 2 ;;
    --scene-smooth) SCENE_SMOOTH="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
  esac
done

if [[ -z "$PED_MODEL" || -z "$LABELS" ]]; then
  echo "Missing required args: --ped-model --labels" >&2
  usage
  exit 1
fi

if [[ ! -x ./crosswalk_ped_cv_test ]]; then
  echo "crosswalk_ped_cv_test not found. Build first: make crosswalk-test" >&2
  exit 2
fi

if ! pkg-config --exists opencv4; then
  echo "opencv4 pkg-config metadata not found" >&2
  exit 2
fi

if ! command -v gst-inspect-1.0 >/dev/null 2>&1; then
  echo "gst-inspect-1.0 not found" >&2
  exit 2
fi

for plugin in appsrc queue kmssink; do
  if ! gst-inspect-1.0 "$plugin" >/dev/null 2>&1; then
    echo "Missing GStreamer plugin: $plugin" >&2
    exit 2
  fi
done

if ! ldconfig -p 2>/dev/null | grep -q "librknnrt"; then
  echo "Warning: librknnrt not found in ldconfig cache; continuing in case LD_LIBRARY_PATH is set" >&2
fi

if [[ ! -c "$DEVICE" ]]; then
  echo "Device not found: $DEVICE" >&2
  exit 3
fi
if [[ ! -e "$DRM_CARD" ]]; then
  echo "DRM card not found: $DRM_CARD" >&2
  exit 3
fi
if [[ ! -f "$PED_MODEL" || ! -f "$LABELS" ]]; then
  echo "Pedestrian model or labels file not found" >&2
  exit 3
fi

CMD=(./crosswalk_ped_cv_test
  --device "$DEVICE"
  --drm-card "$DRM_CARD"
  --ped-model "$PED_MODEL"
  --labels "$LABELS"
  --fps "$FPS"
  --pixel-order "$PIXEL_ORDER"
  --swap16 "$SWAP16"
  --timeout-ms "$TIMEOUT_MS"
  --stats-interval "$STATS_INTERVAL"
  --copy-buffers "$COPY_BUFFERS"
  --queue-depth "$QUEUE_DEPTH"
  --min-person-conf "$MIN_PERSON_CONF"
  --cv-every-n "$CV_EVERY_N"
  --scene-smooth "$SCENE_SMOOTH")

if [[ -n "$CONNECTOR_ID" ]]; then
  CMD+=(--connector-id "$CONNECTOR_ID")
fi

echo "Launching: ${CMD[*]}"
exec "${CMD[@]}"
