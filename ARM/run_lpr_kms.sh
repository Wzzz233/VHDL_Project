#!/usr/bin/env bash
set -euo pipefail

DEVICE="/dev/fpga_dma0"
DRM_CARD="/dev/dri/card0"
VEH_MODEL=""
PLATE_MODEL=""
LABELS=""
CONNECTOR_ID=""
FPS="15"
PIXEL_ORDER="bgr565"
SWAP16="1"
TIMEOUT_MS="5000"
STATS_INTERVAL="1"
COPY_BUFFERS="2"
QUEUE_DEPTH="1"
MIN_CAR_CONF="0.35"
MIN_PLATE_CONF="0.35"
PLATE_ON_CAR_ONLY="0"

usage() {
  cat <<EOF
Usage: $0 --veh-model <path> --plate-model <path> --labels <path> [options]
  --device <path>            FPGA device (default: ${DEVICE})
  --drm-card <path>          DRM card (default: ${DRM_CARD})
  --veh-model <path>         Vehicle RKNN model (required)
  --plate-model <path>       Plate RKNN model (required)
  --labels <path>            Labels file (required)
  --connector-id <id>        Optional KMS connector id
  --fps <num>                Target FPS (default: ${FPS})
  --pixel-order <mode>       bgr565|rgb565 (default: ${PIXEL_ORDER})
  --swap16 <0|1>             Swap bytes per 16-bit pixel (default: ${SWAP16})
  --timeout-ms <ms>          Timeout ms (default: ${TIMEOUT_MS})
  --stats-interval <sec>     Stats interval (default: ${STATS_INTERVAL})
  --copy-buffers <num>       Copy buffers (default: ${COPY_BUFFERS})
  --queue-depth <num>        appsrc queue depth (default: ${QUEUE_DEPTH})
  --min-car-conf <v>         Car conf threshold (default: ${MIN_CAR_CONF})
  --min-plate-conf <v>       Plate conf threshold (default: ${MIN_PLATE_CONF})
  --plate-on-car-only <0|1>  Reserve switch (default: ${PLATE_ON_CAR_ONLY})
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --device) DEVICE="$2"; shift 2 ;;
    --drm-card) DRM_CARD="$2"; shift 2 ;;
    --veh-model) VEH_MODEL="$2"; shift 2 ;;
    --plate-model) PLATE_MODEL="$2"; shift 2 ;;
    --labels) LABELS="$2"; shift 2 ;;
    --connector-id) CONNECTOR_ID="$2"; shift 2 ;;
    --fps) FPS="$2"; shift 2 ;;
    --pixel-order) PIXEL_ORDER="$2"; shift 2 ;;
    --swap16) SWAP16="$2"; shift 2 ;;
    --timeout-ms) TIMEOUT_MS="$2"; shift 2 ;;
    --stats-interval) STATS_INTERVAL="$2"; shift 2 ;;
    --copy-buffers) COPY_BUFFERS="$2"; shift 2 ;;
    --queue-depth) QUEUE_DEPTH="$2"; shift 2 ;;
    --min-car-conf) MIN_CAR_CONF="$2"; shift 2 ;;
    --min-plate-conf) MIN_PLATE_CONF="$2"; shift 2 ;;
    --plate-on-car-only) PLATE_ON_CAR_ONLY="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
  esac
done

if [[ -z "$VEH_MODEL" || -z "$PLATE_MODEL" || -z "$LABELS" ]]; then
  echo "Missing required args: --veh-model --plate-model --labels" >&2
  usage
  exit 1
fi

if [[ ! -x ./fpga_lpr_display ]]; then
  echo "fpga_lpr_display not found. Build first: make lprapp" >&2
  exit 2
fi

if ! command -v gst-inspect-1.0 >/dev/null 2>&1; then
  echo "gst-inspect-1.0 not found" >&2
  exit 2
fi
for plugin in appsrc queue kmssink; do
  if ! gst-inspect-1.0 "$plugin" >/dev/null 2>&1; then
    echo "Missing plugin: $plugin" >&2
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
if [[ ! -f "$VEH_MODEL" || ! -f "$PLATE_MODEL" || ! -f "$LABELS" ]]; then
  echo "Model or label file not found" >&2
  exit 3
fi

CMD=(./fpga_lpr_display
  --device "$DEVICE"
  --drm-card "$DRM_CARD"
  --veh-model "$VEH_MODEL"
  --plate-model "$PLATE_MODEL"
  --labels "$LABELS"
  --fps "$FPS"
  --pixel-order "$PIXEL_ORDER"
  --swap16 "$SWAP16"
  --timeout-ms "$TIMEOUT_MS"
  --stats-interval "$STATS_INTERVAL"
  --copy-buffers "$COPY_BUFFERS"
  --queue-depth "$QUEUE_DEPTH"
  --min-car-conf "$MIN_CAR_CONF"
  --min-plate-conf "$MIN_PLATE_CONF"
  --plate-on-car-only "$PLATE_ON_CAR_ONLY")

if [[ -n "$CONNECTOR_ID" ]]; then
  CMD+=(--connector-id "$CONNECTOR_ID")
fi

echo "Launching: ${CMD[*]}"
exec "${CMD[@]}"

