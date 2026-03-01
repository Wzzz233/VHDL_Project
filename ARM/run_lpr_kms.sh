#!/usr/bin/env bash
set -euo pipefail

DEVICE="/dev/fpga_dma0"
DRM_CARD="/dev/dri/card0"
VEH_MODEL=""
PLATE_MODEL=""
OCR_MODEL=""
OCR_KEYS=""
LABELS=""
PRED_LOG=""
CONNECTOR_ID=""
FPS="15"
PIXEL_ORDER="bgr565"
SWAP16="1"
TIMEOUT_MS="5000"
STATS_INTERVAL="1"
COPY_BUFFERS="2"
QUEUE_DEPTH="1"
MIN_CAR_CONF="0.35"
MIN_PLATE_CONF="0.45"
PLATE_ON_CAR_ONLY="0"
PLATE_ONLY="1"

usage() {
  cat <<EOF
Usage: $0 --veh-model <path> --plate-model <path> --ocr-model <path> --ocr-keys <path> --labels <path> [options]
  --device <path>            FPGA device (default: ${DEVICE})
  --drm-card <path>          DRM card (default: ${DRM_CARD})
  --veh-model <path>         Vehicle RKNN model (required)
  --plate-model <path>       Plate RKNN model (required)
  --ocr-model <path>         OCR RKNN model (required)
  --ocr-keys <path>          OCR keys txt (required)
  --labels <path>            Labels file (required)
  --pred-log <path>          Prediction CSV output path (optional)
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
  --plate-only <0|1>         Disable vehicle dependency for plate output (default: ${PLATE_ONLY})
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --device) DEVICE="$2"; shift 2 ;;
    --drm-card) DRM_CARD="$2"; shift 2 ;;
    --veh-model) VEH_MODEL="$2"; shift 2 ;;
    --plate-model) PLATE_MODEL="$2"; shift 2 ;;
    --ocr-model) OCR_MODEL="$2"; shift 2 ;;
    --ocr-keys) OCR_KEYS="$2"; shift 2 ;;
    --labels) LABELS="$2"; shift 2 ;;
    --pred-log) PRED_LOG="$2"; shift 2 ;;
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
    --plate-only) PLATE_ONLY="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
  esac
done

if [[ -z "$VEH_MODEL" || -z "$PLATE_MODEL" || -z "$OCR_MODEL" || -z "$OCR_KEYS" || -z "$LABELS" ]]; then
  echo "Missing required args: --veh-model --plate-model --ocr-model --ocr-keys --labels" >&2
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
if [[ ! -f "$VEH_MODEL" || ! -f "$PLATE_MODEL" || ! -f "$OCR_MODEL" || ! -f "$OCR_KEYS" || ! -f "$LABELS" ]]; then
  echo "Model/keys/label file not found" >&2
  exit 3
fi

CMD=(./fpga_lpr_display
  --device "$DEVICE"
  --drm-card "$DRM_CARD"
  --veh-model "$VEH_MODEL"
  --plate-model "$PLATE_MODEL"
  --ocr-model "$OCR_MODEL"
  --ocr-keys "$OCR_KEYS"
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
  --plate-on-car-only "$PLATE_ON_CAR_ONLY"
  --plate-only "$PLATE_ONLY")

if [[ -n "$CONNECTOR_ID" ]]; then
  CMD+=(--connector-id "$CONNECTOR_ID")
fi
if [[ -n "$PRED_LOG" ]]; then
  CMD+=(--pred-log "$PRED_LOG")
fi

echo "Launching: ${CMD[*]}"
exec "${CMD[@]}"
