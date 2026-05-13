#!/usr/bin/env bash
set -euo pipefail

# Launcher for the bird-detection driver (fpga_bird_display).
# Mirrors the style/argument handling of run_lpr_kms.sh, trimmed to the
# arguments actually consumed by fpga_bird_display.

DEVICE="/dev/fpga_dma0"
DRM_CARD="/dev/dri/card0"
BIRD_MODEL="coco_yolov8n_640_bird_det.rknn"
CONNECTOR_ID=""
FPS="15"
PIXEL_ORDER="bgr565"
SWAP16="1"
TIMEOUT_MS="5000"
STATS_INTERVAL="1"
COPY_BUFFERS="2"
QUEUE_DEPTH="1"
MIN_CONF="0.30"
NMS_IOU="0.45"
MAX_DET="128"
CLASS_ID="16"   # COCO bird

usage() {
  cat <<EOF
Usage: $0 [--bird-model <path>] [options]

  Defaults: bird-model=${BIRD_MODEL} class-id=${CLASS_ID} (bird)

  --device <path>          FPGA device (default: ${DEVICE})
  --drm-card <path>        DRM card (default: ${DRM_CARD})
  --bird-model <path>      Bird (COCO YOLOv8n) RKNN model path (default: ${BIRD_MODEL})
  --connector-id <id>      Optional KMS connector id
  --fps <num>              Target FPS (default: ${FPS})
  --pixel-order <mode>     bgr565|rgb565 (default: ${PIXEL_ORDER})
  --swap16 <0|1>           Swap bytes per 16-bit pixel (default: ${SWAP16})
  --timeout-ms <ms>        Timeout ms (default: ${TIMEOUT_MS})
  --stats-interval <sec>   Stats interval (default: ${STATS_INTERVAL})
  --copy-buffers <num>     Copy buffers (default: ${COPY_BUFFERS})
  --queue-depth <num>      appsrc queue depth (default: ${QUEUE_DEPTH})
  --min-conf <v>           Detection confidence threshold (default: ${MIN_CONF})
  --nms-iou <v>            NMS IoU threshold (default: ${NMS_IOU})
  --max-det <n>            Max detections after NMS (default: ${MAX_DET})
  --class-id <n>           COCO class id to keep (-1=any, default: ${CLASS_ID}=bird)
  -h|--help                Show this help

Notes:
  * Model preprocessing is letterbox to 640x640 with pad_value=114 (handled in-app).
  * Output tensor is 1x84x8400 (COCO 80 classes); app decodes + NMS + class filter.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --device)         DEVICE="$2"; shift 2 ;;
    --drm-card)       DRM_CARD="$2"; shift 2 ;;
    --bird-model)     BIRD_MODEL="$2"; shift 2 ;;
    --connector-id)   CONNECTOR_ID="$2"; shift 2 ;;
    --fps)            FPS="$2"; shift 2 ;;
    --pixel-order)    PIXEL_ORDER="$2"; shift 2 ;;
    --swap16)         SWAP16="$2"; shift 2 ;;
    --timeout-ms)     TIMEOUT_MS="$2"; shift 2 ;;
    --stats-interval) STATS_INTERVAL="$2"; shift 2 ;;
    --copy-buffers)   COPY_BUFFERS="$2"; shift 2 ;;
    --queue-depth)    QUEUE_DEPTH="$2"; shift 2 ;;
    --min-conf)       MIN_CONF="$2"; shift 2 ;;
    --nms-iou)        NMS_IOU="$2"; shift 2 ;;
    --max-det)        MAX_DET="$2"; shift 2 ;;
    --class-id)       CLASS_ID="$2"; shift 2 ;;
    -h|--help)        usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
  esac
done

if [[ -z "$BIRD_MODEL" ]]; then
  echo "Missing required --bird-model" >&2
  usage
  exit 1
fi

if [[ ! -x ./fpga_bird_display ]]; then
  echo "fpga_bird_display not found. Build first: make birdapp" >&2
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
if [[ ! -f "$BIRD_MODEL" ]]; then
  echo "Bird model file not found: $BIRD_MODEL" >&2
  exit 3
fi

CMD=(./fpga_bird_display
  --device "$DEVICE"
  --drm-card "$DRM_CARD"
  --bird-model "$BIRD_MODEL"
  --fps "$FPS"
  --pixel-order "$PIXEL_ORDER"
  --swap16 "$SWAP16"
  --timeout-ms "$TIMEOUT_MS"
  --stats-interval "$STATS_INTERVAL"
  --copy-buffers "$COPY_BUFFERS"
  --queue-depth "$QUEUE_DEPTH"
  --min-conf "$MIN_CONF"
  --nms-iou "$NMS_IOU"
  --max-det "$MAX_DET"
  --class-id "$CLASS_ID")

if [[ -n "$CONNECTOR_ID" ]]; then
  CMD+=(--connector-id "$CONNECTOR_ID")
fi

echo "Launching: ${CMD[*]}"
exec "${CMD[@]}"
