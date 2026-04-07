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
SW_PREPROC="0"
FPGA_A_MASK="0"
A_PROJ_RATIO="0.35"
A_ROI_IOU_MIN="0.05"
PED_EVENT="0"
RED_STABLE_FRAMES="5"
RED_RATIO_THR="0.002"
STOPLINE_RATIO="0.55"
DET_RESIZE_MODE="letterbox"
PLATE_REFINE="1"
PLATE_DETECTOR_TYPE="yolov5"
PLATE_NMS_IOU="0.45"
PLATE_MAX_DET="128"
PLATE_CLASS_ID="-1"
OCR_CHANNEL_ORDER="rgb"
OCR_CROP_MODE="fixed"
OCR_RESIZE_MODE="stretch"
OCR_RESIZE_KERNEL="nn"
OCR_PREPROC="none"
SHOW_CROP_BOX="0"
OCR_MIN_PLATE_H="24"
OCR_MIN_SHARPNESS="20"
OCR_MIN_OCC_RATIO="0.90"
OCR_CTC_DIAG="0"
OCR_CROP_DUMP_DIR=""
OCR_CROP_DUMP_MAX="20"
OFFLINE_IMAGE=""
OFFLINE_ROI=""
OFFLINE_DETECT_PLATE="1"

usage() {
  cat <<EOF
Usage: $0 [--offline-image <path>] --plate-model <path> --ocr-model <path> --ocr-keys <path> [options]
  --device <path>            FPGA device (default: ${DEVICE})
  --drm-card <path>          DRM card (default: ${DRM_CARD})
  --veh-model <path>         Vehicle RKNN model (required for live camera mode)
  --plate-model <path>       Plate RKNN model (required)
  --ocr-model <path>         OCR RKNN model (required)
  --ocr-keys <path>          OCR keys txt (required)
  --labels <path>            Labels file (required for live camera mode)
  --pred-log <path>          Prediction CSV output path (optional)
  --offline-image <path>     Run one-shot offline on image (jpg/png/ppm), no camera path
  --offline-roi <x1,y1,x2,y2> Optional plate ROI for offline image
  --offline-detect-plate <0|1> Auto plate detect in offline mode (default: ${OFFLINE_DETECT_PLATE})
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
  --sw-preproc <0|1>         Enable software preproc A/B path (default: ${SW_PREPROC})
  --fpga-a-mask <0|1>        Enable FPGA A-channel ROI fusion (default: ${FPGA_A_MASK})
  --a-proj-ratio <v>         A-channel projection threshold ratio (default: ${A_PROJ_RATIO})
  --a-roi-iou-min <v>        Min IoU for A-ROI filtering (default: ${A_ROI_IOU_MIN})
  --ped-event <0|1>          Enable pedestrian red-light event (default: ${PED_EVENT})
  --red-stable-frames <n>    Red light debounce frames (default: ${RED_STABLE_FRAMES})
  --red-ratio-thr <v>        A-channel red ratio threshold (default: ${RED_RATIO_THR})
  --stopline-ratio <v>       Stopline Y ratio [0,1] (default: ${STOPLINE_RATIO})
  --det-resize-mode <m>      Detect resize: stretch|letterbox (default: ${DET_RESIZE_MODE})
  --plate-refine <0|1>       Enable local high-res plate refine (default: ${PLATE_REFINE})
  --plate-detector-type <m>  Plate detector: yolov5|yolov8_obb_rknn (default: ${PLATE_DETECTOR_TYPE})
  --plate-nms-iou <v>        Plate NMS IoU threshold (default: ${PLATE_NMS_IOU})
  --plate-max-det <n>        Plate max detections after NMS (default: ${PLATE_MAX_DET})
  --plate-class-id <n>       Optional class filter for plate model (-1 disables, default: ${PLATE_CLASS_ID})
  --ocr-channel-order <m>    OCR input order: rgb|bgr (default: ${OCR_CHANNEL_ORDER})
  --ocr-crop-mode <m>        OCR crop mode: fixed|box|tight|box-pad|match|obb_warp (default: ${OCR_CROP_MODE})
  --ocr-resize-mode <m>      OCR resize mode: stretch|letterbox (default: ${OCR_RESIZE_MODE})
  --ocr-resize-kernel <m>    OCR resize kernel: nn|bilinear (default: ${OCR_RESIZE_KERNEL})
  --ocr-preproc <m>          OCR crop preproc: none|gray|bin (default: ${OCR_PREPROC})
  --show-crop-box <0|1>      Overlay OCR crop box in red (default: ${SHOW_CROP_BOX})
  --ocr-min-plate-h <n>      Skip OCR if plate box h < n (default: ${OCR_MIN_PLATE_H})
  --ocr-min-sharpness <v>    Skip OCR if Laplacian var < v (default: ${OCR_MIN_SHARPNESS})
  --ocr-min-occ-ratio <v>    Re-crop once if OCR occupancy < v (default: ${OCR_MIN_OCC_RATIO})
  --ocr-ctc-diag <0|1>       Print CTC decode diagnostics (default: ${OCR_CTC_DIAG})
  --ocr-crop-dump-dir <p>    Dump OCR crops+inputs to directory (default: off)
  --ocr-crop-dump-max <n>    Max dumped OCR samples (default: ${OCR_CROP_DUMP_MAX})
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
    --offline-image) OFFLINE_IMAGE="$2"; shift 2 ;;
    --offline-roi) OFFLINE_ROI="$2"; shift 2 ;;
    --offline-detect-plate) OFFLINE_DETECT_PLATE="$2"; shift 2 ;;
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
    --sw-preproc) SW_PREPROC="$2"; shift 2 ;;
    --fpga-a-mask) FPGA_A_MASK="$2"; shift 2 ;;
    --a-proj-ratio) A_PROJ_RATIO="$2"; shift 2 ;;
    --a-roi-iou-min) A_ROI_IOU_MIN="$2"; shift 2 ;;
    --ped-event) PED_EVENT="$2"; shift 2 ;;
    --red-stable-frames) RED_STABLE_FRAMES="$2"; shift 2 ;;
    --red-ratio-thr) RED_RATIO_THR="$2"; shift 2 ;;
    --stopline-ratio) STOPLINE_RATIO="$2"; shift 2 ;;
    --det-resize-mode) DET_RESIZE_MODE="$2"; shift 2 ;;
    --plate-refine) PLATE_REFINE="$2"; shift 2 ;;
    --plate-detector-type) PLATE_DETECTOR_TYPE="$2"; shift 2 ;;
    --plate-nms-iou) PLATE_NMS_IOU="$2"; shift 2 ;;
    --plate-max-det) PLATE_MAX_DET="$2"; shift 2 ;;
    --plate-class-id) PLATE_CLASS_ID="$2"; shift 2 ;;
    --ocr-channel-order) OCR_CHANNEL_ORDER="$2"; shift 2 ;;
    --ocr-crop-mode) OCR_CROP_MODE="$2"; shift 2 ;;
    --ocr-resize-mode) OCR_RESIZE_MODE="$2"; shift 2 ;;
    --ocr-resize-kernel) OCR_RESIZE_KERNEL="$2"; shift 2 ;;
    --ocr-preproc) OCR_PREPROC="$2"; shift 2 ;;
    --show-crop-box) SHOW_CROP_BOX="$2"; shift 2 ;;
    --ocr-min-plate-h) OCR_MIN_PLATE_H="$2"; shift 2 ;;
    --ocr-min-sharpness) OCR_MIN_SHARPNESS="$2"; shift 2 ;;
    --ocr-min-occ-ratio) OCR_MIN_OCC_RATIO="$2"; shift 2 ;;
    --ocr-ctc-diag) OCR_CTC_DIAG="$2"; shift 2 ;;
    --ocr-crop-dump-dir) OCR_CROP_DUMP_DIR="$2"; shift 2 ;;
    --ocr-crop-dump-max) OCR_CROP_DUMP_MAX="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
  esac
done

OFFLINE_MODE=0
if [[ -n "$OFFLINE_IMAGE" ]]; then
  OFFLINE_MODE=1
fi

if [[ "$OFFLINE_MODE" == "1" ]]; then
  if [[ -z "$PLATE_MODEL" || -z "$OCR_MODEL" || -z "$OCR_KEYS" ]]; then
    echo "Offline mode requires: --plate-model --ocr-model --ocr-keys" >&2
    usage
    exit 1
  fi
else
  if [[ -z "$VEH_MODEL" || -z "$PLATE_MODEL" || -z "$OCR_MODEL" || -z "$OCR_KEYS" || -z "$LABELS" ]]; then
    echo "Missing required args: --veh-model --plate-model --ocr-model --ocr-keys --labels" >&2
    usage
    exit 1
  fi
fi

if [[ ! -x ./fpga_lpr_display ]]; then
  echo "fpga_lpr_display not found. Build first: make lprapp" >&2
  exit 2
fi

if [[ "$OFFLINE_MODE" == "0" ]]; then
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
fi

if [[ "$OFFLINE_MODE" == "1" ]]; then
  if [[ ! -f "$OFFLINE_IMAGE" || ! -f "$PLATE_MODEL" || ! -f "$OCR_MODEL" || ! -f "$OCR_KEYS" ]]; then
    echo "Offline image/model/keys file not found" >&2
    exit 3
  fi
  if [[ "$OFFLINE_IMAGE" =~ \.[Pp][Pp][Mm]$ ]]; then
    OFFLINE_INPUT="$OFFLINE_IMAGE"
  else
    if ! command -v ffmpeg >/dev/null 2>&1; then
      echo "ffmpeg is required to convert offline image to PPM" >&2
      exit 3
    fi
    OFFLINE_INPUT="/tmp/lpr_offline_input_$$.ppm"
    ffmpeg -loglevel error -y -i "$OFFLINE_IMAGE" -frames:v 1 "$OFFLINE_INPUT"
  fi
else
  if [[ ! -f "$VEH_MODEL" || ! -f "$PLATE_MODEL" || ! -f "$OCR_MODEL" || ! -f "$OCR_KEYS" || ! -f "$LABELS" ]]; then
    echo "Model/keys/label file not found" >&2
    exit 3
  fi
fi

CMD=(./fpga_lpr_display
  --plate-model "$PLATE_MODEL"
  --ocr-model "$OCR_MODEL"
  --ocr-keys "$OCR_KEYS"
  --min-plate-conf "$MIN_PLATE_CONF"
  --plate-detector-type "$PLATE_DETECTOR_TYPE"
  --plate-nms-iou "$PLATE_NMS_IOU"
  --plate-max-det "$PLATE_MAX_DET"
  --plate-class-id "$PLATE_CLASS_ID"
  --ocr-channel-order "$OCR_CHANNEL_ORDER"
  --ocr-crop-mode "$OCR_CROP_MODE"
  --ocr-resize-mode "$OCR_RESIZE_MODE"
  --ocr-resize-kernel "$OCR_RESIZE_KERNEL"
  --ocr-preproc "$OCR_PREPROC"
  --show-crop-box "$SHOW_CROP_BOX"
  --ocr-min-plate-h "$OCR_MIN_PLATE_H"
  --ocr-min-sharpness "$OCR_MIN_SHARPNESS"
  --ocr-min-occ-ratio "$OCR_MIN_OCC_RATIO"
  --ocr-ctc-diag "$OCR_CTC_DIAG"
  --ocr-crop-dump-max "$OCR_CROP_DUMP_MAX")

if [[ "$OFFLINE_MODE" == "0" ]]; then
  CMD+=(
    --device "$DEVICE"
    --drm-card "$DRM_CARD"
    --veh-model "$VEH_MODEL"
    --labels "$LABELS"
    --fps "$FPS"
    --pixel-order "$PIXEL_ORDER"
    --swap16 "$SWAP16"
    --timeout-ms "$TIMEOUT_MS"
    --stats-interval "$STATS_INTERVAL"
    --copy-buffers "$COPY_BUFFERS"
    --queue-depth "$QUEUE_DEPTH"
    --min-car-conf "$MIN_CAR_CONF"
    --plate-on-car-only "$PLATE_ON_CAR_ONLY"
    --plate-only "$PLATE_ONLY"
    --sw-preproc "$SW_PREPROC"
    --fpga-a-mask "$FPGA_A_MASK"
    --a-proj-ratio "$A_PROJ_RATIO"
    --a-roi-iou-min "$A_ROI_IOU_MIN"
    --ped-event "$PED_EVENT"
    --red-stable-frames "$RED_STABLE_FRAMES"
    --red-ratio-thr "$RED_RATIO_THR"
    --stopline-ratio "$STOPLINE_RATIO"
    --det-resize-mode "$DET_RESIZE_MODE"
    --plate-refine "$PLATE_REFINE")
else
  CMD+=(
    --offline-image "$OFFLINE_INPUT"
    --offline-detect-plate "$OFFLINE_DETECT_PLATE"
    --sw-preproc "$SW_PREPROC"
    --det-resize-mode "$DET_RESIZE_MODE"
    --plate-refine "$PLATE_REFINE")
  if [[ -n "$OFFLINE_ROI" ]]; then
    CMD+=(--offline-roi "$OFFLINE_ROI")
  fi
fi

if [[ -n "$OCR_CROP_DUMP_DIR" ]]; then
  CMD+=(--ocr-crop-dump-dir "$OCR_CROP_DUMP_DIR")
fi

if [[ -n "$CONNECTOR_ID" ]]; then
  CMD+=(--connector-id "$CONNECTOR_ID")
fi
if [[ -n "$PRED_LOG" ]]; then
  CMD+=(--pred-log "$PRED_LOG")
fi

echo "Launching: ${CMD[*]}"
exec "${CMD[@]}"
