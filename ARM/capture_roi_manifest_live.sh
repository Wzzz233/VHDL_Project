#!/usr/bin/env bash
set -euo pipefail

DEVICE="/dev/fpga_dma0"
DRM_CARD="/dev/dri/card0"
VEH_MODEL=""
PLATE_MODEL=""
OCR_MODEL=""
OCR_KEYS=""
LABELS=""
OUT_DIR="/tmp/roi_capture"
DURATION_SEC="60"
FPS="15"
COPY_BUFFERS="2"
QUEUE_DEPTH="1"
MIN_CAR_CONF="0.35"
MIN_PLATE_CONF="0.45"
SCENARIO="capture_live"
SHOW_CROP_BOX="1"
MAX_FRAME_DUMPS="200"
MAX_CROP_DUMPS="400"
PIXEL_ORDER="bgr565"
SWAP16="1"
TIMEOUT_MS="5000"
STATS_INTERVAL="1"
PLATE_ONLY="1"
SW_PREPROC="0"
FPGA_A_MASK="0"
DET_RESIZE_MODE="letterbox"
PLATE_REFINE="1"
OCR_CHANNEL_ORDER="bgr"
OCR_CROP_MODE="match"
OCR_RESIZE_MODE="letterbox"
OCR_RESIZE_KERNEL="nn"
OCR_PREPROC="none"
OCR_MIN_PLATE_H="24"
OCR_MIN_SHARPNESS="20"
OCR_MIN_OCC_RATIO="0.90"

usage() {
  cat <<EOF
Usage: $0 --veh-model <path> --plate-model <path> --ocr-model <path> --ocr-keys <path> --labels <path> [options]
  --out-dir <path>         capture output directory (default: ${OUT_DIR})
  --duration-sec <n>       capture duration in seconds (default: ${DURATION_SEC})
  --scenario <name>        scenario label written into roi_manifest_candidates.csv (default: ${SCENARIO})
  --max-frame-dumps <n>    max dumped full frames (default: ${MAX_FRAME_DUMPS})
  --max-crop-dumps <n>     max dumped OCR crop pairs (default: ${MAX_CROP_DUMPS})
  --fps <n>                target fps (default: ${FPS})
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
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --duration-sec) DURATION_SEC="$2"; shift 2 ;;
    --scenario) SCENARIO="$2"; shift 2 ;;
    --max-frame-dumps) MAX_FRAME_DUMPS="$2"; shift 2 ;;
    --max-crop-dumps) MAX_CROP_DUMPS="$2"; shift 2 ;;
    --fps) FPS="$2"; shift 2 ;;
    --copy-buffers) COPY_BUFFERS="$2"; shift 2 ;;
    --queue-depth) QUEUE_DEPTH="$2"; shift 2 ;;
    --min-car-conf) MIN_CAR_CONF="$2"; shift 2 ;;
    --min-plate-conf) MIN_PLATE_CONF="$2"; shift 2 ;;
    --pixel-order) PIXEL_ORDER="$2"; shift 2 ;;
    --swap16) SWAP16="$2"; shift 2 ;;
    --timeout-ms) TIMEOUT_MS="$2"; shift 2 ;;
    --stats-interval) STATS_INTERVAL="$2"; shift 2 ;;
    --plate-only) PLATE_ONLY="$2"; shift 2 ;;
    --sw-preproc) SW_PREPROC="$2"; shift 2 ;;
    --fpga-a-mask) FPGA_A_MASK="$2"; shift 2 ;;
    --det-resize-mode) DET_RESIZE_MODE="$2"; shift 2 ;;
    --plate-refine) PLATE_REFINE="$2"; shift 2 ;;
    --ocr-channel-order) OCR_CHANNEL_ORDER="$2"; shift 2 ;;
    --ocr-crop-mode) OCR_CROP_MODE="$2"; shift 2 ;;
    --ocr-resize-mode) OCR_RESIZE_MODE="$2"; shift 2 ;;
    --ocr-resize-kernel) OCR_RESIZE_KERNEL="$2"; shift 2 ;;
    --ocr-preproc) OCR_PREPROC="$2"; shift 2 ;;
    --ocr-min-plate-h) OCR_MIN_PLATE_H="$2"; shift 2 ;;
    --ocr-min-sharpness) OCR_MIN_SHARPNESS="$2"; shift 2 ;;
    --ocr-min-occ-ratio) OCR_MIN_OCC_RATIO="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
  esac
done

if [[ -z "$VEH_MODEL" || -z "$PLATE_MODEL" || -z "$OCR_MODEL" || -z "$OCR_KEYS" || -z "$LABELS" ]]; then
  echo "Missing required args: --veh-model --plate-model --ocr-model --ocr-keys --labels" >&2
  usage
  exit 1
fi

mkdir -p "$OUT_DIR" "$OUT_DIR/ocr_dump"
PRED_LOG="$OUT_DIR/pred_live.csv"
CAPTURE_LOG="$OUT_DIR/capture.log"
FRAME_DUMP_PATH="$OUT_DIR/frame_raw.ppm"
OCR_INDEX="$OUT_DIR/ocr_dump/index.csv"
MANIFEST_CAND="$OUT_DIR/roi_manifest_candidates.csv"

CMD=(
  timeout "${DURATION_SEC}s"
  sudo ./run_lpr_kms.sh
  --device "$DEVICE"
  --drm-card "$DRM_CARD"
  --veh-model "$VEH_MODEL"
  --plate-model "$PLATE_MODEL"
  --ocr-model "$OCR_MODEL"
  --ocr-keys "$OCR_KEYS"
  --labels "$LABELS"
  --pred-log "$PRED_LOG"
  --fps "$FPS"
  --pixel-order "$PIXEL_ORDER"
  --swap16 "$SWAP16"
  --timeout-ms "$TIMEOUT_MS"
  --stats-interval "$STATS_INTERVAL"
  --copy-buffers "$COPY_BUFFERS"
  --queue-depth "$QUEUE_DEPTH"
  --min-car-conf "$MIN_CAR_CONF"
  --min-plate-conf "$MIN_PLATE_CONF"
  --plate-only "$PLATE_ONLY"
  --sw-preproc "$SW_PREPROC"
  --fpga-a-mask "$FPGA_A_MASK"
  --fpga-preproc-profile raw
  --fpga-preproc-target ocr
  --fpga-a-format flags
  --det-resize-mode "$DET_RESIZE_MODE"
  --plate-refine "$PLATE_REFINE"
  --ocr-channel-order "$OCR_CHANNEL_ORDER"
  --ocr-crop-mode "$OCR_CROP_MODE"
  --ocr-resize-mode "$OCR_RESIZE_MODE"
  --ocr-resize-kernel "$OCR_RESIZE_KERNEL"
  --ocr-preproc "$OCR_PREPROC"
  --ocr-min-plate-h "$OCR_MIN_PLATE_H"
  --ocr-min-sharpness "$OCR_MIN_SHARPNESS"
  --ocr-min-occ-ratio "$OCR_MIN_OCC_RATIO"
  --show-crop-box "$SHOW_CROP_BOX"
  --ocr-crop-dump-dir "$OUT_DIR/ocr_dump"
  --ocr-crop-dump-max "$MAX_CROP_DUMPS"
  --fpga-preproc-dump-path "$FRAME_DUMP_PATH"
  --fpga-preproc-dump-max "$MAX_FRAME_DUMPS"
)

echo "Launching capture: ${CMD[*]}"
"${CMD[@]}" 2>&1 | tee "$CAPTURE_LOG" || true

python3 ./build_roi_manifest_from_live.py \
  --pred "$PRED_LOG" \
  --capture-log "$CAPTURE_LOG" \
  --ocr-index "$OCR_INDEX" \
  --scenario "$SCENARIO" \
  --out-csv "$MANIFEST_CAND"

echo "candidate_manifest=$MANIFEST_CAND"
echo "pred_log=$PRED_LOG"
echo "capture_log=$CAPTURE_LOG"
echo "ocr_dump_dir=$OUT_DIR/ocr_dump"
