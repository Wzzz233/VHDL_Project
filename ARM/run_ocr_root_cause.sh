#!/usr/bin/env bash
set -euo pipefail

VEH_MODEL=""
PLATE_MODEL=""
OCR_MODEL=""
OCR_KEYS=""
LABELS=""
GT_TEXT=""
FPS="15"
DURATION_SEC="60"
COPY_BUFFERS="2"
QUEUE_DEPTH="1"
MIN_CAR_CONF="0.35"
MIN_PLATE_CONF="0.45"
OUT_DIR="/tmp/score"
LOG_DIR="/tmp"
OCR_DUMP_DIR="/tmp/ocr_cmp"
RUN_PHASE1="1"
DEMO_BIN=""

usage() {
  cat <<EOF
Usage: $0 --veh-model <path> --plate-model <path> --ocr-model <path> --ocr-keys <path> --labels <path> [options]
  --gt-text <text>         Known plate GT text for exact-match ratio (optional)
  --fps <n>                FPS (default: ${FPS})
  --duration-sec <n>       Duration per run in seconds (default: ${DURATION_SEC})
  --copy-buffers <n>       copy-buffers (default: ${COPY_BUFFERS})
  --queue-depth <n>        queue-depth (default: ${QUEUE_DEPTH})
  --min-car-conf <v>       car conf (default: ${MIN_CAR_CONF})
  --min-plate-conf <v>     plate conf (default: ${MIN_PLATE_CONF})
  --out-dir <path>         output directory for csv/json (default: ${OUT_DIR})
  --log-dir <path>         output directory for logs (default: ${LOG_DIR})
  --ocr-dump-dir <path>    dump dir used by ROI run (default: ${OCR_DUMP_DIR})
  --demo-bin <path>        official rknn_lprnet_demo binary path (for Phase1)
  --skip-phase1            skip official demo compare
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --veh-model) VEH_MODEL="$2"; shift 2 ;;
    --plate-model) PLATE_MODEL="$2"; shift 2 ;;
    --ocr-model) OCR_MODEL="$2"; shift 2 ;;
    --ocr-keys) OCR_KEYS="$2"; shift 2 ;;
    --labels) LABELS="$2"; shift 2 ;;
    --gt-text) GT_TEXT="$2"; shift 2 ;;
    --fps) FPS="$2"; shift 2 ;;
    --duration-sec) DURATION_SEC="$2"; shift 2 ;;
    --copy-buffers) COPY_BUFFERS="$2"; shift 2 ;;
    --queue-depth) QUEUE_DEPTH="$2"; shift 2 ;;
    --min-car-conf) MIN_CAR_CONF="$2"; shift 2 ;;
    --min-plate-conf) MIN_PLATE_CONF="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --log-dir) LOG_DIR="$2"; shift 2 ;;
    --ocr-dump-dir) OCR_DUMP_DIR="$2"; shift 2 ;;
    --demo-bin) DEMO_BIN="$2"; shift 2 ;;
    --skip-phase1) RUN_PHASE1="0"; shift 1 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
  esac
done

if [[ -z "$VEH_MODEL" || -z "$PLATE_MODEL" || -z "$OCR_MODEL" || -z "$OCR_KEYS" || -z "$LABELS" ]]; then
  echo "Missing required model/keys/labels args." >&2
  usage
  exit 1
fi

mkdir -p "$OUT_DIR" "$LOG_DIR"
rm -rf "$OCR_DUMP_DIR"
mkdir -p "$OCR_DUMP_DIR"

run_one() {
  local name="$1"
  local ocr_ch="$2"
  local crop_mode="$3"
  local resize_mode="$4"
  local pp="$5"
  local dump_dir="$6"

  local pred_csv="${OUT_DIR}/pred_${name}.csv"
  local log_file="${LOG_DIR}/lpr_${name}.log"
  local met_json="${OUT_DIR}/metrics_${name}.json"

  local cmd=(
    sudo ./run_lpr_kms.sh
    --veh-model "$VEH_MODEL"
    --plate-model "$PLATE_MODEL"
    --ocr-model "$OCR_MODEL"
    --ocr-keys "$OCR_KEYS"
    --labels "$LABELS"
    --fps "$FPS"
    --copy-buffers "$COPY_BUFFERS"
    --queue-depth "$QUEUE_DEPTH"
    --min-car-conf "$MIN_CAR_CONF"
    --min-plate-conf "$MIN_PLATE_CONF"
    --plate-only 1
    --sw-preproc 0
    --fpga-a-mask 0
    --ocr-channel-order "$ocr_ch"
    --ocr-crop-mode "$crop_mode"
    --ocr-resize-mode "$resize_mode"
    --ocr-preproc "$pp"
    --ocr-ctc-diag 1
    --ocr-crop-dump-max 40
    --pred-log "$pred_csv"
  )

  if [[ -n "$dump_dir" ]]; then
    cmd+=(--ocr-crop-dump-dir "$dump_dir")
  fi

  echo "=== [${name}] ch=${ocr_ch} crop=${crop_mode} resize=${resize_mode} pp=${pp} ==="
  timeout "${DURATION_SEC}s" "${cmd[@]}" 2>&1 | tee "$log_file" || true

  local mcmd=(python3 ./ocr_phase0_metrics.py --pred "$pred_csv" --out-json "$met_json")
  if [[ -n "$GT_TEXT" ]]; then
    mcmd+=(--gt-text "$GT_TEXT")
  fi
  "${mcmd[@]}" | tee "${OUT_DIR}/metrics_${name}.txt"
}

run_one "base" "rgb" "fixed" "stretch" "none" ""
run_one "ch"   "bgr" "fixed" "stretch" "none" ""
run_one "roi"  "bgr" "box-pad" "letterbox" "none" "$OCR_DUMP_DIR"
run_one "pp"   "bgr" "box-pad" "letterbox" "bin" ""

echo
echo "Phase0 done. Metrics json:"
ls -1 "${OUT_DIR}"/metrics_*.json 2>/dev/null || true

if [[ "$RUN_PHASE1" == "1" ]]; then
  if [[ -z "$DEMO_BIN" ]]; then
    echo "Phase1 skipped: --demo-bin not provided."
    exit 0
  fi
  if [[ ! -f "${OCR_DUMP_DIR}/index.csv" ]]; then
    echo "Phase1 skipped: dump index not found at ${OCR_DUMP_DIR}/index.csv"
    exit 0
  fi
  echo
  echo "=== Phase1 official demo compare ==="
  demo_cmd=(
    python3 ./ocr_demo_compare.py
    --index "${OCR_DUMP_DIR}/index.csv"
    --demo-bin "$DEMO_BIN"
    --model "$OCR_MODEL"
    --max-samples 20
    --out-csv "${OUT_DIR}/demo_compare_samples.csv"
    --out-json "${OUT_DIR}/demo_compare_summary.json"
  )
  if [[ -n "$GT_TEXT" ]]; then
    demo_cmd+=(--gt-text "$GT_TEXT")
  fi
  "${demo_cmd[@]}" | tee "${OUT_DIR}/demo_compare_summary.txt"
fi

echo "Done."
