#!/usr/bin/env bash
set -euo pipefail

DEVICE="/dev/fpga_dma0"
DRM_CARD="/dev/dri/card0"
FPS="15"
DURATION_SEC="20"
OUT_DIR="/tmp/score"
COPY_BUFFERS="2"
QUEUE_DEPTH="1"
RELOAD_DRIVER="1"

VEH_MODEL=""
PLATE_MODEL=""
OCR_MODEL=""
OCR_KEYS=""
LABELS=""

usage() {
  cat <<EOF
Usage: $0 --veh-model <path> --plate-model <path> --ocr-model <path> --ocr-keys <path> --labels <path> [options]
  --device <path>         FPGA device (default: ${DEVICE})
  --drm-card <path>       DRM card (default: ${DRM_CARD})
  --fps <n>               Target FPS (default: ${FPS})
  --duration-sec <n>      Capture seconds per scenario (default: ${DURATION_SEC})
  --out-dir <path>        Output directory for scenario prediction logs (default: ${OUT_DIR})
  --copy-buffers <n>      Copy buffers (default: ${COPY_BUFFERS})
  --queue-depth <n>       Queue depth (default: ${QUEUE_DEPTH})
  --reload-driver <0|1>   rmmod/insmod before capture (default: ${RELOAD_DRIVER})
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --veh-model) VEH_MODEL="$2"; shift 2 ;;
    --plate-model) PLATE_MODEL="$2"; shift 2 ;;
    --ocr-model) OCR_MODEL="$2"; shift 2 ;;
    --ocr-keys) OCR_KEYS="$2"; shift 2 ;;
    --labels) LABELS="$2"; shift 2 ;;
    --device) DEVICE="$2"; shift 2 ;;
    --drm-card) DRM_CARD="$2"; shift 2 ;;
    --fps) FPS="$2"; shift 2 ;;
    --duration-sec) DURATION_SEC="$2"; shift 2 ;;
    --out-dir) OUT_DIR="$2"; shift 2 ;;
    --copy-buffers) COPY_BUFFERS="$2"; shift 2 ;;
    --queue-depth) QUEUE_DEPTH="$2"; shift 2 ;;
    --reload-driver) RELOAD_DRIVER="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "Unknown option: $1" >&2; usage; exit 1 ;;
  esac
done

if [[ -z "${VEH_MODEL}" || -z "${PLATE_MODEL}" || -z "${OCR_MODEL}" || -z "${OCR_KEYS}" || -z "${LABELS}" ]]; then
  echo "Missing required args." >&2
  usage
  exit 1
fi

mkdir -p "${OUT_DIR}"

if [[ "${RELOAD_DRIVER}" == "1" ]]; then
  sudo rmmod pcie_fpga_dma 2>/dev/null || true
  sudo insmod pcie_fpga_dma.ko dma_pixel_format=1
fi

scenarios=(sunny cloudy night far tilt)

echo "Output dir: ${OUT_DIR}"
echo "Scenarios: ${scenarios[*]}"

for s in "${scenarios[@]}"; do
  pred="${OUT_DIR}/pred_${s}.csv"
  echo
  echo "=== Scenario: ${s} ==="
  echo "Please prepare scene '${s}', then press Enter to start capture..."
  read -r _

  timeout "${DURATION_SEC}s" sudo ./run_lpr_kms.sh \
    --device "${DEVICE}" \
    --drm-card "${DRM_CARD}" \
    --veh-model "${VEH_MODEL}" \
    --plate-model "${PLATE_MODEL}" \
    --ocr-model "${OCR_MODEL}" \
    --ocr-keys "${OCR_KEYS}" \
    --labels "${LABELS}" \
    --fps "${FPS}" \
    --copy-buffers "${COPY_BUFFERS}" \
    --queue-depth "${QUEUE_DEPTH}" \
    --plate-only 1 \
    --pred-log "${pred}" || true

  if [[ -f "${pred}" ]]; then
    rows=$(($(wc -l < "${pred}") - 1))
    if [[ ${rows} -lt 0 ]]; then rows=0; fi
    echo "saved: ${pred} (rows=${rows})"
  else
    echo "warn: ${pred} was not generated"
  fi
done

echo
echo "Capture matrix completed."
echo "Next steps:"
echo "1) Prepare GT csv per scenario."
echo "2) Run eval_lpr.py for each scenario or merged dataset."
