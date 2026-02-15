#!/usr/bin/env bash
set -euo pipefail

REG_SCRIPT="${REG_SCRIPT:-./audit_reg_config.py}"
METRIC_SCRIPT="${METRIC_SCRIPT:-./analyze_ppm_metrics.py}"
REG_FILE="${REG_FILE:-../hdl/reg_config.v}"

COUNT="${COUNT:-60}"
PREFIX="${PREFIX:-/tmp/fprobe/phaseS}"
PPM_MODE="${PPM_MODE:-bgr565}"
LABEL="${LABEL:-static_texture}"
STRICT_AUDIT="${STRICT_AUDIT:-1}"
FORCE_RUN="${FORCE_RUN:-0}"
SAMPLE_STEP="${SAMPLE_STEP:-2}"

usage() {
  cat <<EOF
Usage: $0 [options]
  --count <N>          Frame count (default: ${COUNT})
  --prefix <path>      Output prefix (default: ${PREFIX})
  --ppm-mode <mode>    PPM decode mode for fpga_dma_test (default: ${PPM_MODE})
  --label <name>       Report label (default: ${LABEL})
  --sample-step <N>    Spatial downsample step for metrics (default: ${SAMPLE_STEP})
  --no-strict-audit    Do not fail when register table audit is untrusted
  --force-run          Continue capture/metrics even if strict audit fails
  --help               Show this help

Examples:
  sudo $0 --label static_checker
  sudo $0 --label graycard_indoor --prefix /tmp/fprobe/gray_indoor
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --count) COUNT="$2"; shift 2 ;;
    --prefix) PREFIX="$2"; shift 2 ;;
    --ppm-mode) PPM_MODE="$2"; shift 2 ;;
    --label) LABEL="$2"; shift 2 ;;
    --sample-step) SAMPLE_STEP="$2"; shift 2 ;;
    --no-strict-audit) STRICT_AUDIT="0"; shift 1 ;;
    --force-run) FORCE_RUN="1"; shift 1 ;;
    -h|--help) usage; exit 0 ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ ! -x ./fpga_dma_test ]]; then
  echo "fpga_dma_test not found/executable. Build first: make testapp" >&2
  exit 2
fi
if [[ ! -f "$REG_SCRIPT" ]]; then
  echo "Register audit script not found: $REG_SCRIPT" >&2
  exit 2
fi
if [[ ! -f "$METRIC_SCRIPT" ]]; then
  echo "Metric script not found: $METRIC_SCRIPT" >&2
  exit 2
fi

echo "[1/3] Register-table blocker audit..."
if [[ "$STRICT_AUDIT" == "1" ]]; then
  if ! python3 "$REG_SCRIPT" --file "$REG_FILE" --strict; then
    if [[ "$FORCE_RUN" == "1" ]]; then
      echo "WARN: strict audit failed, but continuing due to --force-run"
    else
      echo "ERR: blocker audit failed; fix init table first or rerun with --force-run" >&2
      exit 3
    fi
  fi
else
  python3 "$REG_SCRIPT" --file "$REG_FILE"
fi

OUT_DIR="$(dirname "$PREFIX")"
mkdir -p "$OUT_DIR"
rm -f "${PREFIX}"_*.raw "${PREFIX}"_*.ppm

echo "[2/3] Capture frames for scenario label='${LABEL}'..."
echo "      Scene guidance:"
echo "      - static texture: checkerboard / printed text / high-frequency pattern"
echo "      - gray card light test: fixed framing, run once per light condition"
./fpga_dma_test \
  --continuous --count "$COUNT" \
  --read "$PREFIX" \
  --save-ppm "$PREFIX" \
  --ppm-mode "$PPM_MODE"

echo "[3/3] Analyze sharpness/aliasing + AWB/AEC behavior..."
python3 "$METRIC_SCRIPT" \
  --pattern "${PREFIX}_*.ppm" \
  --label "$LABEL" \
  --sample-step "$SAMPLE_STEP"

echo "Done."
echo "For AWB/AEC comparison, repeat with fixed framing:"
echo "  1) --label gray_indoor   --prefix /tmp/fprobe/gray_indoor"
echo "  2) --label gray_daylight --prefix /tmp/fprobe/gray_daylight"
echo "  3) --label gray_filllight --prefix /tmp/fprobe/gray_filllight"

