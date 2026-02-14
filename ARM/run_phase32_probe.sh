#!/usr/bin/env bash
set -euo pipefail

COUNT="${COUNT:-60}"
PREFIX="${PREFIX:-/tmp/fprobe/motion}"
PPM_MODE="${PPM_MODE:-bgr565}"
PY_SCRIPT="${PY_SCRIPT:-./analyze_frame_diff.py}"

usage() {
  cat <<EOF
Usage: $0 [options]
  --count <N>        Frame count (default: ${COUNT})
  --prefix <path>    Output prefix (default: ${PREFIX})
  --ppm-mode <mode>  PPM mode for fpga_dma_test (default: ${PPM_MODE})
  --help             Show this help

Examples:
  sudo $0
  sudo $0 --count 80 --prefix /tmp/fprobe/motion_fast
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --count) COUNT="$2"; shift 2 ;;
    --prefix) PREFIX="$2"; shift 2 ;;
    --ppm-mode) PPM_MODE="$2"; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *)
      echo "Unknown option: $1" >&2
      usage
      exit 1
      ;;
  esac
done

if [[ ! -x ./fpga_dma_test ]]; then
  echo "fpga_dma_test not found/executable. Build it first: make testapp" >&2
  exit 2
fi
if [[ ! -f "$PY_SCRIPT" ]]; then
  echo "Analyzer script not found: $PY_SCRIPT" >&2
  exit 2
fi

OUT_DIR="$(dirname "$PREFIX")"
mkdir -p "$OUT_DIR"
rm -f "${PREFIX}"_*.raw "${PREFIX}"_*.ppm

echo "[1/3] Capture ${COUNT} motion frames (please move object by hand)..."
./fpga_dma_test \
  --continuous --count "$COUNT" \
  --read "$PREFIX" \
  --save-ppm "$PREFIX" \
  --ppm-mode "$PPM_MODE"

echo "[2/3] Analyze adjacent frame diffs..."
python3 "$PY_SCRIPT" --pattern "${PREFIX}"_*.raw

echo "[3/3] Done."
echo "Suggested manual visual check:"
echo "  ${PREFIX}_0000.ppm ${PREFIX}_0010.ppm ${PREFIX}_0020.ppm ${PREFIX}_0030.ppm ${PREFIX}_0040.ppm ${PREFIX}_0050.ppm"

