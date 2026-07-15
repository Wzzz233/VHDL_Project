#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ARM_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
MODEL_DIR="${MODEL_DIR:-/userdata/model}"
PLATE_TYPE_MODEL="${PLATE_TYPE_MODEL:-${MODEL_DIR}/plate_type_classifier_5color_large_green_v2_rk3568_fp16_opt0.rknn}"

exec "${ARM_ROOT}/pplcnet_bgp_live" \
  --plate-model "${MODEL_DIR}/best_fp16_5color_largegreen_scorex256_rk3568.rknn" \
  --ocr-blue-model "${MODEL_DIR}/pplcnet_blue_v3_rk3568_fp16.rknn" \
  --ocr-green-model "${MODEL_DIR}/pplcnet_green_v2_b1plus_rk3568_fp16.rknn" \
  --ocr-yellow-model "${MODEL_DIR}/pplcnet_yellow_all_single_v1_rk3568_fp16.rknn" \
  --ocr-police-model "${MODEL_DIR}/pplcnet_police_v5_whiteexpand_rk3568_fp16.rknn" \
  --ocr-embassy-model "${MODEL_DIR}/pplcnet_black_unified_v1_rk3568_fp16.rknn" \
  --plate-type-classifier-model "${PLATE_TYPE_MODEL}" \
  --ocr-blue-keys "${MODEL_DIR}/special_keys.txt" \
  --ocr-green-keys "${MODEL_DIR}/pplcnet_green_keys.txt" \
  --ocr-yellow-keys "${MODEL_DIR}/yellow_keys.txt" \
  --ocr-police-keys "${MODEL_DIR}/police_keys.txt" \
  --ocr-embassy-keys "${MODEL_DIR}/black_unified_keys.txt" \
  --ocr-keys "${MODEL_DIR}/special_keys.txt" \
  --det-resize letterbox \
  --ocr-preproc gray \
  --fps 30 \
  --display-sync 1 \
  --det-score-scale 256 \
  --min-plate-conf 0.35 \
  --plate-nms-iou 0.35 \
  --plate-max-det 24 \
  --source fpga \
  --phone-rtsp rtsp://127.0.0.1:8554/phone \
  --control-socket /run/pplcnet-bgp-live/control.sock
