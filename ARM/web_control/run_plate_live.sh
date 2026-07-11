#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
ARM_ROOT="$(cd -- "${SCRIPT_DIR}/.." && pwd)"
MODEL_DIR="${MODEL_DIR:-/userdata/model}"

exec "${ARM_ROOT}/pplcnet_bgp_live" \
  --plate-model "${MODEL_DIR}/best_fp16.rknn" \
  --ocr-blue-model "${MODEL_DIR}/pplcnet_blue_v3_rk3568_fp16.rknn" \
  --ocr-green-model "${MODEL_DIR}/pplcnet_green_v2_b1plus_rk3568_fp16.rknn" \
  --ocr-yellow-model "${MODEL_DIR}/pplcnet_yellow_all_single_v1_rk3568_fp16.rknn" \
  --ocr-police-model "${MODEL_DIR}/pplcnet_police_v4_warmblue_rk3568_fp16.rknn" \
  --ocr-embassy-model "${MODEL_DIR}/pplcnet_embassy_v1_rk3568_fp16.rknn" \
  --plate-type-classifier-model "${MODEL_DIR}/plate_type_classifier_6cls_resnet18_warped_nocrop_rk3568_fp16_opt0.rknn" \
  --ocr-blue-keys "${MODEL_DIR}/special_keys.txt" \
  --ocr-green-keys "${MODEL_DIR}/pplcnet_green_keys.txt" \
  --ocr-yellow-keys "${MODEL_DIR}/yellow_keys.txt" \
  --ocr-police-keys "${MODEL_DIR}/police_keys.txt" \
  --ocr-embassy-keys "${MODEL_DIR}/embassy_keys.txt" \
  --ocr-keys "${MODEL_DIR}/special_keys.txt" \
  --det-resize letterbox \
  --ocr-preproc gray \
  --fps 60 \
  --display-sync 1 \
  --det-score-scale 1 \
  --min-plate-conf 0.45 \
  --plate-nms-iou 0.65 \
  --plate-max-det 9 \
  --source fpga \
  --phone-rtsp rtsp://127.0.0.1:8554/phone \
  --control-socket /run/pplcnet-bgp-live/control.sock
