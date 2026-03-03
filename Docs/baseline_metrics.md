# Baseline Metrics (OCR Focus)

## Scope
- Date: 2026-03-03
- Branch: `main`
- Pipeline: `run_lpr_kms.sh` offline mode
- Detector model: `/userdata/model/plate_yolov5n_rk3568.rknn`
- OCR models:
  - INT8: `/userdata/model/ocr_lprnet_rk3568.rknn`
  - FP16: `/userdata/model/ocr_lprnet_rk3568_fp16.rknn`

## Unified Test Settings
- `--ocr-channel-order bgr`
- `--ocr-crop-mode match`
- `--ocr-resize-mode letterbox`
- `--ocr-resize-kernel nn`
- `--ocr-min-occ-ratio 0.90`
- `--ocr-ctc-diag 1`

## Key Results

| Case | Model | blank_top1 | OCR text | Note |
|---|---|---:|---|---|
| `/tmp/frame_bgrx.jpg` | INT8 | 0.778 | `NRA2` | geometry already corrected |
| `/tmp/frame_bgrx.jpg` | FP16 | 0.667 | `*RA26T` | same crop chain, lower blank bias |
| `/tmp/test2.jpg` (auto det) | INT8 | 0.833 | `*66` | under-recognition |
| `/tmp/test2.jpg` (auto det) | FP16 | 0.611 | `*1N6666` | closer to full plate |

## Observations
- `crop-geom` can reach `iou=1.000` with `match` mode, so geometry mismatch is not the primary blocker now.
- `blank_top1` consistently lower on FP16 than INT8 under same chain.
- Main bottleneck is currently quantization/model behavior, not crop alignment.

## Next Metrics to Add
- Batch-level `ocr_nonempty_ratio`, `len>=7_ratio`, `exact_match_ratio`.
- Realtime 60s stability (FPS, timeout, dropped frame).
- INT8 re-quantized model comparison against current INT8 and FP16.
