# Project Status Summary (2026-03-03)

## Scope
- Branch: `main`
- Focus area: OCR chain convergence for plate recognition
- Code files in this update:
  - `ARM/fpga_lpr_display.c`
  - `ARM/run_lpr_kms.sh`
  - `ARM/ocr_demo_compare.py`

## Requirements vs Progress (Docs Cross-check)

Cross-checked against:
- Competition requirements document in `Docs/` (Chinese filename).
- Internal requirements-progress mapping document in `Docs/` (Chinese filename).

Current conclusion:
- End-to-end capture -> detect -> OCR -> display path is available.
- Geometry/crop correctness for OCR is now converged and explainable.
- Main remaining blocker for OCR accuracy is model/quantization behavior (INT8 vs FP16), not crop-box alignment.

## What Was Implemented in This Round

### 1. Geometry first: blue/yellow box alignment
- Added `--ocr-crop-mode match`:
  - `crop_box == det_box` (zero padding)
- Added geometry log:
  - `[crop-geom] box=[...] crop=[...] iou=...`
- Verified case:
  - `iou=1.000` in offline mode.

### 2. Better downscale control for OCR input
- Added `--ocr-resize-kernel nn|bilinear` (default `nn` for compatibility).
- Kept existing `--ocr-resize-mode stretch|letterbox` behavior unchanged.
- Added OCR input diagnostics:
  - `[ocrin] resize_mode=... kernel=... in_occ_ratio=...`

### 3. One-shot adaptive recrop by effective occupancy
- Added `--ocr-min-occ-ratio <float>` (default `0`, disabled).
- When `in_occ_ratio` is too low, recrop at most once:
  - For `match`: use `match-ytrim` only if truly improvable.
  - For non-`match`: fallback path remains available.
- Added recrop diagnostics:
  - `[ocr-recrop] trigger=1 old_occ=... new_mode=... new_occ=...`
  - `[ocr-recrop] trigger=0 ... reason=not-improvable`

### 4. Evidence export + app/demo compare
- `index.csv` now includes:
  - `app_blank_top1`
  - `app_occ_ratio`
- `ocr_demo_compare.py` now reports:
  - `blank_top1_mean`
  - improved demo text parsing robustness.

### 5. FPGA AI-ISP control path (phase-1/2 skeleton)
- Added FPGA BAR1 preproc register block plumbing:
  - `PREP_CTRL @0x200`
  - `PREP_CLAHE @0x204`
  - `PREP_USM @0x208`
  - `PREP_MED @0x20C`
- Added kernel ioctl register bridge:
  - `FPGA_DMA_SET_BAR1_REG`
  - `FPGA_DMA_GET_BAR1_REG` (driver shadow readback)
- Added app/runtime controls:
  - `--fpga-preproc-profile raw|clahe|clahe_usm|median_clahe_usm`
  - `--fpga-preproc-target ocr|all`
  - `--fpga-a-format flags|yenh`
  - `--fpga-clahe tile=64x64,clip=48,strength=192`
  - `--fpga-usm gain=1.0,thr=6,limit=24`
- Added BGR565->BGRX pipeline support for:
  - Y-channel enhancement proxy in FPGA stream path
  - OCR-only mode (`target=ocr`) via A-channel `Y_enh`
  - Full-path mode (`target=all`) via luma remap back to RGB

## Key Evidence (from latest logs)

### Geometry converged
- Example:
  - `box=[403,161,956,339] crop=[403,161,956,339] iou=1.000`

### INT8 vs FP16 gap (same chain/settings)
- Sample A:
  - INT8: `blank_top1=0.778`, `text=NRA2`
  - FP16: `blank_top1=0.667`, `text=*RA26T` (province prefix recovered)
- Sample B (`test2`):
  - INT8: `blank_top1=0.833`, `text=*66` (short/incomplete)
  - FP16: `blank_top1=0.611`, `text=*1N6666` (longer and closer)

Interpretation:
- Crop geometry issues are no longer the main root cause.
- Quantization/model confidence behavior is now the dominant issue.

## Recommended Baseline Runtime Params

Use this as current baseline for reproducible OCR tests:
- `--ocr-channel-order bgr`
- `--ocr-crop-mode match`
- `--ocr-resize-mode letterbox`
- `--ocr-resize-kernel nn`
- `--ocr-min-occ-ratio 0.90`

## Open Items

1. Build a plate-ROI calibration set and re-quantize INT8 OCR model.
2. Keep FP16 as temporary quality fallback during validation.
3. Continue detector robustness tuning for hard images where auto detect is empty.
4. Run 60s realtime regression with crop-box overlay only in validation runs.

## Status

- OCR chain observability: **Done**
- Geometry alignment: **Done**
- Downscale kernel A/B switch: **Done**
- One-shot recrop control: **Done**
- Accuracy closure: **In progress** (currently limited by INT8 model behavior)
