# FPGA AI-ISP A/B Report Template

## 1. Scope
- Date:
- Branch / commit:
- Bitstream:
- OCR model:
- Runtime command baseline:

## 2. Test Matrix
- Dataset subsets:
  - Daylight clear:
  - Night low-light:
  - Backlight / glare:
  - Mild blur / mild tilt:
- Profiles:
  - `raw`
  - `clahe`
  - `clahe_usm`
  - `median_clahe_usm`

## 3. Fixed Runtime Knobs
- `--ocr-channel-order bgr`
- `--ocr-crop-mode match`
- `--ocr-resize-mode letterbox`
- `--ocr-resize-kernel nn`
- `--ocr-min-occ-ratio 0.90`

## 4. Commands
```bash
# Example baseline
sudo ./run_lpr_kms.sh \
  --veh-model /userdata/model/yolov5s_rk3568.rknn \
  --plate-model /userdata/model/plate_yolov5n_rk3568.rknn \
  --ocr-model /userdata/model/ocr_lprnet_rk3568_fp16.rknn \
  --ocr-keys /userdata/model/ocr_keys_lprnet.txt \
  --labels /userdata/model/coco_80_labels_list.txt \
  --fps 15 --copy-buffers 2 --queue-depth 1 \
  --plate-only 1 --det-resize-mode letterbox --plate-refine 1 \
  --ocr-channel-order bgr --ocr-crop-mode match \
  --ocr-resize-mode letterbox --ocr-resize-kernel nn \
  --ocr-min-occ-ratio 0.90 \
  --fpga-preproc-profile raw \
  --fpga-preproc-target ocr \
  --fpga-a-format flags \
  --pred-log /tmp/score/pred_live_raw.csv
```

## 5. Metrics
- Required:
  - `blank_top1_mean`
  - `ocr_nonempty_ratio`
  - `len_ge_7_ratio`
  - realtime FPS stability
- Optional:
  - `exact_match_ratio` (if GT provided)

## 6. Results Table
| subset | profile | blank_top1_mean | ocr_nonempty_ratio | len_ge_7_ratio | fps_mean | notes |
|---|---:|---:|---:|---:|---:|---|
| daylight | raw |  |  |  |  |  |
| daylight | clahe |  |  |  |  |  |
| daylight | clahe_usm |  |  |  |  |  |
| daylight | median_clahe_usm |  |  |  |  |  |
| night | raw |  |  |  |  |  |
| night | clahe |  |  |  |  |  |
| night | clahe_usm |  |  |  |  |  |
| night | median_clahe_usm |  |  |  |  |  |

## 7. Acceptance Gates
- Gate-1: `clahe_usm` on (night + backlight) has `ocr_nonempty_ratio` >= raw + 8% absolute.
- Gate-2: `blank_top1_mean` <= raw - 0.06.
- Gate-3: Global `len_ge_7_ratio` not lower than raw.
- Gate-4: 15fps target, long-run no crash.

## 8. Conclusion
- Best profile:
- Keep / rollback decision:
- Follow-up actions:
