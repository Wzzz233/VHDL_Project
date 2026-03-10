# FPGA AI-ISP A/B Report Template

## 1. Scope
- Date:
- Branch / commit:
- Bitstream:
- OV5640 preset: `REF | LOW_NOISE_BALANCED | LOW_NOISE_DISPLAY`
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
  - `median`
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

# Display-only denoise A/B
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
  --fpga-preproc-profile median \
  --fpga-preproc-target all \
  --fpga-a-format flags \
  --pred-log /tmp/score/pred_live_median_all.csv
```

## 5. Metrics
- Required:
  - live image stable / no black screen / no snow
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
| daylight | median |  |  |  |  |  |
| daylight | clahe |  |  |  |  |  |
| daylight | clahe_usm |  |  |  |  |  |
| daylight | median_clahe_usm |  |  |  |  |  |
| night | raw |  |  |  |  |  |
| night | median |  |  |  |  |  |
| night | clahe |  |  |  |  |  |
| night | clahe_usm |  |  |  |  |  |
| night | median_clahe_usm |  |  |  |  |  |

## 6.1 Sensor Preset Matrix
| subset | ov5640_preset | profile | blank_top1_mean | ocr_nonempty_ratio | fps_mean | notes |
|---|---|---|---:|---:|---:|---|
| daylight | REF | raw |  |  |  |  |
| daylight | LOW_NOISE_BALANCED | raw |  |  |  |  |
| daylight | LOW_NOISE_DISPLAY | raw |  |  |  |  |
| night | REF | raw |  |  |  |  |
| night | LOW_NOISE_BALANCED | raw |  |  |  |  |
| night | LOW_NOISE_DISPLAY | raw |  |  |  |  |

## 7. Acceptance Gates
- Gate-1: no regression to black screen, snow, stripes, or obvious frame corruption.
- Gate-2: `median` or tuned sensor preset should reduce visible weak-light grain versus `raw` baseline.
- Gate-3: Global `len_ge_7_ratio` should not be materially worse than `raw`.
- Gate-4: `ocr_nonempty_ratio` should not be worse than baseline by more than 5% absolute unless image quality gain is compelling and intentional.
- Gate-5: 15fps target, 60s run, no crash / timeout / forced exit.

## 8. Conclusion
- Best profile:
- Best OV5640 preset:
- Keep / rollback decision:
- Follow-up actions:
