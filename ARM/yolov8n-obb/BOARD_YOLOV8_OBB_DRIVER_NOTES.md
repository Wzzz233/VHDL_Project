# Board Driver Integration Notes: YOLOv8n-OBB

## RKNN Output Boundary

CPU driver now uses the preferred safe split for RK3568:

- `484`: `[1, 4, 8400]` decoded `ltrb`
- `415`: `[1, C, 8400]` class logits (`C` is dynamic, not hardcoded)
- `464`: `[1, 1, 8400]` decoded angle scalar

The driver does **not** depend on full `output0` decode tail and does not use rotated NMS in RKNN.

## Decode Formula (CPU)

For each of 8400 anchor points (`stride=8/16/32`, anchor center `(x+0.5, y+0.5)`):

- `score = sigmoid(class_logit)`
- `angle = angle_tensor` (already decoded, no extra sigmoid)
- `l,t,r,b = ltrb_tensor`
- `offset = (rb - lt)/2`
- `cx = anchor_x + offset_x * cos(angle) - offset_y * sin(angle)`
- `cy = anchor_y + offset_x * sin(angle) + offset_y * cos(angle)`
- `w = l + r`
- `h = t + b`
- `(cx, cy, w, h)` scaled by corresponding stride

Then convert OBB to 4-point quad and axis-aligned bbox.

## NMS Location

Rotated NMS is performed on CPU (`rotated_iou + rotated_nms_inplace`) with runtime-configurable parameters:

- confidence threshold: existing `--min-plate-conf`
- IoU threshold: `--plate-nms-iou`
- max detections: `--plate-max-det`
- optional class filter: `--plate-class-id`

## OCR Crop Change

- Added crop mode `obb_warp`.
- For OBB detections, driver reconstructs 4-point quad and applies perspective warp on CPU.
- Warped image then enters the existing LPRNet preprocessing path unchanged:
  - BGR
  - resize to 94x24
  - letterbox
  - nearest-neighbor
  - normalize `(x - 127.5) / 128.0`

When detector is `yolov8_obb_rknn`, driver force-aligns OCR settings to this contract:

- `ocr-channel-order=bgr`
- `ocr-resize-mode=letterbox`
- `ocr-resize-kernel=nn`

## Compatibility Switches

- New detector switch: `--plate-detector-type yolov5|yolov8_obb_rknn`
- New crop mode: `--ocr-crop-mode obb_warp`
- Default behavior remains YOLOv5 path.
- When detector is `yolov8_obb_rknn`, driver auto-forces `ocr-crop-mode=obb_warp`.

## Chinese Plate Text Display Fix

OCR decode now appends UTF-8 tokens safely without partial-byte truncation, and OCR key loading strips UTF-8 BOM on the first key entry. This fixes common Chinese character corruption in predicted plate text output.
