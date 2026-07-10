# C+ RK3568 FP16 Driver

This is a standalone FPGA/DMA driver for the 2026-07-10 C+ handoff. It does
not modify the live licence-plate pipeline.

The driver loads two RKNN models:

- `yolov5nu_coco_rk3568_fp16_20260710.rknn` for `person`, `bicycle`, and
  `motorcycle` detection.
- `mapillary_cplus_ground_4class_v2_rk3568_fp16_20260710.rknn` for `other`,
  `road`, `sidewalk`, and `crosswalk_zebra` segmentation.

The detector receives an RGB letterboxed 640x640 image with padding value 114.
The segmenter receives a separately prepared RGB 640x640 direct resize. RKNN
applies the two model-specific normalizations, so this driver passes RGB uint8
pixels without an additional normalization step.

## Host test

```sh
make -C cplus_driver test
```

The host test covers the two input mappings, `[1,84,8400]` YOLO decoding and
class-aware suppression, rider filtering, final rule boundaries, and Candidate
C mask cleanup.

## Board build

Use the verified local Docker environment:

```sh
cd <feature-branch-checkout>/ARM
sudo docker run --rm --privileged -u root \
  -v "$PWD":/app \
  -v /home/wzzz/RK3568J_SDK:/home/hjf/SDK \
  -w /app/cplus_driver \
  cdc81f835218 make -f Makefile.rk3568 cplus-rk3568-driver
```

The board must have a compatible `librknnrt.so` and a BGRX8888 FPGA DMA stream.
Copy the executable and both `.rknn` files to the board. The driver shows the live FPGA image on HDMI by default and draws the retained targets directly on that image. Start a continuous board run with:

```sh
sudo ./cplus-rk3568-driver \
  --det-model /userdata/model/yolov5nu_coco_rk3568_fp16_20260710.rknn \
  --seg-model /userdata/model/mapillary_cplus_ground_4class_v2_rk3568_fp16_20260710.rknn \
  --device /dev/fpga_dma0 --frames 0 --display 1
```

`--frames 0` means continuous operation; press `Ctrl+C` to stop. A successful display startup prints the selected DRM connector and CRTC. The FPGA image size must be an available HDMI mode (the observed board stream is 1280x720). To use another DRM card or a specific connected HDMI connector, use `--drm-card PATH` and `--connector-id N`. Use `--display 0` for headless JSON-only operation.

When a frame produces no targets, the HDMI image still appears with `CPLUS NO TARGET`; this distinguishes a display problem from an empty detector result. To retain the original, unannotated FPGA frame for inspection, add `--dump-bgrx /userdata/cplus_capture.bgrx --frames 1 --display 0`. The 1280x720 file can then be viewed on the host with:

```sh
ffplay -f rawvideo -pixel_format bgr0 -video_size 1280x720 /userdata/cplus_capture.bgrx
```

For a saved BGRX8888 frame, replace the DMA options with `--input-bgrx frame.bgrx --width 1280 --height 720`. Each output line is one JSON result containing retained targets, the decision, its reason, the original image box, and the road/sidewalk/zebra ratios.

The source handoff's model checksums are:

```text
detector   07be6f10cafc6544b2fbc6ce9af5cb3c24938464890786007e0f57bea1bf0ef1
segmenter  50c4ef148830f6e77db4ebf9ce6c1d38bc0435ffe29f3cb075e0a4adcca0bda8
```

The board acceptance gate is: both models report a 640x640 input, detector
output is accepted as 84 by 8400 values, segmenter output is accepted as four
640x640 planes, the 50 supplied images are visually sensible, and real camera
frames retain the expected letterbox/direct-resize coordinate alignment.
