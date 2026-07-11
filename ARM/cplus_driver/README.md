# C+ RK3568 FP16 Driver

This is a standalone FPGA/DMA driver for the 2026-07-10 C+ handoff. It does
not modify the live licence-plate pipeline.

The driver loads two RKNN models:

- `yolov5nu_coco_rk3568_fp16_20260710.rknn` for `person`, `bicycle`, and
  `motorcycle` detection.
- `mapillary_cplus_ground_4class_v2_rk3568_fp16_20260710.rknn` for `other`,
  `road`, `sidewalk`, and `crosswalk_zebra` segmentation.

The detector receives a bilinearly resized RGB letterboxed 640x640 image with padding value 114.
The segmenter receives a separately prepared bilinear RGB 640x640 direct resize. RKNN
applies the two model-specific normalizations, so this driver passes RGB uint8
pixels without an additional normalization step.

## Host test

```sh
make -C cplus_driver test
```

The host tests cover bilinear input mapping, FP32 and native FP16 decoding, the semantic-mask palette, class-aware suppression, rider filtering, final rule boundaries, and reusable Candidate C mask cleanup. They also exercise shared-frame reference lifetimes, pending-frame replacement, shutdown draining, and concurrent producer/consumer delivery.

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
Copy the executable and both `.rknn` files to the board. The driver shows the live FPGA image on HDMI by default, overlays the completed ground mask at 50 percent opacity, and draws target and pedestrian-foot boxes. Road is green, sidewalk is blue, and zebra crossing is red, matching the handoff review palette. Start a continuous board run with:

```sh
sudo ./cplus-rk3568-driver \
  --det-model /userdata/model/yolov5nu_coco_rk3568_fp16_20260710.rknn \
  --seg-model /userdata/model/mapillary_cplus_ground_4class_v2_rk3568_fp16_20260710.rknn \
  --device /dev/fpga_dma0 --frames 0 --fps 30 --display 1
```

`--frames 0` means continuous operation and `--fps 30` sets the capture cadence; press `Ctrl+C` to stop. A successful display startup prints the selected DRM connector and CRTC. The FPGA image size must be an available HDMI mode (the observed board stream is 1280x720). To use another DRM card or a specific connected HDMI connector, use `--drm-card PATH` and `--connector-id N`. Use `--display 0` for headless JSON-only operation.

The RKNN path reads native FP16 outputs instead of expanding the two tensors to FP32. Candidate C uses an exact four-core circular morphology implementation and reuses its scratch memory between frames.

The live path uses three independent stages: paced DMA capture, an asynchronous latest-frame HDMI thread, and an asynchronous latest-frame RKNN thread. Both worker queues replace an older pending frame with the newest submission, so a slow display or inference never blocks capture and does not build latency. The result boxes describe the most recently completed inference and may trail the current camera frame by one inference interval. Before the first result completes, HDMI shows `CPLUS STARTING`. The exit summary reports completed work and pending frames replaced by fresher frames.

By default, segmentation runs only when an ordinary pedestrian remains after rider filtering. Add `--always-segment` during visual mask validation to keep the mask active even with no pedestrian; remove it for the normal lower-latency run. Each segmented inference prints `timing_ms` for every stage and a `[mask]` line with raw and postprocessed pixel counts. `det_npu` and `seg_npu` are the RKNN runtime durations; compare them with `det_run` and `seg_run` to separate NPU execution from input/output overhead. Startup also prints the NPU clock when the kernel exposes it. A healthy road scene normally has at least one nonzero count among `road`, `sidewalk`, and `zebra`. An all-zero ground result now produces a one-time warning.

After an inference result, when a frame produces no targets, the HDMI image still appears with `CPLUS NO TARGET`; this distinguishes a display problem from an empty detector result. To retain the original, unannotated FPGA frame for inspection, add `--dump-bgrx /userdata/cplus_capture.bgrx --frames 1 --display 0`. The 1280x720 file can then be viewed on the host with:

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
output is accepted as 84 by 8400 FP16 or FP32 values, segmenter output is accepted as four
contiguous NCHW 640x640 FP16 or FP32 planes, raw mask counts include expected ground classes, the HDMI overlay uses the handoff colors, the 50 supplied images are visually sensible, and real camera
frames retain the expected letterbox/direct-resize coordinate alignment.
