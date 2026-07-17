#!/bin/bash
# 视频诊断 v2：强制硬解 + 精确 fps（字节计数，无需 Ctrl+C 读数）。
# 用法：bash diag_video_v2.sh [视频路径]
set -u
V="${1:-/mnt/sdcard/test.mp4}"
D="${DRIVER:-/home/linaro/ARM/cplus-rk3568-driver}"
M="${MODELS:-/userdata/model}"
DURATION=12                      # 每个测速段跑 12 秒
FRAME=$((1280 * 720 * 4))        # 单帧字节数 = 3686400

export GST_PLUGIN_FEATURE_RANK=mppvideodec:300   # 和生产一致，强制硬解

echo "视频: $V  driver: $D  models: $M  frame_bytes=$FRAME  (强制 mppvideodec)"
[ -f "$V" ] || { echo "视频不存在"; exit 1; }

echo
echo "===== 0. 确认 decodebin 选了 mppvideodec (应看到 mppvideodec0) ====="
timeout 4 gst-launch-1.0 -v filesrc location="$V" ! decodebin ! fakesink sync=false 2>&1 | grep -iE "mppvideodec|avdec|nvh264|v4l2" | head -3

echo
echo "===== A. 硬解+转BGRx，无缩放 (看 1080p 转换本身多快) ====="
A=$(timeout "$DURATION" gst-launch-1.0 -q filesrc location="$V" ! decodebin ! videoconvert ! video/x-raw,format=BGRx ! fdsink fd=1 sync=false 2>/dev/null | wc -c)
awk -v b="$A" -v f="$FRAME" -v t="$DURATION" 'BEGIN{printf "  A(1080p解+转BGRx,无缩放) -> %.1f fps\n", b/f/t}'

echo
echo "===== B. 生产现状: 硬解 + videoconvert单线程 + scale 到720p ====="
B=$(timeout "$DURATION" gst-launch-1.0 -q filesrc location="$V" ! decodebin ! videoconvert ! videoscale add-borders=true ! video/x-raw,format=BGRx,width=1280,height=720 ! fdsink fd=1 sync=false 2>/dev/null | wc -c)
awk -v b="$B" -v f="$FRAME" -v t="$DURATION" 'BEGIN{printf "  B(单线程 convert+scale) -> %.1f fps\n", b/f/t}'

echo
echo "===== C. 硬解 + videoconvert n-threads=4 + scale ====="
C=$(timeout "$DURATION" gst-launch-1.0 -q filesrc location="$V" ! decodebin ! videoconvert n-threads=4 ! videoscale add-borders=true ! video/x-raw,format=BGRx,width=1280,height=720 ! fdsink fd=1 sync=false 2>/dev/null | wc -c)
awk -v b="$C" -v f="$FRAME" -v t="$DURATION" 'BEGIN{printf "  C(4线程 convert+scale) -> %.1f fps\n", b/f/t}'

echo
echo "===== D. 候选优化: 先 scale(NV12) 再 convert (转换像素更少) ====="
D2=$(timeout "$DURATION" gst-launch-1.0 -q filesrc location="$V" ! decodebin ! videoscale add-borders=true ! videoconvert ! video/x-raw,format=BGRx,width=1280,height=720 ! fdsink fd=1 sync=false 2>/dev/null | wc -c)
awk -v b="$D2" -v f="$FRAME" -v t="$DURATION" 'BEGIN{printf "  D(先scale后convert) -> %.1f fps\n", b/f/t}'

echo
echo "===== E. 生产等价完整管线 + driver (强制硬解，看 input_fps/infer) ====="
echo "  先在网页停掉实时车牌释放 NPU，再回车"
read -r -p "  按回车继续..." _
timeout 25 gst-launch-1.0 -q filesrc location="$V" ! decodebin ! capsfilter caps=video/x-raw ! identity sync=true ! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! videoconvert ! videoscale add-borders=true ! video/x-raw,format=BGRx,width=1280,height=720 ! queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! fdsink fd=1 sync=false 2>/dev/null | \
"$D" --det-model "$M/yolov5nu_coco_rk3568_fp16_20260710.rknn" --seg-model "$M/mapillary_cplus_ground_4class_v2_rk3568_fp16_20260710.rknn" --input-bgrx-stream --width 1280 --height 720 --frames 0 --display 0 2>&1 | grep -E "^\[(stream|infer|npu|pipeline)\]"

echo
echo "===== 完成 ====="
