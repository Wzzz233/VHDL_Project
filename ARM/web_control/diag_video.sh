#!/bin/bash
# 视频播放流畅度诊断脚本。用法：bash diag_video.sh [视频路径]
# 默认视频 /mnt/sdcard/test.mp4，默认 driver/model 路径按板子实际部署。
set -u
V="${1:-/mnt/sdcard/test.mp4}"
D="${DRIVER:-/home/linaro/ARM/cplus-rk3568-driver}"
M="${MODELS:-/userdata/model}"

echo "视频: $V"
echo "driver: $D"
echo "models: $M"
[ -f "$V" ] || { echo "视频文件不存在: $V"; exit 1; }
[ -x "$D" ] || { echo "driver 不可执行: $D"; exit 1; }

echo
echo "===== 1. 编码信息 ====="
gst-discoverer-1.0 "$V" 2>&1 | grep -iE "H\.26|Width|Height|framerate"

echo
echo "===== 2A. 单线程 videoconvert（跑15秒后 Ctrl+C，看 average framerate）====="
gst-launch-1.0 -q filesrc location="$V" ! decodebin ! videoconvert ! \
  videoscale add-borders=true ! \
  video/x-raw,format=BGRx,width=1280,height=720 ! fakesink sync=false

echo
echo "===== 2B. 多线程 videoconvert n-threads=4（跑15秒后 Ctrl+C）====="
gst-launch-1.0 -q filesrc location="$V" ! decodebin ! videoconvert n-threads=4 ! \
  videoscale add-borders=true ! \
  video/x-raw,format=BGRx,width=1280,height=720 ! fakesink sync=false

echo
echo "===== 3. 完整管线 + driver 日志（先在网页停掉实时车牌释放 NPU，再回车）====="
read -r -p "按回车继续..." _
gst-launch-1.0 -q filesrc location="$V" ! decodebin ! \
  capsfilter caps=video/x-raw ! identity sync=true ! \
  queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! \
  videoconvert ! videoscale add-borders=true ! \
  video/x-raw,format=BGRx,width=1280,height=720 ! \
  queue max-size-buffers=1 max-size-bytes=0 max-size-time=0 leaky=downstream ! \
  fdsink fd=1 sync=false | \
"$D" \
  --det-model "$M/yolov5nu_coco_rk3568_fp16_20260710.rknn" \
  --seg-model "$M/mapillary_cplus_ground_4class_v2_rk3568_fp16_20260710.rknn" \
  --input-bgrx-stream --width 1280 --height 720 --frames 0 \
  --display 0 2>&1 | grep -E "^\[(stream|infer|pipeline|npu)\]"

echo
echo "===== 诊断完成 ====="
