#!/bin/bash
# 编译并运行 rga_test：验证 librga 能否把 1080p NV12 硬件转换成 720p BGRx。
# 用法：bash rga_test_run.sh [视频路径]
set -u
cd "$(dirname "$0")"
V="${1:-/mnt/sdcard/test.mp4}"

echo "===== 定位 librga 头文件 ====="
HDR=""
for p in /usr/include/rga /usr/include/rockchip/rga /usr/include; do
  for h in im2d.h RgaUtils.h rga.h; do
    if [ -f "$p/$h" ]; then HDR="$HDR -I$p"; fi
  done
done
echo "头文件目录: $HDR"

echo
echo "===== 编译 rga_test ====="
gcc -Wall -O2 -std=c11 $HDR -o rga_test rga_test.c -lrga -lm
[ -x ./rga_test ] || { echo "编译失败"; exit 1; }
echo "编译成功: rga_test"

echo
echo "===== 测速：硬解 NV12 -> rga_test(BGRx) -> 丢到 /dev/null ====="
echo "跑 12 秒后自动停，看 stderr 的 [rga] ... fps 行"
timeout 14 bash -c "
  gst-launch-1.0 -q filesrc location='$V' ! decodebin ! \
    video/x-raw,format=NV12,width=1920,height=1080 ! fdsink fd=1 sync=false 2>/dev/null | \
  ./rga_test > /dev/null
" 2>rga_err.txt
cat rga_err.txt | grep -E "^\[rga\]|failed|error|Error" | head -20

echo
echo "===== 对照：纯硬解吐 NV12 到 /dev/null（看解码本身多快）====="
N=$(timeout 12 gst-launch-1.0 -q filesrc location="$V" ! decodebin ! video/x-raw,format=NV12,width=1920,height=1080 ! fdsink fd=1 sync=false 2>/dev/null | wc -c)
awk -v b="$N" -v f=3110400 -v t=12 'BEGIN{printf "  纯解码吐NV12 -> %.1f fps\n", b/f/t}'

echo
echo "===== 完成 ====="
echo "如果上面 [rga] 显示 >=30 fps，说明 RGA 硬件路径可用，可放心改 driver。"
echo "完整 err 输出见 rga_err.txt"
