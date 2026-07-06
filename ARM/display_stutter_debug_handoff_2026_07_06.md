# 30fps 画面抽动排查交接记录（2026-07-06）

## 当前结论

这次现象已经从“上下画面撕裂”重新定位为“运动物体抽动/闪现”。目前最可靠的证据显示：

- 摄像头输入是稳定 30fps。
- FPGA 摄像头侧每帧内容在变化，没有相邻重复帧。
- ARM 侧按 `--wait-new-frame 1` 读取时，采集开始、DMA 完成、推送到显示管线的节奏都是稳定 33.33ms。
- 画面仍然抽动，所以问题大概率发生在“推给显示之后的实际呈现”，或者发生在“ARM 读到的 DDR 帧身份/稳定性”这个更靠后的边界。
- `--display-do-timestamp 0` 的手动时间戳实验仍然没有画面，不能作为修复方向；默认自动时间戳模式仍然保留。

不要继续把主要怀疑点放在“摄像头有没有 30fps”上。这个问题已经被板端计数和 hash 证实过。

## 使用过的仓库

### ARM/Linux 侧

路径：

```text
/home/wzzz/VHDL_Project/ARM
```

父仓库：

```text
/home/wzzz/VHDL_Project
```

分支：

```text
yuv422-yuyv-bgrx
```

这里包含：

- `pplcnet_bgp_live`
- `fpga_hdmi_display`
- `fpga_drm_display`
- `pcie_fpga_dma.c/.h`
- ARM 侧 GStreamer/KMS 显示代码
- ARM 侧 FPGA 状态 ioctl 读取代码

最近相关提交：

```text
b61a87b Fix manual appsrc timestamp startup
c2e5f5a Add manual appsrc timestamp mode
b4ecd71 Pipeline direct DRM page flips
07938e9 Add direct DRM display diagnostic
bd4de1a Report live frame interval jitter
0b9337e Track camera input hash cadence
ab98cb1 Expose camera input cadence diagnostics
9724c59 Revert "Add sampled live frame hash diagnostic"
b599178 Add sampled live frame hash diagnostic
4f5b291 Delay BAR0 status mux select
5f72a8c Report frame status probe details
8cb164d Expose FPGA frame counter for live pacing
```

当前 ARM 仓库中，源码已提交；剩余未跟踪文件主要是旧文档和本地诊断二进制，不应自动提交。

### 正式 FPGA 工程

路径：

```text
C:\Users\Wzzz2\OneDrive\Desktop\project_test
```

WSL 路径：

```text
/mnt/c/Users/Wzzz2/OneDrive/Desktop/project_test
```

分支：

```text
codex-crosswalk-cv-test
```

最近相关提交：

```text
41e7dd7 Expose camera input cadence status
745f378 Delay BAR0 status mux select
6c23aa5 Expose FPGA frame counter status
a8af013 Revert "Add rd_buf stable bank selection test"
fbff36f Revert "Read stable DDR bank for PCIe snapshots"
```

注意：正式 FPGA 工程当前有大量既有脏工作区改动。不要用“整目录复制粘贴”的方式同步 ARM/Linux 侧改动，也不要默认这些脏改动都是本次排查产生的。需要合并时应基于 git diff/commit 精确挑选。

### 板端运行目录

板端路径：

```text
/home/linaro/ARM
```

板端运行时，用户从本机复制 `pplcnet_bgp_live`、`pcie_fpga_dma.ko`、bitstream 等产物过去。板端没有用于正式编译。

## 现象演化

最初怀疑是“画面撕裂”或“上下半屏错位”。后来用户补充：即使只有下半屏物体运动，也会抽动，因此不是典型上下半屏撕裂。

当前更准确的描述是：

- 30fps 时运动物体出现抽动/闪现。
- 25fps 抽动减弱。
- 15fps 通常明显更好，但中间也出现过一次不稳定观察；最终应以带状态计数的测试为准。
- `--display-every 2` 没有改善，甚至更严重。
- `--dma-pre-delay-us` 增加延迟没有本质改善。
- `--display-sync 0/1` 没有解决抽动。
- 旧的 `fpga_hdmi_display` 和新的 `pplcnet_bgp_live` 都能看到抽动，因此问题不只属于 `pplcnet_bgp_live` 的业务逻辑。

## 已经探索过的方向

### 1. 文件 dump 和离线帧差分析

曾尝试：

```bash
sudo ./pplcnet_bgp_live --no-infer --no-display --fps 30 --frames 300 --dump-frames 300 --dump-path /tmp/d30
python3 analyze_frame_diff.py --pattern '/tmp/d30/frame_*.bgrx'
```

问题：

- `/tmp` 空间不够，后面大量 dump 文件为 0 字节。
- 中间出现过半截文件和 0 字节文件。
- 这个方向不能作为最终证据。

结论：dump 分析只适合少量帧或换大空间路径；当时那批结果不能证明根因。

### 2. sampled/full/exact frame hash

曾经看到过类似：

```text
mode=sampled frames=600 adjacent_duplicates=176~179
mode=fast-full frames=600 adjacent_duplicates=100~106
mode=xxh64-full frames=600 adjacent_duplicates=100~102
mode=exact-adjacent frames=600 adjacent_duplicates=138
```

这些结果一度让人怀疑“读到重复帧”。但后来发现这些 hash 模式受 ARM 读取节奏、是否等待 FPGA 新帧、以及读同一 DDR 快照的影响。它们只能说明“ARM 当时读到的帧可能重复”，不能说明“摄像头输入重复”。

后续加入 FPGA 摄像头状态后，摄像头侧 hash 证明没有相邻重复帧，所以不应继续把这些早期 sampled/full hash 当成摄像头问题证据。

### 3. FPGA frame-status / wait-new-frame

加入过 FPGA 帧状态 ioctl，让 ARM 能等待 FPGA 新帧：

```bash
sudo ./pplcnet_bgp_live --no-infer --no-display --fps 30 --frames 10 --hash-frames 10 --hash-full --wait-new-frame 1
```

早期遇到：

```text
--wait-new-frame requested but FPGA frame status is unavailable
frame-status probe raw ... magic=0xdcba9876 expected=0x46505331
```

原因不是功能本身坏了，而是板端 bitstream/ko/app 没完全同步，BAR 状态读到的是旧 layout 或错误区域。重新同步后状态正常：

```text
frame-status counter=3 changes=2818 flags=0x00000001 magic=0x46505331
```

结论：`wait-new-frame` 机制已经能用，但它只证明“FPGA 有新帧状态”，不能单独证明“ARM 读出的 DDR 帧一定是稳定完整的新帧”。

### 4. 摄像头输入节奏和内容 hash

这是目前最重要的证据。

典型命令：

```bash
sudo ./pplcnet_bgp_live --no-infer --no-display --fps 30 --frames 300 --wait-new-frame 1
```

典型结果：

```text
camera-status frames=13303 lines=720 words=921600 hash=...
camera-status summary: frames_delta=300 elapsed_ms=10000.1 camera_fps=30.00
camera-hash summary: samples=300 adjacent_duplicates=0 longest_run=1 counter_nonunit_steps=0 duplicate_ratio=0.0%
```

这说明：

- 摄像头确实是 30fps。
- 每帧 720 行。
- 每帧 921600 words。
- 摄像头侧连续 300 帧没有相邻重复 hash。
- 摄像头帧 counter 没有跳步。

结论：摄像头输入不是根因。

### 5. ARM 采集、DMA、显示推送节奏

典型命令：

```bash
sudo ./pplcnet_bgp_live --no-infer --fps 30 --frames 300 --wait-new-frame 1
```

或：

```bash
sudo ./pplcnet_bgp_live --no-infer --fps 30 --frames 300 --wait-new-frame 1 --display-sync 1
```

典型结果：

```text
capture-start interval summary: samples=299 avg_ms=33.33 min_ms=33.18 max_ms=33.56 over40ms=0 over50ms=0
dma-done interval summary: samples=299 avg_ms=33.33 min_ms=32.96 max_ms=33.82 over40ms=0 over50ms=0
display-push interval summary: samples=299 avg_ms=33.33 min_ms=32.91 max_ms=33.87 over40ms=0 over50ms=0
```

这说明：

- ARM 主循环没有明显周期性卡顿。
- DMA 完成时间没有大抖动。
- 推送给显示管线的时间没有大抖动。

结论：如果用户仍然看到抽动，问题发生在 `display-push` 之后，或者 `display-push` 前读到的“帧身份/内容稳定性”仍有问题但未被现有 timing 统计捕获。

### 6. GStreamer 显示路径

`pplcnet_bgp_live` 使用：

```text
appsrc BGRx 1280x720 -> kmssink
```

测试过：

- `display_sync=0`
- `display_sync=1`
- `display_do_timestamp=1`
- `display_do_timestamp=0`
- `display_every=2`
- 不同 `dma-pre-delay-us`

当前结果：

- 自动时间戳模式有画面，但仍抽动。
- 手动时间戳模式没有画面，即使后来把第一帧 PTS 往后推也仍无画面。
- `display_every=2` 更糟，不是修复方向。

结论：手动 PTS 方向目前是失败诊断，不应作为默认设置。下一步如果继续查 GStreamer，应做更直接的“纯软件 synthetic 30fps moving pattern”测试，而不是继续调真实 DMA 输入。

### 7. 旧显示程序 fpga_hdmi_display

用户跑过：

```bash
sudo ./fpga_hdmi_display --fps 30 --display-sync 1 --io-mode mmap --mmap-mode staged --copy-buffers 6 --queue-depth 1
```

实际启动日志里仍有默认：

```text
release_delay_ms=20
```

因为用户第一次命令换行后，`--release-delay-ms 0 --swap16 0` 没有真正进入程序。

现象：旧程序也抽动。

结论：问题不是 `pplcnet_bgp_live` 的检测/分类/业务叠框逻辑引入的；更可能在共同的显示链路、DDR 读帧身份、或 KMS/HDMI 呈现边界。

### 8. 直接 DRM 诊断程序

新增过 `fpga_drm_display`，目的是绕过 GStreamer，直接 DRM page flip。

典型命令：

```bash
sudo ./fpga_drm_display --fps 30 --frames 300 --wait-new-frame 1
sudo ./fpga_drm_display --fps 60 --frames 600 --wait-new-frame 0
```

观察：

- 第一版把等待新帧和等待 page flip 串行化，导致接近 50ms，不可靠。
- 后来改成三缓冲流水 page flip。
- 仍看到 37~40ms 级别的 read/page-flip 间隔，画面仍抽动。

当前解释：

- 这个工具每帧要把 1280x720x4 的 3.6MB 数据拷进 DRM dumb buffer。
- 在板端这个 copy/read 路径本身可能成为瓶颈。
- 因此它不能证明“DRM/KMS 一定有问题”，只能证明“当前 direct DRM 诊断程序还不够干净”。

下一步如果继续 direct DRM，应先做不读 FPGA 的 synthetic page-flip 测试，确认 30/60fps 纯本地 buffer 能否平滑显示。

## 目前较可信的排除项

可以先排除或降级怀疑：

- 摄像头不是 30fps。
- 摄像头重复帧。
- 推理负载导致卡顿，因为 `--no-infer` 仍复现。
- ARM 主循环 30fps sleep 不稳定。
- DMA 完成节奏大幅抖动。
- `display_every` 能修复问题。
- 简单增加 `dma-pre-delay-us` 能修复问题。
- 手动 appsrc PTS 当前能修复问题。

## 仍然值得查的方向

### 方向 A：纯软件显示链路是否本身抽动

写一个不读 FPGA、不走 DMA 的测试：CPU 生成 1280x720 BGRx moving bar，以 30fps 推到同一个 `lpr_display` / `appsrc -> kmssink`。

判断：

- 如果 synthetic moving bar 也抽动，优先查 GStreamer/KMS/HDMI/timestamp/60Hz 呈现。
- 如果 synthetic moving bar 平滑，优先查 FPGA DDR 读出帧身份或 DMA 数据内容。

这是下一步最高性价比的隔离测试。

### 方向 B：更干净的 direct DRM synthetic 测试

写一个不读 FPGA 的 direct DRM 程序，只在 DRM buffer 中移动色块并 page flip。

判断：

- 如果 synthetic DRM 平滑，当前 `fpga_drm_display` 的抽动主要来自 FPGA read/copy 路径，不是 DRM 呈现。
- 如果 synthetic DRM 也抽动，查 KMS mode、HDMI 输出、60Hz/30Hz 节奏、page flip 时序。

### 方向 C：DDR 读出帧身份/稳定 bank

`wait-new-frame` 只能说明有新帧，不保证 ARM 读出的 DDR 区域是稳定 bank，也不保证读的时候不会跨到另一个帧身份。

建议增加更强的硬件/软件证据：

- FPGA 在每个写入 DDR 的视频帧里打 frame id，或者在 BAR 暴露当前写 bank、稳定读 bank、最近完成 bank。
- ARM 在 DMA 前后读取 frame id/status，确认一次 DMA 读的是同一个完整帧。
- 统计是否有 frame id 重复、跳变、DMA 读前后 bank 改变。
- 如果能在画面角落叠加 FPGA 原始 frame id，更容易肉眼确认抽动时是“显示重复/跳帧”还是“内容源本身闪变”。

### 方向 D：GStreamer 实际呈现时序

现在只统计到 `gst_app_src_push_buffer()` 返回时间，还没有统计 kmssink 实际 page flip / render 时间。

可继续查：

- appsrc 是否丢帧。
- queue 是否 leaky drop。
- kmssink 是否 late drop。
- pipeline 是否按 60Hz 呈现每个 30fps 帧两次。
- `do-timestamp`、PTS、duration、base-time 是否和 30fps 匹配。
- 是否需要关闭/打开 atomic flip。

但在继续调参数前，最好先完成方向 A 的 synthetic appsrc 测试。

## 不建议继续投入的方向

- 不建议继续反复跑旧的 sampled/full hash 来证明摄像头问题。
- 不建议继续只改 `--dma-pre-delay-us`。
- 不建议继续用 `/tmp` 大量 dump 300 帧，除非确认空间足够。
- 不建议把 Windows 正式 FPGA 工程用整目录复制覆盖。
- 不建议把手动 PTS 模式作为默认修复；当前它仍无画面。

## 当前推荐复现命令

验证摄像头输入：

```bash
sudo ./pplcnet_bgp_live --no-infer --no-display --fps 30 --frames 300 --wait-new-frame 1
```

验证实际显示抽动：

```bash
sudo ./pplcnet_bgp_live --no-infer --fps 30 --frames 300 --wait-new-frame 1 --display-sync 1
```

手动 PTS 诊断，目前预期仍失败或无画面：

```bash
sudo ./pplcnet_bgp_live --no-infer --fps 30 --frames 300 --wait-new-frame 1 --display-sync 1 --display-do-timestamp 0
```

不要把这个命令作为修复验证，只能作为“手动 PTS 分支目前不可用”的证据。

## 给下一个 agent 的重点提醒

1. 先相信已经拿到的板端证据：camera 30fps 稳定，camera hash 无重复。
2. 不要从“摄像头到底有没有 30fps”重新开始。
3. 不要混淆两个仓库：ARM/Linux 侧和正式 FPGA 工程不是同一个工作区。
4. Windows 正式 FPGA 工程有大量既有脏改动，合并前必须审查 diff。
5. 现在最应该做的是隔离“显示链路本身”与“FPGA/DDR/DMA 输入内容”：
   - 先做纯软件 appsrc moving bar。
   - 再做纯软件 direct DRM moving bar。
   - 然后才回到 DDR stable bank / frame id。
6. 如果 synthetic appsrc 平滑，而真实 DMA 抽动，优先查 DDR 读帧身份和稳定 bank。
7. 如果 synthetic appsrc 也抽动，优先查 GStreamer/KMS/HDMI 呈现节奏。
