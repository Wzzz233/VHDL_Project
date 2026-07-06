# 30fps 画面抽动排查交接(2026-07-06,第二轮)

本文档接续 `display_stutter_debug_handoff_2026_07_06.md`(第一轮,ChatGPT 所写)。
第一轮的结论大部分仍然有效,但本轮有重大进展:**问题已经分裂成两个独立的、都有实锤的子问题**,
并且已经做好了下一步的诊断工具(帧号像素戳),等待板测。

## 一、当前最重要的结论(先读这个)

### 问题 A:FPGA DDR 读出约 11% 陈旧帧 —— RTL 真 bug,必须修

决定性证据(本轮板测实测):

```text
# 带显示:
frame-hash summary: mode=strong-full frames=600 adjacent_duplicates=66 duplicate_ratio=11.0% read_fps=9.41
# 不带显示(--no-display):
frame-hash summary: mode=strong-full frames=600 adjacent_duplicates=71 duplicate_ratio=11.9% read_fps=9.56
```

关键推理链,不要推翻:

1. `--hash-full` 的 3.6MB 全帧哈希把主循环拖慢到 ~9.5fps(每帧 ~106ms ≈ 3 个相机帧周期)。
2. 相机侧已证明每帧内容都不同(camera-status 硬件 hash,300 帧 0 重复)。
3. 因此两次相隔 3 帧的 DMA 读回,内容本应永远不同,但实测 11% 完全一致(逐字节)。
4. `--wait-new-frame 1` 开着也一样 → 不是 ARM 读太快,不是等待机制失效。
5. `--no-display` 结果相同 → 与显示路径无关。

结论:**FPGA 内部"DDR 写入→3-bank 轮转→PCIe 读出"链路,有 ~11% 的概率把陈旧内容端给 ARM。**

这也完美解释了原始现象梯度:30fps 读取时每次只隔 1 个相机帧,陈旧内容直接表现为重复/回跳帧
→ 运动物体抽动;读得越慢,陈旧 bank 有更多机会被覆写 → 25fps 减轻、15fps 基本正常。
用户看到的"加了 hash-full 就不抽了"就是同一规律:那不是修复,是把读取拖慢到了 9.5fps。

### 问题 B:显示链路 —— 硬件已排除,残留问题在软件时间戳/渲染,优先级低

本轮排除工作:

- `modetest -M rockchip -s 175:1280x720-60 -v` → freq 稳定 60.00Hz。KMS/VOP2/HDMI 驱动、翻页时序全部健康。
- modetest 期间"画面和 logo/黑屏交替闪"是良性的:modetest -v 在两个 buffer 间翻页,第二个 buffer
  内容是残留 logo 或黑,不是 HDMI 失锁(失锁是秒级黑屏+屏自身 OSD,不是 60Hz 快闪)。
- 这块 HDMI 屏原生模式是 1024x600@59.82(`type: preferred`),720p60 输入要经屏内部缩放,
  且 59.82Hz 与 30fps 有慢拍频;屏质量一般,但不是主因。
- 板上有两个 connector 同时 connected:DSI-1(720x1280)和 HDMI-A-1(id=175)。
  以后测试建议显式指定 connector(kmssink `connector-id=175`)。
- `videotestsrc 30fps ball + kmssink sync=true` → basesink 报"buffers being dropped / too late"
  丢帧风暴。说明 kmssink 渲染耗时超过 PTS 预算。这同时解释了第一轮"手动 PTS 模式黑屏"之谜
  (PTS 稍早 → 全部判迟到 → 全丢 → 黑屏),不是玄学。
- 已知代码瑕疵(未修,低优先级):`ARM/lpr_live/lpr_display.c` 中 `GST_BUFFER_PTS(buf) = d->next_pts_ns`
  是无条件执行的,自动时间戳模式下 buffer 带着从 0 开始的假 PTS 入管线,依赖 appsrc do-timestamp
  覆盖。若板上 GStreamer 行为变化会出问题。修法:自动模式下设 `GST_CLOCK_TIME_NONE`。

**先修问题 A。问题 A 修好后再回来看画面是否还抽,再决定是否投入问题 B。**

### 一次性事件(已解决,勿再排查)

用户报告过一次"完全不显示",后来自行恢复,判断为 HDMI 连接/环境偶发。若再遇到:
先 `ps aux | grep -E "fpga_drm|gst"` 查残留进程占 DRM master,再 `fuser -v /dev/dri/card0`。

## 二、已就绪待板测:帧号像素戳(本轮新做的诊断工具)

hash 只能证明"重复",分不清读侧重复/写侧未更新/撕裂混 bank。帧号戳可以。已实现并提交:

### FPGA 侧

- 仓库:`C:\Users\Wzzz2\OneDrive\Desktop\project_test`(WSL: `/mnt/c/Users/Wzzz2/OneDrive/Desktop/project_test`)
- 分支 `codex-crosswalk-cv-test`,提交 **09af72e**,只改 `hdl/wr_buf.v`(+19/-1)
- 逻辑:wr_clk 域 8 位计数器 `stamp_frame_cnt`,每个 wr_fsync 上升沿 +1;
  写 DDR 时把**第一行和最后一行的前 8 个像素**替换为 BGRX `{X=0xA5, R=cnt, G=~cnt, B=0x5A}`。
  只影响 PIX_WIDTH=32 分支(本工程实际使用的分支)。
- 已用 iverilog 行为仿真验证 PASS(3 帧,戳记位置/递增/其余像素零污染)。
- **注意**:该仓库工作区有大量既有脏文件(不是本次产生的),合并/checkout 时只碰目标文件。
  另外仓库行尾是 CRLF 敏感的:上一轮曾把 fram_buf.v 的行尾状态备份进 stash@{0},勿 pop。
  编辑 hdl 下文件时保持原文件行尾格式(wr_buf.v 是 LF)。

### ARM/Linux 侧

- 仓库:`/home/wzzz/VHDL_Project`,分支 `yuv422-yuyv-bgrx`,提交 **611d4b7**
- 改动:`ARM/lpr_live/lpr_common.h`(选项字段)、`ARM/pplcnet_bgp_live.c`(校验+统计)、
  `hdl/wr_buf.v`(镜像同步)
- 新增 `--frame-stamp-check <0|1>`:每次 DMA 读回后解析首/末行戳记
  (解析函数 `frame_stamp_extract()`,BGRX 字节序:p[0]=B=0x5A, p[1]=G=~cnt, p[2]=R=cnt, p[3]=X=0xA5),
  统计并打印:
  - `malformed`:戳记格式不对(位流没烧对/没同步)
  - `torn`:首行与末行帧号不一致(一次 DMA 读跨了两帧/两个 bank)
  - `duplicates`:与上次读回帧号相同(整帧重复)
  - `backward`:帧号回退(读到更旧 bank;用 8 位环绕差 delta>=128 判定)
  - `skips/skipped_frames`:跳号(正常丢帧,30fps 相机 vs 更慢的读取时预期非零)
  - 每类前 8~12 条打印带帧号明细,结尾打 `frame-stamp summary` 一行
- 交叉编译已通过(零警告),产物已拷到 `/home/wzzz/ARM/pplcnet_bgp_live`。
  编译命令(WSL 本机,不需要 Docker):

```bash
cd /home/wzzz/VHDL_Project/ARM
make pplcnet-bgp-live \
  SDK_SYSROOT=/home/wzzz/RK3568J_SDK/debian/binary \
  RKNN_RUNTIME_DIR=/home/wzzz/RK3568J_SDK/external/rknpu2/runtime/Linux/librknn_api
```

### 板测步骤(下一步就做这个)

1. 用包含 09af72e 的 FPGA 工程重新综合、布局布线、烧录。**ko 不用重编**(戳记走像素数据,
   不改寄存器布局;当前 BAR0 状态寄存器 magic 机制不变)。
2. 拷 `/home/wzzz/ARM/pplcnet_bgp_live` 到板端 `/home/linaro/ARM`。
3. 跑(注意:不开 hash-full,戳记校验是 O(1),就是要在真实 30fps 节奏下测):

```bash
sudo ./pplcnet_bgp_live --no-infer --no-display --fps 30 --frames 600 \
  --wait-new-frame 1 --frame-stamp-check 1
```

4. 判读 `frame-stamp summary`:

| 现象 | 结论 | 下一步 |
|---|---|---|
| malformed 全命中 | 位流是旧的 | 检查烧录/综合是否包含 09af72e |
| duplicates ≈ 11%,torn≈0 | DMA 把同一帧完整端两次:**读侧** bug | 查 rd_buf 会话 bank 锁定(见下方"读侧嫌疑点") |
| torn > 0 | 一次读跨帧:读会话期间 bank 不稳定 | 查 rd_buf wr_rst/锁定时序 |
| backward > 0 | 读到更旧的 bank | 查 locked_frame_idx 选择逻辑 |
| 全部 ≈ 0 但 hash-full 复测仍 11% 重复 | **写侧** bug:相机帧没写进 DDR 但 fsync 计数照走(写路径欠载/静默丢写) | 查 wr_buf→wr_rd_ctrl_top→DDR 写链路,注意绿线问题的旧根因(读路径欠载)可能有写侧同款 |
| duplicates 明细有周期性(等间隔) | 节拍/相位类 bug | 对比周期与 fsync/读取节奏的拍频 |
| duplicates 随机分布 | 带宽/欠载类 | 查 DDR 仲裁、读写冲突 |

另外建议同时带显示跑一次 30fps 看画面,把 stamp 统计和主观抽动对上。

## 三、读侧嫌疑点(如果判读指向读侧,从这里开始)

以下是本轮代码审查已确认的机制,给后续排查省时间:

- `frame_wcnt`(= wr_buf.rd_frame_cnt,`hdl/wr_buf.v`)在 wr_fsync 上升沿(ddr_clk 域)递增,
  与写 bank 轮转 `frame_widx = mod3(frame_wcnt)`(`hdl/fram_buf.v:125`)同源。计数变化 =
  写方切到新 bank = 上一帧 bank 可读。该机制本身自洽,已审查无误。
- rd_buf(`hdl/rd_buf.v:241-261`)在每次读会话的 rd_fsync 沿把 `locked_frame_idx` 锁为
  `i_wr_frame_idx - 1 (mod 3)`,即"上一个完成的 bank",整个会话期间不变。
- 一次 ioctl 读整帧虽然分 ~900 个 4KB chunk,但 FPGA 侧
  `dma_session_start = mwr_cmd_start & ~dma_session_active`(`hdl/pcie_dma_ddr3_cam1.v:1230`)
  保证 rd_fsync 只在首 chunk 触发一次,整帧一个会话。理论安全窗口 ~2 帧(66ms)。
- **重点疑问**:上面每一环"看起来"都对,但 11% 陈旧帧是事实。可疑方向:
  1. `i_wr_frame_idx` 传到 rd_buf 是 ddr_clk 域信号,rd_fsync 是从 pclk_div2 域 stretch 过来的
     (rd_fsync_stretch_cnt),锁定时刻采样 i_wr_frame_idx 是否有 CDC 采样错位,导致偶发锁到
     "当前正在写的 bank 的再上一个"之外的错误 bank?
  2. rd_buf 内部 `wr_fsync_*` 信号名实际接的是 rd_fsync(命名混乱,rd_buf.v:121),
     排查时别被名字骗了。
  3. 历史线索:a8af013/fbff36f revert 过"Read stable DDR bank for PCIe snapshots"和
     "rd_buf stable bank selection test"——之前有人动过这块又撤了,看这两个 revert 的
     diff 可能有线索。
- BAR0 状态寄存器(0xFF0,magic "FPS1"/camera "CAM1")机制已审查正确且 CDC 安全
  (change_count 用 1-bit wirq 边沿累加)。**word0 frame_counter 是 8-bit 二进制直接 2FF 同步,
  有多 bit CDC 危险,只可用于日志,勿用于 pacing/判断。**

## 四、仓库/环境速查

| 项 | 值 |
|---|---|
| Linux 仓库 | `/home/wzzz/VHDL_Project`,分支 `yuv422-yuyv-bgrx`,HEAD=611d4b7 |
| FPGA 正式工程 | `/mnt/c/Users/Wzzz2/OneDrive/Desktop/project_test`,分支 `codex-crosswalk-cv-test`,HEAD=09af72e |
| 板端 | `/home/linaro/ARM`,产物从本机拷贝,板端不编译 |
| 本机编译产物输出 | `/home/wzzz/ARM/`(pplcnet_bgp_live 已是新版) |
| HDMI connector id | 175(1280x720-60 可用;屏原生 1024x600@59.82) |
| 主诊断命令 | 见上文板测步骤 3 |
| FPGA 语法/仿真检查 | WSL 有 iverilog;wr_buf 单测的 testbench 在 /tmp/stamp_tb/(临时目录,重启会丢,必要时重写:喂 3 帧小分辨率,检查首末行前 8 像素戳记) |

## 五、给下一个 agent 的注意事项

1. 问题 A(FPGA 11% 陈旧帧)是主线,问题 B(显示软件时间戳)先放着。
2. 不要再跑 hash-full 当修复手段——它只是把读取拖慢了。诊断用 `--frame-stamp-check 1`。
3. 不要再怀疑相机、ARM 主循环节奏、wait-new-frame 机制,这三者已被反复证实无辜。
4. FPGA 工程行尾敏感(CRLF 混杂),编辑时保持目标文件原行尾;工作区脏文件不是你产生的,别动。
5. `--fps 30` + `--wait-new-frame 1` 联用有双时钟拍频(软件 30Hz vs 相机 30Hz 漂移),
   正式跑建议 `--fps 60 --wait-new-frame 1` 让相机单独定节奏;但诊断 stamp 时 `--fps 30` 没问题
   (等待机制保证每次读之间有新 fsync)。
6. 回退方式:两侧各 revert 一个提交即可(09af72e / 611d4b7)。
