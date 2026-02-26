# PG2L50H + RK3568 摄像头到 HDMI 全链路技术文档（现网实现详解）

## 1. 文档目的与范围

本文档基于当前仓库代码，完整说明以下链路在“现有实现”中的工作方式：

`OV5640 -> FPGA采集与格式化 -> DDR3帧缓存 -> PCIe DMA(MWR) -> RK3568 DMA缓冲 -> 用户态颜色转换 -> DRM/KMS HDMI显示`

本文档强调“按当前代码事实描述”，不假设未来功能。

- FPGA 顶层实现：`hdl/pcie_dma_ddr3_cam1.v`
- Linux 驱动：`ARM/pcie_fpga_dma.c` + `ARM/pcie_fpga_dma.h`
- HDMI 显示应用：`ARM/fpga_hdmi_display.c`
- 启动脚本：`ARM/run_hdmi_kms.sh`

---

## 2. 当前工程状态（与链路直接相关）

### 2.1 综合目标

当前综合 top module 为 `pcie_dma_ddr3_cam1`。

- 见：`multiseed_summary.csv`
- 见：`impl.tcl` 中多次 `compile -top_module pcie_dma_ddr3_cam1`

### 2.2 链路是否闭环

从代码层面，链路是闭环的：

1. 相机初始化与采集已接入 FPGA。
2. 采集数据写入 DDR3 帧缓存。
3. DMA 控制器通过 BAR2 外部读接口从帧缓存读数据并发起 MWR。
4. RK3568 驱动通过 BAR1 寄存器触发 DMA 到主机内存。
5. 用户态程序读取帧、转换到 `BGRx`，送 `kmssink` 输出 HDMI。

### 2.3 时序收敛状态（实现风险）

当前 `multiseed_summary.csv` 显示 `Convergence=Fail`，说明“功能代码实现完成”与“P&R 时序完全收敛”是两件事。

---

## 3. 顶层硬件架构（FPGA）

顶层文件：`hdl/pcie_dma_ddr3_cam1.v`

### 3.1 外设接口

- PCIe x2：`rxp/rxn/txp/txn`
- DDR3 物理接口：`mem_*`
- 相机 OV5640：
- I2C：`cmos1_scl/cmos1_sda`
- 像素流：`cmos1_pclk/cmos1_vsync/cmos1_href/cmos1_data[7:0]`
- 相机复位：`cmos_reset`

### 3.2 顶层主要模块

1. PCIe 子系统
- `pcie_test`（PCIe IP wrapper）
- `ips2l_pcie_dma`（DMA 控制与 TLP TX/RX）

2. Camera 采集链
- `power_on_delay`
- `reg_config`
- `cmos_8_16bit`

3. 帧缓存链
- `fram_buf`
- `DDR3_50H`

4. DMA-Frame 桥接
- 使用 `ips2l_pcie_dma` 的 BAR2 外部读覆盖接口：
- `o_bar2_rd_clk_en_ext`
- `o_bar2_rd_addr_ext`
- `i_ext_bar2_rd_data`

---

## 4. 时钟与复位树

### 4.1 时钟来源

- `sys_clk`：板级输入（注释标明 25MHz）
- `PLL` 生成：
- `cfg_clk`：配置/延时逻辑时钟
- `ddr_clk`：DDR 参考时钟
- PCIe IP 导出：`pclk` / `pclk_div2`
- 相机像素时钟：`cmos1_pclk`

### 4.2 复位关键路径

1. `pll_locked` 后，`rstn_1ms` 计数到 `16'h2710`，得到 `ddr_rstn`。
2. `ddr_rstn` 驱动 `power_on_delay`，产生：
- `camera_rstn`
- `initial_en`
3. `reg_config` 完成后输出 `cmos1_init_done`。
4. `cmos_8_16bit` 使用 `cmos1_init_done` 作为复位释放条件。
5. `fram_buf` 使用 `ddr_init_done` 作为 DDR 域复位。

说明：链路采用多时钟域结构，关键 CDC 在 `wr_buf/rd_buf` 内完成（双口 RAM + 同步寄存）。

---

## 5. Camera 上电与 I2C 初始化

### 5.1 `power_on_delay` 的三段时序

文件：`hdl/power_on_delay.v`

三段延时逻辑：

1. 电源稳定到 `PWDN` 拉低
2. `PWDN` 拉低到 `RESETB` 拉高
3. `RESETB` 拉高到允许 SCCB/I2C 初始化（`initial_en`）

输出：`camera1_rstn/camera_pwnd/initial_en`。

### 5.2 `reg_config` 的寄存器写入机制

文件：`hdl/reg_config.v`

- 使用 20kHz I2C 控制时钟（由 25MHz 分频）。
- 通过 `i2c_com` 按序写寄存器表（`reg_index < 358`）。
- 设备地址：`0x78`（OV5640 8-bit I2C 地址写格式）。
- 完成后拉高 `reg_conf_done`（顶层命名为 `cmos1_init_done`）。

### 5.3 与 720p 输出相关的关键寄存器

`reg_config.v` 中配置了 1280x720 输出窗口和时序参数，典型包括：

- `0x3808/0x3809` 输出宽度
- `0x380A/0x380B` 输出高度
- `0x380C/0x380D` HTS
- `0x380E/0x380F` VTS
- `0x4300=0x60` 输出格式路径设置
- `0x503D` 可选测试图（由 `SENSOR_TEST_PATTERN_EN` 控制）

顶层默认：`SENSOR_TEST_PATTERN_EN = 1'b0`。

---

## 6. 像素采集与格式化

### 6.1 `cmos_8_16bit` 功能

文件：`hdl/cmos_8_16bit.v`

输入：
- `pdata_i[7:0]`
- `de_i`（href）
- `vs_i`（vsync）
- `pclk`

输出：
- `pdata_o[15:0]`
- `pix_vld_o`（16bit 像素有效脉冲）
- `de_o` / `vs_o`
- `pixel_clk`（内部翻转分频输出）

工作方式：
- 每两个 8-bit 字节拼成一个 16-bit 像素。
- 在 `line_start/frame_start` 时重置 byte 相位。

### 6.2 RGB/BGR次序控制

顶层 `pcie_dma_ddr3_cam1.v` 内：

- `CAM_SWAP_RB` 控制是否交换 R/B 位置。
- 默认 `CAM_SWAP_RB = 0`，即直接使用 `cmos1_d_16bit`。

### 6.3 预DDR/后DDR调试图案开关

顶层内置两个开关（默认都关闭）：

- `FORCE_COLOR_BAR_PRE_DDR`：写 DDR 前强制色条
- `FORCE_PATTERN_POST_DDR`：DMA读出侧强制色条

这两个开关用于链路分段定位：

1. 先验证 DDR+DMA+显示链路
2. 再回到真实 camera 数据

---

## 7. DDR3 帧缓存架构

核心文件：`hdl/fram_buf.v`、`hdl/wr_buf.v`、`hdl/rd_buf.v`

### 7.1 基本参数

- 分辨率：`H_NUM=1280`, `V_NUM=720`
- 像素位宽：`PIX_WIDTH=16`（RGB565/BGR565）
- DDR 用户数据宽度：`MEM_DQ_WIDTH=16`，AXI 数据通路为 `MEM_DQ_WIDTH*8 = 128bit`

### 7.2 写路径（Camera -> DDR）

1. `wr_buf` 在 `wr_clk=cmos1_pclk` 域接收像素。
2. 16-bit 像素两拍拼成 32-bit 写入 `wr_fram_buf`（行缓存双口 RAM）。
3. 在 `ddr_clk` 域读行缓存，组装 DDR 写命令与写数据。
4. `wr_rd_ctrl_top` 负责 AXI 写事务发往 DDR3 控制器。

### 7.3 读路径（DDR -> DMA 供数）

1. `rd_buf` 在 `ddr_clk` 域发起 DDR 读请求并写入 `rd_fram_buf`。
2. 在 `vout_clk=pclk_div2` 域按 `rd_en` 递增地址读 `rd_fram_buf`。
3. `vout_data` 为 128-bit 宽（直接供给 DMA BAR2 外部读数据）。

### 7.4 三帧 bank 轮转机制

`fram_buf.v` 中通过 `frame_wcnt mod 3` 计算 `frame_widx`，并生成 `wr_frame_base`。

- 写入 bank：`frame_widx`
- 读出 bank：`rd_buf` 锁定“上一完整帧 bank”

`rd_buf.v` 逻辑：

- 当写 frame index 为 0 时，读 2
- 当写 frame index 为 1 时，读 0
- 当写 frame index 为 2 时，读 1

目的：避免读写同一帧 bank，减少 tearing/半帧读取风险。

### 7.5 帧读会话控制

顶层定义：

- `FRAME_WORDS = (1280*720*16)/128 = 115200`（128-bit words/frame）
- `mwr_cmd_start` 触发 `dma_session_start`
- `mwr_rd_clk_en` 在会话期间推进读地址
- 达到 `FRAME_WORDS` 后会话结束

并通过 `rd_fsync_stretch_cnt` 在读会话开始阶段拉出 `rd_fsync` 脉冲，驱动 `rd_buf` 重置到新帧起点。

---

## 8. PCIe DMA 通路与寄存器协议

### 8.1 BAR1 控制寄存器定义

驱动头文件：`ARM/pcie_fpga_dma.h`

- `0x100` `BAR1_DMA_CMD_REG`
- `0x110` `BAR1_DMA_L_ADDR`
- `0x120` `BAR1_DMA_H_ADDR`

命令位：

- `[9:0]` 长度（DW-1）
- `[16]` 64-bit 地址使能
- `[24]` 方向：1=MWR（FPGA写主机）

### 8.2 FPGA 侧对 BAR1 的解析

文件：`hdl/pcie_dma_ctrl/ips2l_pcie_dma_controller.v`

- 监听 BAR1 写入地址 `0x100/0x110/0x120`
- 组装 `o_req_length/o_req_addr`
- 产生 `mwr64_req`（满足配置完成且无4KB越界）
- `o_tx_restart` 在写 `0x110` 时拉高（顶层接成 `mwr_cmd_start`）

### 8.3 BAR2 数据源切换

文件：`hdl/pcie_dma_ctrl/ips2l_pcie_dma.v`

- `bar2_rd_data = i_ext_bar2_rd_sel ? i_ext_bar2_rd_data : bar2_rd_data_int`
- 顶层固定 `i_ext_bar2_rd_sel=1'b1`

即：DMA 发包 payload 来自外部注入 `frame_rd_data`，不是内部 BAR2 RAM。

### 8.4 4KB 边界规则

DMA 控制器内部做跨 4KB 边界检测；Linux 驱动也做分块规避。

双重防护可以降低 TLP 构造非法或中断风险。

---

## 9. RK3568 内核驱动数据面

文件：`ARM/pcie_fpga_dma.c`

### 9.1 Probe 初始化

1. `pci_enable_device`
2. 打开 bus master
3. 设置 64-bit DMA mask（失败则回退32-bit）
4. 映射 BAR0/BAR1
5. `dma_alloc_coherent` 分配帧 DMA 缓冲
6. 建立字符设备 `/dev/fpga_dma0`

### 9.2 IOCTL 接口

- `FPGA_DMA_GET_INFO`
- `FPGA_DMA_READ_FRAME`
- `FPGA_DMA_MAP_BUFFER`

### 9.3 DMA 传输函数 `fpga_dma_perform_transfer`

核心行为：

1. 将请求帧按 `DMA_MAX_LEN_BYTES=2048` 分块。
2. 每块写 BAR1 地址寄存器与 CMD 寄存器。
3. 使用尾哨兵轮询判断块传输完成。
4. 可配置块间 `dma_chunk_delay_us`。

对 720p RGB565 一帧：

- `FPGA_FRAME_SIZE = 1280*720*2 = 1,843,200 bytes`
- 2048B/块 -> 正好 `900` 块。

### 9.4 mmap 模式

`FPGA_DMA_MAP_BUFFER` + `dma_mmap_coherent` 支持用户态直接只读映射驱动 DMA buffer。

---

## 10. 用户态 HDMI 显示面

文件：`ARM/fpga_hdmi_display.c`

### 10.1 运行流程

1. 打开 `/dev/fpga_dma0`
2. `FPGA_DMA_GET_INFO` 校验帧格式（必须 1280x720x2）
3. 选择 `io-mode`：
- `mmap`：映射驱动 buffer
- `copy`：每帧由 ioctl 拷贝到用户 buffer
4. 建立 GStreamer pipeline：
- `appsrc -> queue(leaky) -> kmssink`
5. 主循环：
- 触发 `FPGA_DMA_READ_FRAME`
- 取源帧（mmap/copy）
- `convert_frame_to_bgrx`
- push 到 `appsrc`

### 10.2 像素格式处理

输入：16-bit 565。

参数：

- `--pixel-order bgr565|rgb565`
- `--swap16 0|1`

输出：`BGRx`（4 bytes/pixel）供 `video/x-raw, format=BGRx`。

### 10.3 低时延策略

- `appsrc block=false`
- `queue leaky=downstream`
- queue 深度限制

目标是“实时性优先，允许丢旧帧”。

### 10.4 脚本默认参数注意项

`run_hdmi_kms.sh` 默认：

- `PIXEL_ORDER=bgr565`
- `SWAP16=1`
- `IO_MODE=copy`

而应用内部默认 `io-mode=mmap`，脚本会显式覆盖为 `copy`。

---

## 11. 端到端时序（单帧）

下面是一次完整“采集帧 -> HDMI显示”的实际时序抽象：

1. 上电后，`power_on_delay` 达成 `camera_rstn` 与 `initial_en`。
2. `reg_config` 通过 I2C 完成 OV5640 表配置，输出 `cmos1_init_done`。
3. 相机输出 `pclk/vsync/href/data[7:0]`。
4. `cmos_8_16bit` 拼成 16-bit 像素与 `pix_vld`。
5. `wr_buf` 将像素打包并经 AXI 写入 DDR3。
6. 主机驱动发起 DMA：写 BAR1 `0x110/0x120/0x100`。
7. FPGA DMA 控制器发起 MWR，读取 BAR2 外部数据源（帧缓存读出）。
8. 帧数据写入 RK3568 coherent DMA buffer。
9. 用户态应用取帧、16bit->BGRx 转换。
10. GStreamer `kmssink` 送入 DRM/KMS，最终 HDMI 出图。

---

## 12. 数据格式与带宽核算

### 12.1 帧格式

- Camera/DDR/PCIe payload：16-bit/pixel（565）
- HDMI appsrc 输入：`BGRx`（32-bit/pixel）

### 12.2 单帧大小

- 原始帧：1,843,200 B
- BGRx 显示帧：3,686,400 B

### 12.3 典型速率（仅做量纲评估）

以 10fps（显示程序默认）计：

- FPGA->Host 原始流：约 17.6 MB/s
- 用户态 BGRx 输出数据面：约 35.2 MB/s

---

## 13. 现状中的关键设计点（工程上值得关注）

### 13.1 读写解耦

三帧 bank + 行缓存双口 RAM，使 camera 写入与 DMA 读出在不同域并行，降低直接冲突概率。

### 13.2 DMA 会话化

`mwr_cmd_start` 触发会话、会话内按 `mwr_rd_clk_en` 持续消费一整帧，减少“块级命令间断导致读指针漂移”的风险。

### 13.3 双侧4KB保护

- 驱动分块规避
- 控制器越界检测

利于稳定性和问题定位。

### 13.4 显示端容错

像素次序与字节序均可运行时切换（`pixel-order` + `swap16`），便于适配 sensor 输出与历史位序差异。

---

## 14. 已知风险与建议排查方向

### 14.1 P&R 时序未收敛

现有报告显示 Convergence Fail，需继续做约束/布局布线优化，否则存在板上偶发不稳定可能。

### 14.2 Camera 初始化表维护风险

`reg_config.v` 为硬编码表，维护成本高。建议将关键寄存器分组并做自动一致性检查（仓库已有 `audit_reg_config.py`）。

### 14.3 `initial_en` 使用一致性

顶层连接了 `initial_en` 到 `reg_config`，但 `reg_config` 状态机当前主要基于 `camera_rstn` 和 `reg_conf_done` 驱动，建议保持信号语义一致，避免后续误判。

### 14.4 应用层开销

当前 `fpga_hdmi_display` 每帧做 565->BGRx CPU 转换。若追求更高 fps，可评估：

- 零拷贝显示路径进一步优化
- GPU/RGA 格式转换
- 降低不必要 memcpy

---

## 15. 关键源码索引（建议阅读顺序）

1. `hdl/pcie_dma_ddr3_cam1.v`
2. `hdl/cmos_8_16bit.v`
3. `hdl/reg_config.v`
4. `hdl/power_on_delay.v`
5. `hdl/fram_buf.v`
6. `hdl/wr_buf.v`
7. `hdl/rd_buf.v`
8. `hdl/pcie_dma_ctrl/ips2l_pcie_dma.v`
9. `hdl/pcie_dma_ctrl/ips2l_pcie_dma_controller.v`
10. `ARM/pcie_fpga_dma.h`
11. `ARM/pcie_fpga_dma.c`
12. `ARM/fpga_hdmi_display.c`
13. `ARM/run_hdmi_kms.sh`

---

## 16. 总结

当前仓库已经具备一条可运行的“相机采集到 HDMI 显示”全链路实现，核心特点是：

- FPGA 侧完成采集、缓存、DMA供数。
- RK3568 侧完成 DMA触发、帧提取、显示输出。
- 工程上已经有较完整的调试与诊断抓手。

后续工程重点不在“链路是否存在”，而在“时序收敛、初始化质量一致性、以及性能优化”。
