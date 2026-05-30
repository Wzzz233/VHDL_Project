# 数据链路与CLAHE实验分析报告

日期: 2026-05-30
作者: 链路分析

---

## 一、全链路数据流

### 1.1 物理连接

```
OV5640 (DVP接口)
  ├─ PCLK, VSYNC, HREF, DATA[7:0]  →  FPGA  (PG2L50H)
  ├─ SDA/SCL (I2C)                  →  FPGA  (引脚F16/U20)
  │                                    ↑ 仅通过 reg_config.v 初始化时配置
  └─ RESET/PWDN                     →  FPGA  (power_on_delay控制)

RK3568
  ├─ PCIe x2 (ref_clk, rx/tx)       ↔  FPGA  (DMA数据传输 + BAR寄存器)
  ├─ I2C-0 (/dev/i2c-0)             →  FPGA   (FPGA内部I2C slave, 地址0x66)
  │                                    ↑ 仅控制FPGA内部寄存器(LED等)，不连OV5640
  └─ DRM/KMS                        →  HDMI  (显示输出)
```

**关键结论：** OV5640 的 I2C 只连 FPGA，不连 RK3568。RK3568 无法直接或通过透传修改 OV5640 寄存器——当前 FPGA 设计中无运行时 I2C 透传机制。

### 1.2 数据格式流转

| 阶段 | 格式 | 大小 | 说明 |
|------|------|------|------|
| OV5640 输出 | YUV422 YUYV | 8-bit DVP @ 84MHz | 0x4300=0x30, 720p 30fps |
| cmos_8_16bit 打包 | YUYV 16-bit | 每PCLK 2拍拼1字 | pixel_clk = pclk/2 = 42MHz |
| YUV→BGRX 转换 | BGRX8888 32-bit | BT.601 full-range | `pack_2pix_yuv_to_bgrx()` 硬件函数 |
| DDR3 帧缓存 | BGRX8888 32-bit | 3-bank轮转, PIX_WIDTH=32 | fram_buf 三帧防tearing |
| PCIe MWR → RK3568 | BGRX8888 32-bit | 230,400 words × 128-bit | 3,686,400 B/frame |
| 显示路径 | BGRX→BGR565→kmssink | BGR565 16-bit | GStreamer pipeline |
| 推理路径 | BGRX→RGB888→RKNN | RGB888 24-bit | YOLO + LPRNet |

### 1.3 YUV→BGRX 转换详情

BT.601 全范围公式（FPGA 硬件实现）：

```
du = U - 128
dv = V - 128
 R = Y + (359 × dv) >> 8    // 1.402 × dv
 G = Y - (88×du + 183×dv)>>8  // -0.344×du - 0.714×dv
 B = Y + (454 × du) >> 8     // 1.772 × du
```

每周期处理2个Y像素（共享1对UV），输出2个BGRX像素（64-bit）。

---

## 二、ARM 侧处理 Pipeline

### 2.1 推理线程流程

```
┌──────────┐   ┌──────────┐   ┌──────────┐   ┌──────────┐   ┌──────────┐
│ 帧捕获   │──▶│ YOLOv8   │──▶│ 车牌检测  │──▶│ LPRNet   │──▶│ 叠加绘制 │
│ DMA读取  │   │ 车辆检测  │   │ +过滤+稳定 │   │ OCR识别  │   │ BGR565   │
└──────────┘   └──────────┘   └──────────┘   └──────────┘   └──────────┘
                                    │
                                    ▼
                              ┌──────────┐
                              │ 裁剪+CLAHE│ ← (本次实验插入点)
                              │ →尺寸/    │
                              │ 清晰度检查 │
                              └──────────┘
```

### 2.2 OCR 专家路由

当前代码支持（保留的部分）：
- **blue** → 蓝牌 OCR 模型
- **green** → 绿牌 OCR 模型
- **yellow** → 黄牌 OCR 模型
- **special** → 特殊车牌 OCR 模型

已移除：police（警察）、embassy（使馆）路由。

### 2.3 PCIe DMA 驱动

- 设备: `/dev/fpga_dma0`
- 帧大小: 3,686,400 B (BGRX8888 模式)
- 传输: 分块 DMA, 2048 B/块
- IOCTL: `FPGA_DMA_READ_FRAME`, `FPGA_DMA_MAP_BUFFER`

---

## 三、CLAHE 实验

### 3.1 实验目的

验证在 LPRNet OCR 前插入 CLAHE（L通道自适应直方图均衡）是否能改善光照差条件下的车牌识别率。

### 3.2 实现

- 位置: `prepare_plate_crop_rgb888()` → re-crop逻辑 → **CLAHE** → 尺寸/清晰度检查 → LPRNet
- 算法: RGB→HSL → L通道CLAHE (tile=8, clip=2.0) → HSL→RGB
- 开关: `--clahe-enable <0|1>` / `--clahe-compare <0|1>`
- 对比模式: 每帧同时跑原图和CLAHE图的OCR，输出差异日志和PPM图片

### 3.3 实验结果

#### 3.3.1 正常光照
- CLAHE 效果: 几乎无变化（对比度已够）
- OCR 结果: 原图和CLAHE图均正确识别 "苏D3Q520"
- 结论: 正常光照不需要 CLAHE

#### 3.3.2 过曝
- CLAHE 效果: 无明显改善
- OCR 结果: 原图即可正确识别
- 结论: 过曝下CLAHE无帮助

#### 3.3.3 极度黑暗
- CLAHE 效果: 图像被暴力拉伸（L通道平均变化~150/255），字符失真、颜色偏移
- OCR 结果: 原模型几乎失能，CLAHE后更差（模型没见过这种增强图）
- 结论: 极端暗光下 CLAHE 不仅无效，反而有害

#### 3.3.4 量化数据

统计数据来自54对对比图片：

| 指标 | 值 |
|------|-----|
| 总样本对 | 54 |
| 最大L通道差异 | 228/255 |
| 平均L通道差异 | ~150/255 |
| 无效样本（CLAHE未执行，已修复bug） | ~30对 |
| 有效样本 | ~24对 |
| CLAHE改善OCR | 0 |
| CLAHE恶化OCR | 多数暗光样本 |
| CLAHE无影响 | 正常/过曝样本 |

### 3.4 结论

**瓶颈不在局部对比度，在曝光整体偏差。**

当前 CLAHE 参数 (clip_limit=2.0, tile_size=8) 对小尺寸车牌crop (~200×70) 过强。但即使调整参数，根本问题仍是 OV5640 的曝光（AEC）在暗场下未做调整——软件侧的任何对比度增强都无法弥补曝光不足，且会放大噪声。

### 3.5 下一阶段判断

| 判断 | 下一步 |
|------|--------|
| ✅ CLAHE 无改善 | 放弃软件增强方案 |
| ✅ 暗光下原模型失能 | 需要调整 OV5640 AEC 寄存器（0x3a0f/0x3a10）或增加补光 |
| ⚠️ FPGA无I2C透传 | 需在FPGA侧增加PCIe→I2C透传寄存器，或采用纯软件Gamma校正 |

---

## 四、I2C 路径详细分析

### 4.1 当前硬件拓扑

```
RK3568                         FPGA (PG2L50H)              OV5640
──────                         ──────────────              ──────
                    PCIe BAR → ips2l_pcie_dma      ←→ DDR3 帧缓存
                    (BAR0,1,2)      ↓
                                    ↓
                    I2C Master (i2c_com.v) ──── SDA/SCL ──→ I2C Slave
                    20kHz, 地址0x78                         (寄存器配置)
                                    ↑
                    reg_config.v ────┘
                    (只启动时跑一次)
```

### 4.2 关键发现

1. **FPGA 引脚**: `cmos1_sda` → F16, `cmos1_scl` → U20 (LVCMOS33)
2. **OV5640 I2C 地址**: 0x78 (8-bit写格式)
3. **FPGA I2C slave 地址**: 0x66（位于 `/dev/i2c-0`，仅控制FPGA内部寄存器如LED）
4. **无透传机制**: 当前FPGA设计中，RK3568无法在运行时通过PCIe或I2C向OV5640写寄存器

### 4.3 实现运行时AEC控制需要的改动

**方案A：FPGA加I2C透传寄存器（推荐）**
- 在PCIe BAR0或APB空间分配1个32-bit寄存器
- RK3568写入 → FPGA检测到BAR写事件 → 驱动 `i2c_com.v`（或新I2C控制器）→ OV5640
- 预计改动量: `reg_config.v` + `pcie_dma_ddr3_cam1.v` + PCIe BAR地址映射

**方案B：纯软件Gamma校正（不动FPGA）**
- 在CLAHE位置替换为自适应Gamma
- 公式: `L' = 255 × (L/255)^(1/gamma)`，`gamma = log(0.5)/log(mean_y/255)`
- 优点: 不改FPGA，今天可实现
- 缺点: 物理极限——纯提亮无法恢复信噪比

---

## 五、代码变更摘要

### 5.1 提交历史

| 提交 | 说明 |
|------|------|
| `a46f733` | 同步ARM驱动 feat/arm-bird-detection-driver |
| `6ee1f30` | 添加 CLAHE L-channel 增强 (首次) |
| `ddf5523` | 回退路由代码 (误删全部路由) |
| `998bc86` | 添加 --clahe-dump-dir (丢失后重加) |
| `3d749af` | 修复缺少的 long_opts 和 case handlers |
| `0dec745` | 修复compare模式不执行CLAHE的bug |
| `2c653b9` | 自动创建dump目录 |
| `e894ecf` | 编译错误修复 |
| `65a6fd6` | 移除鸟类识别 |
| `41601c4` | 补充缺失的 struct options 字段 |

### 5.2 增减行数统计

- `fpga_lpr_display.c`: ~+260 / ~-10
- `run_lpr_kms.sh`: ~+14 / ~-2
- 新增文件: `ocr_decode.c/h` (455行), `lpr_special_route.h` (56行), 模型文件
- 移除非警察/使馆路由: 约-200行

### 5.3 CLAHE 函数位置

```c
// fpga_lpr_display.c 约 line 3472
static void clahe_l_channel(uint8_t *rgb, int w, int h,
                            float clip_limit, int tile_size);
```

插入点（两处）:
1. `infer_thread_main()` - 主推理循环 (line ~7182)
2. `run_offline_once()` - 离线模式, `temporal_confirm_and_update` 调用 (line ~6653)

---

## 六、附录

### 6.1 OV5640 AEC 关键寄存器

| 寄存器 | 地址 | 默认值 | 功能 |
|--------|------|--------|------|
| AEC stable high | 0x3a0f | 0x30 | 自动曝光稳定区间上限 |
| AEC stable low | 0x3a10 | 0x28 | 自动曝光稳定区间下限 |
| AEC fast zone high | 0x3a11 | 0x60 | 快速收敛区上限 |
| AEC fast zone low | 0x3a1f | 0x14 | 快速收敛区下限 |
| Gain ceiling high | 0x3a18 | 0x00 | 增益上限高位 |
| Gain ceiling low | 0x3a19 | 0x80(当前) | 增益上限低位 (8x = 0x0080) |

### 6.2 CLAHE 测试命令

```bash
# 对比模式（每帧同时输出原图和CLAHE图OCR结果）
sudo ./run_lpr_kms.sh ${COMMON} --clahe-compare 1 --clahe-dump-dir /tmp/clahe_dump

# 仅CLAHE增强模式（不对比）
sudo ./run_lpr_kms.sh ${COMMON} --clahe-enable 1

# 输出文件格式
/tmp/clahe_dump/frame_{seq}_{idx}_orig.ppm
/tmp/clahe_dump/frame_{seq}_{idx}_clahe.ppm
```

### 6.3 日志关键行说明

```
[clahe-cmp] frame=349 orig="苏D3Q520" conf=0.90 clahe="苏D3Q520" conf=0.90
           ↑ 帧号    ↑ 原图OCR结果    ↑ 原图conf  ↑ CLAHE图结果   ↑ CLAHE conf
                                                                  ★ 表示两结果不同

[clahe-dump] frame=349 idx=42 w=201 h=67 dir=/tmp/clahe_dump
            ↑ 帧号    ↑ dump序号 ↑ 图片尺寸
```
