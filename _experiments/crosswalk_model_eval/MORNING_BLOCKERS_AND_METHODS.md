# 上午斑马线/场景识别实验碰壁点与方法整理

## 1. 背景目标

目标是在现有 ARM 独立测试模块里，复用已经可用的行人 YOLO RKNN，只新增斑马线、正常路面、人行道区域识别，然后用行人框脚点判断是否横穿马路。

约束：

- 不影响现有车牌识别主路径。
- 不改 `fpga_lpr_display.c`、OCR、车牌分类、AE 反馈逻辑。
- 优先做独立测试程序 `crosswalk_ped_cv_test`。
- 先验证传统 CV 和现成 crosswalk 模型是否可用，再决定是否训练。

## 2. 上午主要实验方法

### 2.1 独立 ARM 测试模块

已实现独立程序：

- `ARM/crosswalk_ped_cv_test.cpp`
- `ARM/run_crosswalk_cv_test.sh`

功能：

- 从 `/dev/fpga_dma0` 读实时帧。
- 复用 `--ped-model /userdata/model/best_noquant.rknn` 行人模型。
- 运行 OpenCV 场景检测。
- 显示 road/crosswalk/person/foot/crossing overlay。
- 新增 debug dump：
  - 原始图：`raw_*.ppm`
  - 带框图：`overlay_*.ppm`
  - 坐标与状态：`meta_*.txt`

关键修复：

- 板端日志显示行人模型输出为 `dims=1,5,8400`，这是单类别 `[x,y,w,h,conf]` 输出。
- 旧解码逻辑按多类别 YOLO 输出处理，可能导致行人框不可靠。
- 已修成兼容 `stride=5` 单类别输出。
- 建议板端 labels 使用一行：

```bash
echo "person" > /userdata/model/person_labels.txt
```

### 2.2 QA dump 样本

QA 目录：

```text
C:\Users\Wzzz2\OneDrive\Desktop\QA\cross_walk_det
```

样本定义：

- 正样本，有路口/斑马线/行人：`20,30,40,50,60,90,100,110,120,130`
- 负样本，暗室/桌面/无上图噪声：`0,10,70,80`
- 另有参考图：`微信图片_20260609110003_298_54.jpg`

检查方式：

- 读取所有 `meta_*.txt`。
- 将 `raw_*.ppm`、`overlay_*.ppm` 转成 PNG contact sheet。
- 对比原图、ARM CV overlay、行人框、脚点和 region 状态。

## 3. 传统 CV 碰壁点

### 3.1 原始传统 CV 问题

原始流程：

```text
灰度/增强 -> 白色阈值 -> 形态学 -> Canny -> HoughLinesP -> 平行线聚类 -> minAreaRect
```

实际问题：

- `crosswalk polygon` 经常被撑成半屏或整屏。
- `road polygon` 被屏幕边框、桥底线、手机边缘线拉歪。
- `sidewalk = 非 road 且非 crosswalk` 的推导导致大量未知区域被误判成人行道。
- 斑马线和人行道/路面重叠，行为判断自然失真。

典型原因：

- 当前 QA 图不是干净路口摄像头，而是板端拍到屏幕/手机里的路口图。
- 有手机边框、屏幕亮边、反光、摩尔纹、桥底横线、树丛、伞、自行车遮挡。
- 传统 CV 只看颜色、亮度、线段，无法理解 road/sidewalk/crosswalk 的语义。

### 3.2 传统 CV 保守修正

为了避免严重误报，改成“宁可 LOST/UNK，不要整屏误判”的策略：

- 加 active-scene 门控：
  - 平均灰度低于阈值，或亮像素比例过低，直接清空 scene。
  - 暗室/桌面帧不再 HOLD 上一次 ROI。
- 斑马线检测只在下半区找：
  - 低饱和、高亮的白色候选。
  - 近水平 Hough 线段。
  - 多条平行条纹局部聚类。
- road polygon 加面积/宽度限制。
- 如果 road 不可靠，脚点默认 `UNK`，不再默认 `SIDEWALK`。

本地 QA 原型结果：

- 负样本 `0,10,70,80` 能被 active-scene 门控拦住。
- 正样本能给出较小 crosswalk 候选，但仍然会偏大、吃进路面/等待区。
- 微信原图上当前算法能找到大致区域，但框偏大；手工理想框也不容易画准。

结论：

```text
传统 CV 可作为门控/辅助/调试提示，不适合做最终 scene 真值。
```

## 4. 现成 CDNet / YOLOv5 crosswalk 模型实验

### 4.1 实验目录

```text
_experiments/crosswalk_model_eval/
```

产物：

- `CDNET_EVAL_REPORT.md`
- `contact_positive.png`
- `contact_negative.png`
- `contact_wechat.png`
- `output_overlays/yolov5s_crosswalk/`
- `output_overlays/yolov5s_crosswalk_lowconf/`

### 4.2 检查的仓库和权重

1. 原始 CDNet：

```text
zhangzhengde0225/CDNet
commit: 6ffc9a6
```

问题：

- 官方权重在百度网盘。
- WSL 环境无法直接下载，需要中文手机/账号验证。
- 没找到可靠公共镜像。

2. 可用替代权重：

```text
WangRongsheng/CDNet-yolov5
```

拿到两个 CDSet 训练权重：

| 权重 | 大小 | 类别 | 说明 |
|---|---:|---|---|
| `yolov5n_crosswalk.pt` | 3.7 MB | crosswalk, guide_arrows | YOLOv5n |
| `yolov5s_crosswalk.pt` | 13.7 MB | crosswalk, guide_arrows | YOLOv5s |

训练集是 CDSet，主要是车载前视斑马线。

### 4.3 推理方法

输入：

- 将 QA 目录里的 `raw_*.ppm` 转成 PNG。
- 包含 10 张正样本、4 张负样本、1 张微信参考图。

阈值：

- 标准阈值：`conf_thres=0.25`
- 极低分析阈值：`conf_thres=0.001`

目的：

- 标准阈值看是否实际可用。
- 极低阈值看模型是否至少“有一点点响应”。

### 4.4 推理结果

标准阈值 `0.25`：

```text
全部 15 张图零检测。
```

极低阈值 `0.001`：

- 正样本最高置信度约 `0.012`。
- 大多数框是图像顶部、底部、手机边缘、屏幕边框条带。
- 微信参考图无检测。
- 负样本 `70` 有一个低置信边缘伪框。

这说明：

```text
模型不是阈值调得太高，而是目标域上基本没有有效响应。
```

## 5. 模型实验的审查结论

员工实验主结论可以采纳：

```text
CDSet 训练的现成 YOLOv5 crosswalk 权重不能直接用于当前 QA 域，不值得现在转 RKNN。
```

但报告有不严谨点：

- 把 QA 图称为真实 overhead surveillance 不严谨。
  - 当前 QA 更准确说法是“板端拍到的屏幕/手机显示路口图”。
  - 这比正常比赛摄像头还多了一层域偏移。
- 把 ARM CV overlay 当 ground truth 不严谨。
  - 这些 overlay 本身就是不稳定算法输出，不是真实标注。
- 报告有编码乱码，交付质量需要修正。
- 只重点列了 YOLOv5s，YOLOv5n 逐图结果没有同等完整展开。

不影响主判断：

```text
现成 CDSet 权重在当前 QA 图上失败。
```

## 6. 为什么现成模型如此不行

根因不是模型太小，而是域偏移：

### 6.1 视角域偏移

CDSet / CDNet：

- 车载前视。
- 斑马线通常在画面下方。
- 斑马线呈透视梯形。
- 背景是车辆前方路面。

当前 QA：

- 屏幕/手机显示的路口画面。
- 斑马线更像高位/侧向视角的白色条纹块。
- 有屏幕边框、反光、模糊、摩尔纹。
- 行人、伞、自行车、树丛遮挡严重。

模型学到的不只是“白色条纹”，还学到了位置、透视、背景和标注习惯。

### 6.2 目标定义不一致

不同数据集可能会：

- 框整片斑马线。
- 框每条白线。
- 框可通行区域。
- 标注 guide arrow 等道路标志。

我们的需求是：

```text
能用于脚点判断的 crosswalk ROI。
```

这和普通检测框不完全一致。

### 6.3 官方没有测试风格

比赛官方没有训练集、没有测试风格，不能押注单一专用模型。

专用模型在自己训练域里很强，跨域可能直接失效。

## 7. 上午形成的工程判断

### 7.1 不建议继续深调传统 CV 做主方案

传统 CV 现在能做：

- 判断是否有有效路口画面。
- 给出粗候选。
- 作为 debug overlay。

不适合做：

- 稳定斑马线真值。
- 稳定 road/sidewalk 语义区域。
- 最终横穿行为判定依据。

### 7.2 不建议直接转 CDSet YOLOv5 权重为 RKNN

原因：

- 当前 QA 正样本几乎零置信。
- 转 RKNN 只会把失败模型搬上板。
- 量化还可能进一步降低弱响应。

### 7.3 最现实路线

如果比赛机位固定或可人工初始化：

```text
固定/半自动标定 scene polygon + 行人 YOLO 脚点轨迹判断
```

这是当前最稳路径。

如果必须自动识别：

```text
收集接近比赛视角的图片 -> 标 crosswalk bbox/seg -> 微调 YOLOv5n/YOLOv8n -> 再转 RKNN
```

不要用 CDSet 原始权重直接上板。

## 8. 下一步建议

### 8.1 快速可行性 probe

准备 20-50 张接近比赛风格的图，手工标注：

- `crosswalk` bbox
- 可选：`road` / `sidewalk` polygon 或 bbox

用 `yolov5n_crosswalk.pt` 或 COCO YOLOv5n 做短微调。

判断：

- 如果 20-50 张就能明显收敛，继续扩到 200-500 张。
- 如果仍然不收敛，换分割方案或固定场景标定。

### 8.2 运行时策略

推荐最终系统做成分层兜底：

```text
行人 YOLO 每帧运行
crosswalk 模型低频运行或初始化运行
scene config / 手工 polygon 优先级高于弱模型输出
模型低置信时显示 SCENE_UNKNOWN，不输出强判定
```

### 8.3 当前代码上的方向

保留：

- `debug dump`
- `stride=5` 单类别行人模型解码
- active-scene 门控

弱化：

- 传统 CV 自动 crosswalk/road/sidewalk 作为最终判断依据

增强：

- 增加可配置 `scene polygon` 文件输入。
- 允许人工标定 crosswalk/road/sidewalk。
- 后续再接入训练后的 crosswalk RKNN。
