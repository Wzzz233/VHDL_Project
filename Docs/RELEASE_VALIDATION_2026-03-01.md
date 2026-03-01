# FPGA Camera-to-HDMI 发布回归记录

- 发布日期: 2026-03-01
- 分支: `main`
- 发布基线提交: `7aabbab`
- 发布标签: `v2026.03.01-stable-hotfix1`
- 记录人: 项目组

## 1. 发布结论

当前版本在板端已确认可稳定运行，`BGRX + mmap zero-copy` 路径可持续推流，未复现“运行数百帧后 `Frame-mode DMA timeout` 卡死”问题。

本次发布采用“稳定优先热修”策略：
- 保留 chunk 首拍步进修复（防止边界相位错位）。
- 回退 `slave2 tready` 为 raw 接线（规避长跑闭环阻塞）。

## 2. 关键提交范围

本次稳定化相关主提交：

1. `00ac6be` Restore frame-mode 1024DW chunk encoding in DMA controller
2. `58bb119` Decode zero length as 1024DW in tx_mwr_rd_ctrl
3. `1a03e8b` Unify 0=>1024DW decode and counters in rd_ctrl
4. `8dca710` Fix DMA chunk first-beat step and enable frame-ready backpressure
5. `7aabbab` Hotfix: use raw slave2 ready to avoid frame-mode timeout deadlock

## 3. 板端回归结果（最终状态）

- 驱动加载成功，`/dev/fpga_dma0` 创建设备正常。
- `run_hdmi_kms.sh` 在 `--io-mode mmap` 下持续运行稳定。
- 应用统计项 `cap/push` 连续增长，FPS 维持目标附近。
- 版本最终状态：系统可稳定运行（由板端实测确认）。

## 4. 验收清单（发布版）

### 4.1 主路径稳定性

```bash
sudo rmmod pcie_fpga_dma 2>/dev/null || true
sudo insmod pcie_fpga_dma.ko dma_pixel_format=1
sudo ./run_hdmi_kms.sh --fps 15 --io-mode mmap --queue-depth 1 --copy-buffers 2
```

验收标准：
- 连续运行不少于 10 分钟；
- 不出现 `Frame-mode DMA timeout`；
- 不出现 `FPGA_DMA_READ_FRAME failed: Connection timed out`。

### 4.2 中断连续性

```bash
cat /proc/interrupts | grep -Ei 'fpga|msi|0755' || true
```

验收标准：
- `fpga_dma` 中断计数持续增长。

### 4.3 对照路径

```bash
timeout 30s sudo ./run_hdmi_kms.sh --fps 15 --io-mode copy --queue-depth 2 --copy-buffers 3 || true
```

验收标准：
- copy 路径稳定，不出现超时退出。

### 4.4 格式回归

```bash
sudo rmmod pcie_fpga_dma
sudo insmod pcie_fpga_dma.ko dma_pixel_format=0
timeout 60s sudo ./run_hdmi_kms.sh --fps 15 --io-mode mmap --queue-depth 1 --copy-buffers 2 || true
```

验收标准：
- 无同类 DMA timeout。

### 4.5 日志检查

```bash
sudo dmesg -T | grep -Ei 'fpga_dma|timeout|irq|msi|dma' || true
```

验收标准：
- 无 `Frame-mode DMA timeout`。

## 5. 发布说明

- 本次发布不改变 Linux IOCTL/API、用户态参数、寄存器映射。
- 仅涉及 FPGA 顶层内部节拍与流控接线热修。
- 若后续出现轻微画面撕裂，将进入二阶段流控增强（带死锁破圈机制）版本。

