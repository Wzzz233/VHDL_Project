# 当前状态汇总（2026-03-16）

## 结论
- OV5640 彩条异常（细密白条纹）问题已修复（你已反馈“修好了”）。
- 修复采用“最小改动”策略：仅回补 OV5640 初始化安全链路，不改动 Prep 去除方向。

## 版本与分支
- 当前分支：`raw-baseline-current-worktree-20260315`
- 当前提交：`3605620`
- 上游分支：`origin/raw-baseline-current-worktree-20260315`
- 推送状态：已 push 到远端

## 本次修复提交
- Commit: `3605620` - `Restore OV5640 init safety path for raw baseline`
- 涉及文件：
  - `ARM/audit_reg_config.py`
  - `hdl/i2c_com.v`
  - `hdl/reg_config.v`

### 修复要点
- `reg_config` 恢复初始化安全逻辑：
  - `initial_en` 门控生效
  - ACK 失败不再盲目前进
  - soft reset 后保留 settle delay
  - 稀疏寄存器表安全跳转，避免默认伪写
- `i2c_com` 恢复主线稳定实现（不依赖未跟踪的 `iic_dri.v`）。
- `audit_reg_config` 与主线规则对齐，用于持续审计配置可信度。

## 验证结果
- 软件审计命令：
  - `python ARM/audit_reg_config.py --file hdl/reg_config.v --strict`
- 审计结果：
  - `trusted_for_iq_analysis=YES`
  - 退出码 `0`
- 板级结果：
  - 你已确认“修好了”。

## 当前工作区状态（提交后）
- 已跟踪但未提交改动：`3` 个
  - `fdc/pcie_dma_test.fdc`
  - `hdl/pcie_dma_ddr3_cam1.v`
  - `impl.tcl`
- 未跟踪项：`102` 个（主要为文档、仿真目录、临时文件）

## 备注
- 本次提交刻意未包含 `hdl/pcie_dma_ddr3_cam1.v` 当前工作区改动，以避免把未定版实验改动混入 OV5640 初始化修复提交。
