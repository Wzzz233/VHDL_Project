# Baseline Manifest (v3)

Date: 2026-03-18
Goal: Freeze known-good display baseline before further timing optimization.

## Frozen Artifacts
- Timing report source: `report_timing/pcie_dma_ddr3_cam1.rtr`
- Timing run log source: `report_timing/run.log`
- Snapshot summary: `report_timing/baseline_v3_20260318/timing_summary.txt`
- Top50 classification: `report_timing/baseline_v3_20260318/top50_path_classification.txt`
- YUV strict-equivalence proof: `report_timing/baseline_v3_20260318/yuv_equivalence.txt`

## Hardware Baseline (to be filled from board run)
- Bitstream path: `TODO_BY_USER`
- Frame capture count target: `300`
- Resolution: `TODO_BY_USER`
- Frame bytes: `TODO_BY_USER`
- CRC policy: per-frame CRC32
- Non-zero ratio policy: non_zero_bytes / total_bytes

## 300-frame Statistics Template
Store one row per frame in `capture_300_frames.csv` with columns:
`frame_idx,bytes,crc32_hex,nonzero_bytes,nonzero_ratio`

## Gate Reference
- P4 conservative trigger: if IP_DDR_PHY ratio in Top50 worst paths > 80%, stop RTL-only push and hand over IP regeneration checklist.
