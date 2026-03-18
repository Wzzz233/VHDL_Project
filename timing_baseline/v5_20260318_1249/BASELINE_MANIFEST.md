# V5 Baseline Manifest

- Base timing report: `report_timing/pcie_dma_ddr3_cam1.rtr`
- Base run logs: `run.log`, `report_timing/run.log`
- P4 gate: `IP_DDR_PHY ratio > 80%` in top50 slow-max

## Current extracted metrics
- pclk_div2 setup target baseline: WNS `-2.205`, TNS `-310.833`, failing endpoints `282`
- Top50 slow-max IP ratio: from `top50_path_classification.txt`
- Severe warning counts: from `warning_counts.txt`
- Warning summary status matrix: from `warning_status_matrix.txt`

## Board regression placeholders
- 300-frame CSV file: `capture_300_frames.csv`
- Fields: frame_idx,bytes,crc32_hex,nonzero_bytes,nonzero_ratio
