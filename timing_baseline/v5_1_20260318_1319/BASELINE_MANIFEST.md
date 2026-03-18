# V5.1 Baseline Manifest

- Snapshot time: 2026-03-18T15:01:02
- Source timing report: `C:/Users/Wzzz2/OneDrive/Desktop/project_test/report_timing/bak/pcie_dma_ddr3_cam1.rtr`
- Source run log: `C:/Users/Wzzz2/OneDrive/Desktop/project_test/report_timing/run.log`

## Key Metrics
- pclk->pclk WNS: `-0.029`
- ddrphy_sysclk->ddrphy_sysclk WNS: `-0.201`
- pclk_div2->pclk_div2 WNS/TNS/FE: `-2.184` / `-305.899` / `282`
- ref_clk->pclk_div2 FE: `47`
- pclk_div2->ref_clk FE: `5`

## Gates
- FDC parser/import gate total: `0`
- Patterns: Timing-4003, CommandTiming-0057, SDC-2023, ConstraintEditor-0046, SDC-2017

## Top50 Ownership
- IP_OWNED ratio: `47/50`

## Board Regression Placeholder
- `capture_300_frames.csv` retained as baseline comparison placeholder.

## Execution Outputs
- `results/p1_multiseed_summary.csv`: P1 5-seed implementation-only sweep
- `results/p2_multiseed_summary.csv`: P2 5-seed candidate sweep
- `results/V5_1_EXECUTION_REPORT.md`: keep/revert decision and gate conclusion
