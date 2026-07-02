# V6 Baseline Manifest

- Snapshot time: 2026-07-02 (captured from 2026-06-05 18:42 synthesis run)
- Source timing report: `report_timing/pcie_dma_ddr3_cam1.rtr`
- Source run log: `report_timing/run.log`
- Source place&route log: `place_route/run.log`
- Source clock utilization: `place_route/clock_utilization.txt`
- Tool: Fabric Compiler 2022.2-SP6.4 (build 146967)
- Top module: `pcie_dma_ddr3_cam1`
- Device: Logos2 PG2L50H FBG484 -6
- Latest bitstream: `generate_bitstream/pcie_dma_ddr3_cam1_30fps_4.sfc` (2026-06-05 18:49)

## Convergence Status

- `multiseed_summary.csv`: `Convergence=OK`, single seed, 100% pass rate
- **All clock domains Setup/Hold MET, TNS=0, no failing endpoints**
- `place_route/run.log`: "No hold violation."

## Key Metrics (Slow Corner)

| Clock | Freq (MHz) | Setup WNS (ns) | Hold WHS (ns) |
|---|---|---|---|
| pclk | 250.00 | 0.991 | 0.241 |
| pclk_div2 | 125.00 | 0.663 | 0.143 |
| ref_clk | 100.00 | 5.644 | 0.148 |
| cmos1_pclk | 84.03 | 3.332 | 0.105 |
| cfg_clk | 37.13 | 24.231 | 0.342 |
| ddr_rst_clk | 25.00 | 36.704 | 0.220 |
| ddrphy_sysclk | 131.25 | 1.919 | 0.144 |

- Worst Setup WNS: `0.991` (pclk -> pclk)
- Worst Hold WHS: `0.105` (cmos1_pclk -> cmos1_pclk)
- All TNS / THS = 0; all failing endpoints = 0

## Resource Utilization

| Resource | Used | Available | Utilization |
|---|---|---|---|
| LUT | 6,130 | 26,300 | 24% |
| FF | 6,793 | 52,600 | 13% |
| LUT-FF pairs | 3,482 | 26,300 | 14% |

## Warnings

- Timing-4086 (unconstrained input): 19 — DDR3 `mem_dq` PHY pins, normal (PHY-calibrated internally)
- Timing-4087 (unconstrained output): 58 — DDR3 `mem_*` / `cmos1_scl` / `cmos_reset` PHY pins, normal
- No Timing-4003 / SDC / ConstraintEditor warnings (all 0)

## Comparison vs Prior Baselines

| Baseline | Date | pclk WNS | pclk_div2 WNS | ddrphy_sysclk WNS | Convergence |
|---|---|---|---|---|---|
| v5.1 (seed208) | 2026-03-18 | 0.349 | 0.462 | 2.143 | partial |
| v5.1_p2full | 2026-03-18 | -0.029 | -2.184 | -0.201 | Fail |
| **v6 (this)** | **2026-06-05** | **0.991** | **0.663** | **1.919** | **OK** |

The v6 baseline supersedes all prior baselines. Timing closure is fully achieved; the historical "Convergence=Fail / timing not converged" references in `Docs/` have been corrected on 2026-07-02.

## Files

- `BASELINE_MANIFEST.md` — this file
- `pcie_dma_ddr3_cam1.rtr` — full timing report (binary-rich)
- `run.log` — report_timing run log
- `timing_summary.txt` — extracted human-readable summary
- `clock_utilization.txt` — clock region / buffer utilization
- `warning_counts.txt` — warning code counts
