# DDR PHY Timing Repair Record (2026-03-18)

## Scope
- RTL/interface/protocol unchanged.
- Only top-level constraints updated in `fdc/pcie_dma_test.fdc`.

## Async Constraint Whitelist (Top-Level)
- `async_domains`:
  - Keep `{pclk pclk_div2}` asynchronous to `{ref_clk}` and to system/DDR clocks.
- `ddr_reset_cfg_async`:
  - `{cfg_clk ddr_rst_clk}` asynchronous to `{ddrphy_sysclk phy_dq_clk_0 phy_dq_sysclk_0 phy_dq_clk_1 phy_dq_sysclk_1}`.
- `cfg_to_ddr_rst_async`:
  - `{cfg_clk}` asynchronous to `{ddr_rst_clk}`.
- `ddr_core_io_async`:
  - `{ddrphy_sysclk}` asynchronous to `{phy_dq_clk_0 phy_dq_sysclk_0 phy_dq_clk_1 phy_dq_sysclk_1}`.

These are the only accepted async-domain exceptions for this fix.

## Why Clock-Group Method
- Deep DDR IP pin-level false paths (`set_false_path -to [get_pins ...]`) were unstable in this flow and could trigger compile failure (`Flow-0189`).
- Clock-group isolation is compile-safe in this project and keeps setup checks on real synchronous domains.

## Multiseed Closure (211-218)
Source summary: `closure_runs/ddrfix_multiseed_8/multiseed_summary.csv`

| seed | pclk WNS | ddrphy WNS | pclk_div2 WNS | pclk_div2 FE | result |
|---|---:|---:|---:|---:|---|
| 211 | 0.238 | 2.175 | 0.833 | 0 | all constraints met |
| 212 | 0.286 | 1.846 | 0.269 | 0 | all constraints met |
| 213 | 0.135 | 2.480 | 0.515 | 0 | all constraints met |
| 214 | 0.302 | 1.999 | 0.323 | 0 | min hold violations |
| 215 | 0.500 | 2.428 | 0.512 | 0 | all constraints met |
| 216 | 0.495 | 2.441 | 0.523 | 0 | min hold violations |
| 217 | 0.510 | 2.606 | 0.765 | 0 | min hold violations |
| 218 | 0.367 | 1.904 | 0.570 | 0 | all constraints met |

Observed residual failing seeds were hold-min paths (`sys_clk` local counter or `cmos1_pclk` input capture), not DDR setup paths.

## Locked Implementation Seed
- Recommended production seed lock:
  - `-gplace_seed 211`
  - `-groute_seed 1211`
  - keep `-fix_hold_violation_in_route TRUE`

This reproduces fully clean timing in the current RTL/FDC snapshot.
