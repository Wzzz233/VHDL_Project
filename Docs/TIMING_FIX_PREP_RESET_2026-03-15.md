# Timing Fix Prep/Reset Handoff

## Scope
- Branch: `timing-fix-pclkdiv2-prep-and-reset`
- Goal: clean reset/CDC timing noise first, then pipeline the `prep` hot path on `pclk_div2`
- This document records the known baseline and the exact items to verify after you rerun `synthesize -> device_map -> place_route -> report_timing`

## Baseline Before This Round
- Post-PnR `Fmax Summary` showed `pclk_div2 = 104.9759 MHz`, requested `125 MHz`, slack `-1.526 ns`
- Synthesis `Fmax Summary` showed `pclk_div2 = 117.5641 MHz`, slack `-0.506 ns`
- Main true hot path was `pclk_div2 -> pclk_div2` inside `prep`, with the reported slow-setup path ending at `prep_stage_b_median_word[14]`
- Known timing-noise classes before cleanup:
- `cfg_clk -> sys_clk` through `rstn_1ms -> power_on_delay`
- `pclk_div2 -> ref_clk` through LED/link-up status fanout
- DDR vendor recovery/removal noise around async reset and synchronizer first-stage endpoints

## Implemented Changes
- Commit `d1cf244` `Constrain reset CDC and isolate LED cross-domain paths`
- `fdc/pcie_dma_test.fdc`
- Added a narrow `set_false_path -through [get_nets {ddr_rstn}]`
- `hdl/pcie_dma_ddr3_cam1.v`
- Added 2FF link-up synchronizers into `ref_clk` and `pclk` before driving `ref_led` and `pclk_led`
- Added a `sys_clk`-domain reset-release synchronizer before `u_power_on_delay.reset_n`

- Commit `f3950a9` `Pipeline prep median hot path on pclk_div2`
- `hdl/pcie_dma_ddr3_cam1.v`
- Split the `prep` neighborhood walk into an extra registered stage
- New stage stores per-lane neighborhood candidates plus row min/max
- Final `median/min/max` reduction now happens one cycle later before `spatial_enhance_y`
- Raw BAR2/DMA startup, `out_pair_*`, and `prep_pair_*` queue semantics were left unchanged

## Required Rerun Checks
- Run full flow locally:
- `synthesize`
- `device_map`
- `place_route`
- `report_timing`

## Fill After Rerun
- `Fmax Summary`
- `synthesize`: `pclk_div2 = ______ MHz`, slack `______ ns`
- `place_route`: `pclk_div2 = ______ MHz`, slack `______ ns`

- `slow corner`
- setup: `______`
- hold: `______`
- recovery: `______`
- removal: `______`

- `fast corner`
- setup: `______`
- hold: `______`
- recovery: `______`
- removal: `______`

- `Clock Interaction`
- `cfg_clk -> sys_clk` reset item still present? `yes / no`
- `pclk_div2 -> ref_clk` LED item still present? `yes / no`
- Any new raw-path red items? `yes / no`

## Acceptance Target
- `cfg_clk -> sys_clk` reset noise no longer appears as a user-RTL red summary item
- `pclk_div2 -> ref_clk` LED status path no longer appears as a red summary item
- `pclk_div2 -> pclk_div2` setup closes with `WNS >= 0`
- No new raw-path red summary item is introduced
- If any residual red item remains, list the endpoint and classify it as:
- user RTL path that still needs fixing
- vendor DDR internal async-reset/synchronizer noise

## Board-Side Follow-Up
- Re-run the raw baseline command after timing passes
- Re-run one `prep` profile for a 60s smoke test and confirm:
- no black screen
- no tearing / corruption
- no DMA timeout
