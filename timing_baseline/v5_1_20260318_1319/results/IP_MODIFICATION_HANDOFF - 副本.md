# IP Modification Handoff (Triggered by >80% IP-Owned Top50)

## Trigger Evidence
- P1 5-seed sweep (`results/p1_multiseed_summary.csv`) shows Top50 `IP_OWNED` ratio in `88%~94%`.
- Under v5.1 gate rule, stop further hard RTL compression and hand off IP-side closure.

## Required IP-Side Focus Areas
1. DDR3 PHY / controller timing inside `u_DDR3/u_ddrphy_top` and `u_DDR3/u_ips_ddrc_top`
- Dominant violated groups: `ddrphy_sysclk` related internal paths and PHY-domain crossing timing.
- High-frequency offenders include reset/control propagation in:
  - `ddrphy_reset_ctrl/*`
  - `ddrphy_dfi/*`
  - `ddrphy_slice_top/*`
- Action on IP side:
  - enable/strengthen internal pipeline or register duplication options (if available in DDR3 IP configurator),
  - review internal reset/control synchronization options exposed by IP parameters,
  - regenerate IP and rerun closure with same RTL/FDC.

2. PCIe hard block internal `pclk` margin sensitivity in `u_ips2l_pcie_wrap/U_IPS2L_PCIE_TOP/U_IPS2L_PCIE_HARD_CTRL`
- Regression group `pclk->pclk` is seed-sensitive and originates from hard block internal RAM-to-gate routes.
- Action on IP side:
  - check available PCIe IP build/implementation strategy knobs for timing-oriented closure,
  - regenerate PCIe IP with timing-closure oriented settings and rerun the same multiseed flow.

3. Keep FDC safety gate unchanged
- Continue requiring zero parser/import errors:
  - `Timing-4003`
  - `CommandTiming-0057`
  - `SDC-2023`
  - `ConstraintEditor-0046`
  - `SDC-2017`
- Do not use broad false-path relaxation to mask functional paths.

## Validation Expectations After IP Regeneration
- Reuse `tools/timing_v51/run_v51_multiseed.py` with the same seed set.
- Acceptance priority:
  1) `pclk->pclk` and `ddrphy_sysclk->ddrphy_sysclk` remain `WNS >= 0`;
  2) `pclk_div2->pclk_div2` improves without display semantic regression;
  3) FDC gate remains zero.
