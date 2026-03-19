# V5.1 Execution Report

## Baseline
- Source: `report_timing/bak/pcie_dma_ddr3_cam1.rtr` (2026-03-18 13:19)
- Baseline key metrics:
  - `pclk->pclk`: WNS `-0.029`
  - `ddrphy_sysclk->ddrphy_sysclk`: WNS `-0.201`
  - `pclk_div2->pclk_div2`: WNS `-2.184`, TNS `-305.899`, FE `282`

## P1 Result (implementation-only multiseed, 5 seeds)
- Summary: `p1_multiseed_summary.csv`
- Best seed: `103`
- Best-seed key metrics:
  - `pclk->pclk`: WNS `0.250` (new regression cleared)
  - `ddrphy_sysclk->ddrphy_sysclk`: WNS `0.073` (new regression cleared)
  - `pclk_div2->pclk_div2`: WNS `-1.972`, TNS `-351.105`, FE `375`
  - FDC parser/import gate count: `0`
- Notes:
  - New 2 fail groups were eliminated in best seed.
  - `pclk_div2` improved in WNS versus baseline but still below `-1.8ns`.

## P2 Candidate Result (extra local input stage in RD ctrl, 5 seeds)
- Summary: `p2_multiseed_summary.csv`
- Best seed: `102`
- Best-seed key metrics:
  - `pclk->pclk`: WNS `0.186`
  - `ddrphy_sysclk->ddrphy_sysclk`: WNS `0.014`
  - `pclk_div2->pclk_div2`: WNS `-2.111`, TNS `-285.526`, FE `239`
  - FDC parser/import gate count: `0`
- Decision:
  - P2 candidate is **not kept** (best WNS worse than P1 best `-1.972`).
  - Reverted to P1-state RTL for display-risk minimization.

## Gate Decision
- Top50 `IP_OWNED` ratio in P1 sweep is consistently high (`88%`~`94%`).
- Per gate (`>80%`), further hard RTL compression is stopped.
- Next step is IP-side closure handoff list (user-regenerated IP only).
