# Raw Baseline Reset

- Branch: `raw-baseline-reset-20260315`
- Base commit: `49a6892`
- Goal: maintain a prep-free raw baseline branch that can be validated and versioned independently before any new prep redesign starts.

## Included in this branch

- Full codebase reset to the pre-prep mainline point `49a6892`
- Raw-focused validation assets:
  - `tb_raw_dma_path.v`
  - `tb_raw_bar2_queue_path.v`
  - `tb_dma_session_start_alignment.v`
- Timing constraint refresh in `fdc/pcie_dma_test.fdc` so `sys_clk`, DDR-derived clocks, and `clock_20k` are modeled explicitly during later timing closure work

## Validation commands

Run from repo root with ModelSim on Windows:

```powershell
vlog hdl/pcie_dma_ctrl/ips2l_pcie_dma_rd_ctrl.v tb_raw_dma_path.v
vsim -c tb_raw_dma_path -do "run -all; quit -f"

vlog hdl/pcie_dma_ctrl/fifo/*.v `
     hdl/pcie_dma_ctrl/ipm_distributed_sdpram_v1_2.v `
     hdl/pcie_dma_ctrl/pgs_pciex4_prefetch_fifo_v1_2.v `
     hdl/pcie_dma_ctrl/ips2l_pcie_dma_rd_ctrl.v `
     tb_raw_bar2_queue_path.v
vsim -c tb_raw_bar2_queue_path -do "run -all; quit -f"

vlog hdl/pcie_dma_ctrl/ips2l_pcie_dma_controller.v hdl/pcie_dma_ctrl/ips2l_pcie_dma_mwr_tx_ctrl.v tb_dma_session_start_alignment.v
vsim -c tb_dma_session_start_alignment -do "run -all; quit -f"
```

Expected result:

- all three testbenches end in `PASS`
- no prep profile, BAR1 prep register, or prep RTL dependency is required for the raw branch

## Intent

This branch is the holding baseline for:

- proving raw stability on its own
- keeping later raw-only fixes auditable
- giving the next prep redesign a clean merge base instead of continuing on the old prep pipeline
