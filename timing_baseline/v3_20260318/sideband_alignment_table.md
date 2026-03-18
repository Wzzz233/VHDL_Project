# Sideband Alignment Table (Current Zero-Latency Path)

Clock domain: `pclk_div2`

| Signal | Cycle relation | Notes |
|---|---|---|
| `bar2_addr_step` | base event | Defined from `mwr_rd_clk_en` edge/address-step. |
| `frame_rd_fetch_en` | same cycle as `bar2_addr_step` when `~dma_expand_phase` | Read-enable for frame buffer; no extra pipeline inserted. |
| `dma_expand_phase` | toggles on each `bar2_addr_step` in expand mode | Selects low/high half output path. |
| `frame_rd_data_hold` | updated on `frame_rd_fetch_en` | Caches 128b source for high-half BGRX expansion. |
| `mwr_rd_data` | combinational select from `frame_dma_data` | `frame_dma_data` uses current phase and held/raw data; no added cycle. |

This table is frozen for P2 (no added latency). Any P3 pipeline proposal must update this table and provide board regression evidence before merge.
