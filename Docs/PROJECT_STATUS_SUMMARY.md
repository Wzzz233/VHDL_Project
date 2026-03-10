# Project Status Summary (2026-03-10)

## Scope
- Branch: `main`
- Current head: `058802c`
- Focus area in this round:
  - Recover OV5640 live image stability after the `2026-03-09` init-sequencing regression window.
  - Add a first safe round of OV5640 noise tuning without expanding runtime sensor-side CLI.
  - Add a display-side `median` FPGA preproc profile for quick A/B during board validation.

## Recent Relevant Commits
- `07df54c` `Harden OV5640 init sequencing and audit`
- `16266c2` `Restore OV5640 reset settle delay`
- `2be3c03` `Fix OV5640 init ACK stall and add debug`
- `da49a17` `Fix OV5640 I2C start hold regression`
- `058802c` `Tune OV5640 noise preset and add median profile`

## Current Technical Conclusion
- The previous live-view blackout / corruption issue was traced to OV5640 init sequencing, not to the `raw` display path itself.
- The live camera image is now back after the init fixes; current work has shifted from “recover image” to “reduce visible noise”.
- The user’s normal runtime command still uses `--fpga-preproc-profile raw`, which bypasses FPGA preprocessing. In that mode, visible noise is still dominated by the sensor’s AGC/CIP behavior.
- Because of that, this round adds two complementary levers:
  - sensor-side compile-time OV5640 noise presets in `hdl/reg_config.v`
  - display-side runtime `median` profile in `ARM/fpga_lpr_display.c`

## Requirements vs Progress

Cross-checked against the competition requirements in `Docs/最终项目要求.md`.

### 1. End-to-end image capture -> transport -> display
- Status: **Working on board**
- Evidence:
  - live OV5640 image is restored after init regression fixes
  - `run_lpr_kms.sh` remains the unified runtime entry
  - no user-visible CLI break was introduced in this round

### 2. FPGA-side image preprocessing capability
- Status: **Partially implemented and now more testable**
- Existing controls already present before this round:
  - `raw`
  - `clahe`
  - `clahe_usm`
  - `median_clahe_usm`
  - `ocr_stroke`
- Newly added in this round:
  - `median`
- Interpretation:
  - this improves alignment with the requirement for FPGA-side denoise / enhancement
  - board-side acceptance still needs A/B evidence, especially for `median --fpga-preproc-target all`

### 3. Plate detection / OCR integration
- Status: **Available, still quality-limited**
- Current baseline chain remains:
  - `--ocr-channel-order bgr`
  - `--ocr-crop-mode match`
  - `--ocr-resize-mode letterbox`
  - `--ocr-resize-kernel nn`
  - `--ocr-min-occ-ratio 0.90`
- Main open risk is no longer crop geometry; it is still model quality / weak-light image quality.

### 4. System stability requirement
- Status: **Recovered, needs fresh long-run validation after noise tuning**
- Regression fixed in code:
  - OV5640 init FSM stall behavior
  - OV5640 I2C transaction hold / `start` regression
- Fresh validation still required for the new noise-tuning state:
  - no black screen
  - no snow / stripe corruption
  - no new DMA timeout regression during 60s run

## What Was Implemented In This Round

### 1. OV5640 compile-time noise presets
- File: `hdl/reg_config.v`
- Added local preset selection:
  - `OV5640_NOISE_PRESET_REF`
  - `OV5640_NOISE_PRESET_LOW_NOISE_BALANCED`
  - `OV5640_NOISE_PRESET_LOW_NOISE_DISPLAY`
- Current default:
  - `LOW_NOISE_BALANCED`
- Scope of register changes intentionally kept small:
  - gain ceiling: `0x3a18/0x3a19`
  - CIP sharpen / DNS: `0x5300..0x530c`
- Kept unchanged in first round:
  - `0x5000`
  - `0x5001`
  - `0x3503`
  - `0x3a13`
  - `0x3a02/03/14/15`
  - `0x4300`
  - `0x501f`

### 2. New display-side `median` FPGA profile
- File: `ARM/fpga_lpr_display.c`
- Added user-visible runtime profile:
  - `--fpga-preproc-profile median`
- Behavior:
  - enable median only
  - disable CLAHE
  - disable USM
  - disable OCR-stroke-specific path
- Help text updated in:
  - `ARM/fpga_lpr_display.c`
  - `ARM/run_lpr_kms.sh`

## Current Recommended Validation Order

### Board validation pass 1: sensor-only effect
- Rebuild / flash the current RTL bitstream from `058802c`.
- Run the normal baseline command with:
  - `--fpga-preproc-profile raw`
  - `--fpga-preproc-target ocr`
- Goal:
  - isolate the effect of `LOW_NOISE_BALANCED`

### Board validation pass 2: display-side A/B
- Keep the same bitstream.
- Compare:
  - `raw`
  - `median --fpga-preproc-target all`
  - `median_clahe_usm --fpga-preproc-target all`
- Goal:
  - check whether visible grain improves enough without unacceptable OCR regression

### Board validation pass 3: more aggressive sensor preset
- Only if needed, rebuild with:
  - `OV5640_NOISE_PRESET_LOW_NOISE_DISPLAY`
- Compare against `LOW_NOISE_BALANCED` under `raw`.

## Acceptance Gates For This Round
- Live image must remain stable.
- No regression to black screen, snow, stripes, or frame misalignment.
- Fine low-light grain should improve versus the previous `raw` baseline.
- OCR quality may soften slightly, but should not show obvious sustained collapse.
- A 60s run should not introduce new DMA timeout / forced-exit behavior.

## Open Items
- Collect board-side A/B evidence for:
  - `BALANCED + raw`
  - `BALANCED + median + target=all`
  - `BALANCED + median_clahe_usm + target=all`
- Decide whether `LOW_NOISE_BALANCED` is sufficient as the default shipped sensor preset.
- If `median` helps display quality but harms OCR too much, keep it as a display-only debug option rather than a default path.
- If noise still remains too visible after the current round, the next safe tuning axis should remain narrow:
  - refine gain ceiling
  - refine CIP sharpen/DNS
  - avoid broad color / matrix / AF changes in the same round

## Status Snapshot
- OV5640 init regression recovery: **Done**
- Live image restoration: **Done**
- First-round sensor noise tuning: **Implemented, awaiting board verdict**
- Runtime display-side `median` A/B lever: **Done**
- Requirement-level evidence package for final delivery: **In progress**
