#!/usr/bin/env python3
"""
Run the fixed-ROI OCR denoise experiment end to end.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path
from typing import List


MODES = [
    "raw",
    "median3",
    "median5",
    "gauss3",
    "gauss3_sharp_mild",
    "gray3",
]


def run_cmd(cmd: List[str]) -> None:
    print("[run]", " ".join(str(x) for x in cmd))
    subprocess.run(cmd, check=True)


def main() -> int:
    ap = argparse.ArgumentParser(description="Run OCR ROI denoise experiment.")
    ap.add_argument("--roi-manifest", required=True, help="CSV with sample_id,image_path,x1,y1,x2,y2,plate_text,scenario")
    ap.add_argument("--ocr-model", required=True, help="OCR RKNN model path")
    ap.add_argument("--ocr-keys", required=True, help="OCR keys txt path")
    ap.add_argument("--out-dir", required=True, help="Output directory")
    ap.add_argument("--batch-bin", default="./ocr_roi_batch", help="Path to compiled OCR-only batch binary")
    ap.add_argument("--max-samples", type=int, default=0, help="0 means all samples")
    ap.add_argument("--ocr-in-w", type=int, default=94)
    ap.add_argument("--ocr-in-h", type=int, default=24)
    ap.add_argument("--sanity-prefix", default="sanity", help="Scenario prefix treated as sanity subset")
    args = ap.parse_args()

    script_dir = Path(__file__).resolve().parent
    gen_script = script_dir / "gen_ocrin_dataset.py"
    eval_script = script_dir / "ocr_roi_eval.py"
    batch_bin = Path(args.batch_bin)
    if not batch_bin.is_absolute() and not batch_bin.exists():
        alt_batch = script_dir / batch_bin
        if alt_batch.exists():
            batch_bin = alt_batch
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    if not gen_script.exists():
        raise FileNotFoundError(gen_script)
    if not eval_script.exists():
        raise FileNotFoundError(eval_script)
    if not batch_bin.exists():
        raise FileNotFoundError(f"batch binary not found: {batch_bin}")

    pred_paths: List[str] = []
    for mode in MODES:
        mode_dir = out_dir / mode
        pred_csv = out_dir / f"pred_{mode}.csv"
        run_cmd(
            [
                sys.executable,
                str(gen_script),
                "--ann-csv",
                str(Path(args.roi_manifest).resolve()),
                "--out-dir",
                str(mode_dir.resolve()),
                "--ocr-in-w",
                str(args.ocr_in_w),
                "--ocr-in-h",
                str(args.ocr_in_h),
                "--ocr-channel-order",
                "bgr",
                "--ocr-crop-mode",
                "match",
                "--ocr-resize-mode",
                "letterbox",
                "--ocr-resize-kernel",
                "nn",
                "--ocr-preproc",
                mode,
            ]
            + (["--max-samples", str(args.max_samples)] if args.max_samples > 0 else [])
        )
        run_cmd(
            [
                str(batch_bin),
                "--index",
                str((mode_dir / "index.csv").resolve()),
                "--model",
                str(Path(args.ocr_model).resolve()),
                "--ocr-keys",
                str(Path(args.ocr_keys).resolve()),
                "--out-csv",
                str(pred_csv.resolve()),
            ]
        )
        pred_paths.append(str(pred_csv.resolve()))

    cmd = [
        sys.executable,
        str(eval_script),
        "--out-dir",
        str(out_dir.resolve()),
        "--sanity-prefix",
        args.sanity_prefix,
    ]
    for pred in pred_paths:
        cmd.extend(["--pred", pred])
    run_cmd(cmd)

    print(f"summary={str((out_dir / 'summary_ranked.md').resolve())}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
