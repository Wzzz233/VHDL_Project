#!/usr/bin/env python3
"""
Apply OCR ROI experiment preprocess modes to a trusted full-frame RGB image.

This is only for human visual comparison on a whole frame. It reuses the same
software-side preprocess functions used in the OCR ROI experiment pipeline and
does not affect FPGA, DMA, or runtime display paths.
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from PIL import Image

from gen_ocrin_dataset import PREPROC_CHOICES, ocr_preprocess_rgb888, save_ppm_rgb


FULLFRAME_MODES = [
    "gray3",
    "median3",
    "median5",
    "gauss3",
    "gauss3_sharp_mild",
]


def load_rgb_image(path: Path) -> np.ndarray:
    img = Image.open(path).convert("RGB")
    return np.array(img, dtype=np.uint8)


def main() -> int:
    parser = argparse.ArgumentParser(description="Apply preprocess modes to one full-frame RGB image")
    parser.add_argument("--input", required=True, help="Trusted input image path, e.g. frame_raw.ppm")
    parser.add_argument("--out-dir", required=True, help="Output directory for processed images")
    parser.add_argument(
        "--modes",
        default=",".join(FULLFRAME_MODES),
        help="Comma-separated modes to run. Default: gray3,median3,median5,gauss3,gauss3_sharp_mild",
    )
    parser.add_argument(
        "--save-raw-copy",
        action="store_true",
        help="Also save a copy of the input image as raw.ppm in the output directory",
    )
    args = parser.parse_args()

    in_path = Path(args.input)
    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    rgb = load_rgb_image(in_path)

    if args.save_raw_copy:
        save_ppm_rgb(out_dir / "raw.ppm", rgb)

    modes = [m.strip() for m in args.modes.split(",") if m.strip()]
    bad = [m for m in modes if m not in PREPROC_CHOICES or m in {"none", "raw", "gray", "bin"}]
    if bad:
        raise SystemExit(f"unsupported full-frame modes: {bad}")

    for mode in modes:
        out = ocr_preprocess_rgb888(rgb.copy(), mode)
        out_path = out_dir / f"frame_{mode}.ppm"
        save_ppm_rgb(out_path, out)
        print(f"wrote mode={mode} path={out_path}")

    print(f"done input={in_path} out_dir={out_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
