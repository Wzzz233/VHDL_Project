#!/usr/bin/env python3
"""
Phase 3.2 frame-diff probe helper.

Usage:
  python3 analyze_frame_diff.py --pattern /tmp/fprobe/motion_*.raw
"""

from __future__ import annotations

import argparse
import glob
import hashlib
import os
import statistics
from typing import List


def calc_unique_hashes(files: List[str]) -> int:
    hashes = set()
    for path in files:
        h = hashlib.sha256()
        with open(path, "rb") as f:
            for chunk in iter(lambda: f.read(1024 * 1024), b""):
                h.update(chunk)
        hashes.add(h.hexdigest())
    return len(hashes)


def frame_diff_ratio(a: bytes, b: bytes) -> float:
    n = min(len(a), len(b))
    if n == 0:
        return 0.0
    diff = sum(x != y for x, y in zip(a[:n], b[:n]))
    return diff / n


def classify(avg_diff: float) -> str:
    if avg_diff < 0.01:
        return "LIKELY_SOURCE_STALE (avg_diff < 1%)"
    if avg_diff >= 0.05:
        return "LIKELY_DISPLAY_LATENCY_OR_BUFFERING (avg_diff >= 5%)"
    return "BORDERLINE_NEED_VISUAL_CONFIRMATION (1% <= avg_diff < 5%)"


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--pattern",
        default="/tmp/fprobe/motion_*.raw",
        help="Glob pattern for raw frame files",
    )
    args = parser.parse_args()

    files = sorted(glob.glob(args.pattern))
    if len(files) < 2:
        print(f"ERR: not enough frames for pattern: {args.pattern}")
        return 1

    ratios: List[float] = []
    for i in range(1, len(files)):
        with open(files[i - 1], "rb") as fa:
            a = fa.read()
        with open(files[i], "rb") as fb:
            b = fb.read()
        ratios.append(frame_diff_ratio(a, b))

    unique = calc_unique_hashes(files)
    gt1 = sum(r > 0.01 for r in ratios)
    gt5 = sum(r > 0.05 for r in ratios)
    avg = statistics.mean(ratios)

    print("=== Phase 3.2 Frame Diff Report ===")
    print(f"frames={len(files)} pairs={len(ratios)} unique_sha256={unique}")
    print(
        "diff_ratio: "
        f"min={min(ratios):.4f} avg={avg:.4f} median={statistics.median(ratios):.4f} max={max(ratios):.4f}"
    )
    print(f"pairs_over_1pct={gt1}/{len(ratios)}")
    print(f"pairs_over_5pct={gt5}/{len(ratios)}")
    print(f"classification={classify(avg)}")

    base = os.path.splitext(files[0])[0]
    if base.endswith("_0000"):
        prefix = base[:-5]
    else:
        prefix = base.rsplit("_", 1)[0]
    print("visual_check_ppm=" + ", ".join(f"{prefix}_{i:04d}.ppm" for i in (0, 10, 20, 30, 40, 50)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

