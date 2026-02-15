#!/usr/bin/env python3
"""
Phase S image-quality diagnostics from PPM (P6) frame sequences.

Outputs:
- Sharpness/aliasing proxies (Laplacian variance, edge density, high-frequency ratio)
- AWB/AEC observation stats (RGB mean drift, luma drift, clipping)
"""

from __future__ import annotations

import argparse
import glob
import math
import statistics
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Sequence, Tuple

try:
    import numpy as np  # type: ignore
except Exception:  # pragma: no cover
    np = None


@dataclass
class FrameMetrics:
    path: str
    width: int
    height: int
    mean_r: float
    mean_g: float
    mean_b: float
    luma_mean: float
    luma_std: float
    dark_clip_pct: float
    bright_clip_pct: float
    lap_var: float
    edge_density: float
    hf_ratio: float


def _next_token(blob: bytes, pos: int) -> Tuple[str, int]:
    n = len(blob)
    while pos < n and blob[pos] in b" \t\r\n":
        pos += 1
    while pos < n and blob[pos] == ord("#"):
        while pos < n and blob[pos] not in b"\r\n":
            pos += 1
        while pos < n and blob[pos] in b" \t\r\n":
            pos += 1
    if pos >= n:
        raise ValueError("Unexpected EOF while parsing PPM header")
    start = pos
    while pos < n and blob[pos] not in b" \t\r\n":
        pos += 1
    return blob[start:pos].decode("ascii"), pos


def read_ppm_p6(path: str) -> Tuple[int, int, bytes]:
    blob = Path(path).read_bytes()
    pos = 0
    magic, pos = _next_token(blob, pos)
    if magic != "P6":
        raise ValueError(f"{path}: unsupported magic {magic}, expected P6")
    w_tok, pos = _next_token(blob, pos)
    h_tok, pos = _next_token(blob, pos)
    m_tok, pos = _next_token(blob, pos)
    w = int(w_tok)
    h = int(h_tok)
    maxval = int(m_tok)
    if maxval != 255:
        raise ValueError(f"{path}: unsupported maxval {maxval}, expected 255")
    while pos < len(blob) and blob[pos] in b" \t\r\n":
        pos += 1
    need = w * h * 3
    payload = blob[pos : pos + need]
    if len(payload) != need:
        raise ValueError(f"{path}: invalid payload length {len(payload)} != {need}")
    return w, h, payload


def _gray_sample(rgb: bytes, w: int, h: int, step: int) -> Tuple[List[int], int, int]:
    gw = max(1, w // step)
    gh = max(1, h // step)
    g: List[int] = [0] * (gw * gh)
    yi = 0
    for y in range(0, h, step):
        if yi >= gh:
            break
        xi = 0
        base_row = y * w * 3
        for x in range(0, w, step):
            if xi >= gw:
                break
            p = base_row + x * 3
            r = rgb[p + 0]
            gch = rgb[p + 1]
            b = rgb[p + 2]
            gray = (77 * r + 150 * gch + 29 * b) >> 8
            g[yi * gw + xi] = gray
            xi += 1
        yi += 1
    return g, gw, gh


def _mean_std(values: Sequence[float]) -> Tuple[float, float]:
    if not values:
        return 0.0, 0.0
    mean = statistics.mean(values)
    std = statistics.pstdev(values) if len(values) > 1 else 0.0
    return mean, std


def _laplacian_metrics(gray: List[int], gw: int, gh: int) -> Tuple[float, float]:
    if gw < 3 or gh < 3:
        return 0.0, 0.0
    lap_values: List[float] = []
    edge_cnt = 0
    total = 0
    for y in range(1, gh - 1):
        row = y * gw
        up = (y - 1) * gw
        dn = (y + 1) * gw
        for x in range(1, gw - 1):
            c = gray[row + x]
            l = gray[row + x - 1]
            r = gray[row + x + 1]
            u = gray[up + x]
            d = gray[dn + x]
            lap = float((4 * c) - l - r - u - d)
            lap_values.append(lap)
            gx = r - l
            gy = d - u
            if abs(gx) + abs(gy) >= 32:
                edge_cnt += 1
            total += 1
    if not lap_values or total == 0:
        return 0.0, 0.0
    mean = statistics.mean(lap_values)
    var = statistics.pvariance(lap_values, mu=mean)
    edge_density = edge_cnt / float(total)
    return var, edge_density


def _hf_ratio_fft(gray: List[int], gw: int, gh: int) -> float:
    if np is None or gw < 16 or gh < 16:
        return float("nan")
    arr = np.array(gray, dtype=np.float32).reshape(gh, gw)
    if gh > 256 or gw > 256:
        sy = max(1, gh // 256)
        sx = max(1, gw // 256)
        arr = arr[::sy, ::sx]
    arr = arr - float(arr.mean())
    f = np.fft.fftshift(np.fft.fft2(arr))
    mag2 = np.abs(f) ** 2
    h, w = mag2.shape
    cy = (h - 1) * 0.5
    cx = (w - 1) * 0.5
    yy, xx = np.ogrid[:h, :w]
    rr = np.sqrt((yy - cy) ** 2 + (xx - cx) ** 2)
    rmax = np.sqrt(cy * cy + cx * cx)
    high = rr >= (0.35 * rmax)
    etot = float(mag2.sum()) + 1e-12
    ehigh = float(mag2[high].sum())
    return ehigh / etot


def calc_metrics(path: str, sample_step: int) -> FrameMetrics:
    w, h, rgb = read_ppm_p6(path)
    n = w * h
    sum_r = 0
    sum_g = 0
    sum_b = 0
    luma: List[float] = []
    dark = 0
    bright = 0
    for i in range(0, len(rgb), 3):
        r = rgb[i + 0]
        g = rgb[i + 1]
        b = rgb[i + 2]
        sum_r += r
        sum_g += g
        sum_b += b
        y = 0.299 * r + 0.587 * g + 0.114 * b
        luma.append(y)
        if y <= 5.0:
            dark += 1
        if y >= 250.0:
            bright += 1
    mean_r = sum_r / float(n)
    mean_g = sum_g / float(n)
    mean_b = sum_b / float(n)
    luma_mean, luma_std = _mean_std(luma)

    gray, gw, gh = _gray_sample(rgb, w, h, sample_step)
    lap_var, edge_density = _laplacian_metrics(gray, gw, gh)
    hf_ratio = _hf_ratio_fft(gray, gw, gh)
    if math.isnan(hf_ratio):
        # Fallback proxy when numpy FFT is unavailable.
        hf_ratio = min(1.0, lap_var / (lap_var + 4000.0))

    return FrameMetrics(
        path=path,
        width=w,
        height=h,
        mean_r=mean_r,
        mean_g=mean_g,
        mean_b=mean_b,
        luma_mean=luma_mean,
        luma_std=luma_std,
        dark_clip_pct=(dark * 100.0 / float(n)),
        bright_clip_pct=(bright * 100.0 / float(n)),
        lap_var=lap_var,
        edge_density=edge_density,
        hf_ratio=hf_ratio,
    )


def _summ(name: str, vals: Iterable[float]) -> str:
    arr = list(vals)
    if not arr:
        return f"{name}: n=0"
    return (
        f"{name}: min={min(arr):.4f} avg={statistics.mean(arr):.4f} "
        f"median={statistics.median(arr):.4f} max={max(arr):.4f}"
    )


def classify(ms: List[FrameMetrics]) -> List[str]:
    notes: List[str] = []
    if not ms:
        return notes
    r_mean = statistics.mean(m.mean_r for m in ms)
    g_mean = statistics.mean(m.mean_g for m in ms)
    b_mean = statistics.mean(m.mean_b for m in ms)
    luma_span = max(m.luma_mean for m in ms) - min(m.luma_mean for m in ms)
    dark_avg = statistics.mean(m.dark_clip_pct for m in ms)
    bright_avg = statistics.mean(m.bright_clip_pct for m in ms)
    lap_med = statistics.median(m.lap_var for m in ms)

    if g_mean > max(r_mean, b_mean) * 1.12:
        notes.append("GREEN_CAST_SUSPECTED (G channel mean dominates R/B)")
    if dark_avg > 3.0:
        notes.append("UNDEREXPOSURE_OR_GAIN_NOISE_SUSPECTED (dark clipping > 3%)")
    if bright_avg > 1.5:
        notes.append("HIGHLIGHT_CLIP_SUSPECTED (bright clipping > 1.5%)")
    if luma_span > 10.0:
        notes.append("AEC_STABILITY_RISK (luma mean span > 10)")
    if lap_med < 40.0:
        notes.append("SOFTNESS_SUSPECTED (median Laplacian variance < 40)")
    if not notes:
        notes.append("NO_STRONG_RED_FLAG_IN_SEQUENCE")
    return notes


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--pattern", default="/tmp/fprobe/static_*.ppm", help="Glob for PPM P6 files")
    parser.add_argument("--label", default="phase_s", help="Label for report heading")
    parser.add_argument("--sample-step", type=int, default=2, help="Spatial downsample step for metrics")
    args = parser.parse_args()

    files = sorted(glob.glob(args.pattern))
    if not files:
        print(f"ERR: no files matched pattern: {args.pattern}")
        return 2

    metrics = [calc_metrics(p, max(1, args.sample_step)) for p in files]

    print(f"=== Phase S PPM Quality Report ({args.label}) ===")
    print(f"frames={len(metrics)} resolution={metrics[0].width}x{metrics[0].height}")
    print(_summ("lap_var", (m.lap_var for m in metrics)))
    print(_summ("edge_density", (m.edge_density for m in metrics)))
    print(_summ("hf_ratio", (m.hf_ratio for m in metrics)))
    print(_summ("luma_mean", (m.luma_mean for m in metrics)))
    print(_summ("luma_std", (m.luma_std for m in metrics)))
    print(_summ("dark_clip_pct", (m.dark_clip_pct for m in metrics)))
    print(_summ("bright_clip_pct", (m.bright_clip_pct for m in metrics)))
    print(_summ("mean_r", (m.mean_r for m in metrics)))
    print(_summ("mean_g", (m.mean_g for m in metrics)))
    print(_summ("mean_b", (m.mean_b for m in metrics)))
    for note in classify(metrics):
        print(f"classification={note}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

