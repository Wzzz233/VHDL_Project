#!/usr/bin/env python3
"""
Generate OCR input samples (ocrin_*.ppm) on PC with the same OCR crop/resize
logic used by the board-side application, while allowing ROI-only preprocess
experiments on the crop before resize.

Supported annotation sources:
1) CSV with columns: image_path,x1,y1,x2,y2
   Optional columns: sample_id, plate_text, scenario
2) CCPD filename auto-parse (bbox from third '-' field: x1&y1_x2&y2)

Output:
- <out_dir>/ocrin_*.ppm
- <out_dir>/crop_*.ppm (optional)
- <out_dir>/index.csv
- <out_dir>/dataset.txt
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import numpy as np
from PIL import Image


OCR_CROP_WIDTH = 150
OCR_CROP_HEIGHT = 50
PREPROC_CHOICES = [
    "none",
    "raw",
    "gray",
    "gray3",
    "bin",
    "median3",
    "median5",
    "gauss3",
    "gauss3_sharp_mild",
]


@dataclass
class Box:
    x1: int
    y1: int
    x2: int
    y2: int

    @property
    def w(self) -> int:
        return self.x2 - self.x1 + 1

    @property
    def h(self) -> int:
        return self.y2 - self.y1 + 1


@dataclass
class Sample:
    sample_id: str
    image_path: str
    box: Box
    plate_text: str = ""
    scenario: str = ""


def norm_mode(mode: str) -> str:
    mode = (mode or "none").strip().lower()
    if mode == "raw":
        return "none"
    if mode == "gray3":
        return "gray"
    return mode


def clamp_box(b: Box, w: int, h: int) -> Box:
    x1 = max(0, min(w - 1, b.x1))
    y1 = max(0, min(h - 1, b.y1))
    x2 = max(0, min(w - 1, b.x2))
    y2 = max(0, min(h - 1, b.y2))
    if x2 < x1:
        x2 = x1
    if y2 < y1:
        y2 = y1
    return Box(x1, y1, x2, y2)


def compute_center_crop_box(src: Box, img_w: int, img_h: int, crop_w: int, crop_h: int) -> Box:
    cx = (src.x1 + src.x2) // 2
    cy = (src.y1 + src.y2) // 2
    x1 = cx - crop_w // 2
    y1 = cy - crop_h // 2
    if x1 < 0:
        x1 = 0
    if y1 < 0:
        y1 = 0
    if x1 + crop_w > img_w:
        x1 = img_w - crop_w
    if y1 + crop_h > img_h:
        y1 = img_h - crop_h
    if x1 < 0:
        x1 = 0
    if y1 < 0:
        y1 = 0
    return clamp_box(Box(x1, y1, x1 + crop_w - 1, y1 + crop_h - 1), img_w, img_h)


def compute_expand_crop_box(src: Box, img_w: int, img_h: int, pad_x: float, pad_y: float) -> Box:
    ex = int(src.w * pad_x + 0.5)
    ey = int(src.h * pad_y + 0.5)
    return clamp_box(Box(src.x1 - ex, src.y1 - ey, src.x2 + ex, src.y2 + ey), img_w, img_h)


def compute_ocr_crop_box(src: Box, img_w: int, img_h: int, crop_mode: str) -> Box:
    if crop_mode == "match":
        return clamp_box(src, img_w, img_h)
    if crop_mode == "box":
        return compute_expand_crop_box(src, img_w, img_h, 0.06, 0.12)
    if crop_mode == "tight":
        return compute_expand_crop_box(src, img_w, img_h, 0.08, 0.16)
    if crop_mode == "box-pad":
        return compute_expand_crop_box(src, img_w, img_h, 0.15, 0.28)
    return compute_center_crop_box(src, img_w, img_h, OCR_CROP_WIDTH, OCR_CROP_HEIGHT)


def estimate_ocr_occ_ratio(crop_w: int, crop_h: int, in_w: int, in_h: int, resize_mode: str) -> float:
    if crop_w <= 0 or crop_h <= 0 or in_w <= 0 or in_h <= 0:
        return 0.0
    if resize_mode != "letterbox":
        return 1.0
    sx = in_w / float(crop_w)
    sy = in_h / float(crop_h)
    scale = min(sx, sy)
    if scale <= 0.0:
        return 0.0
    scaled_w = int(crop_w * scale + 0.5)
    scaled_w = max(1, min(in_w, scaled_w))
    return scaled_w / float(in_w)


def compute_match_ytrim_crop(src: Box, in_w: int, in_h: int, min_occ_ratio: float) -> Optional[Box]:
    if min_occ_ratio <= 0.0:
        return None
    bw = src.w
    bh = src.h
    if bw <= 0 or bh <= 0 or in_w <= 0 or in_h <= 0:
        return None
    min_occ_ratio = min(1.0, min_occ_ratio)
    model_aspect = in_w / float(in_h)
    target_aspect = model_aspect * min_occ_ratio
    if target_aspect <= 0.0:
        return None
    cur_aspect = bw / float(bh)
    if cur_aspect >= target_aspect:
        return None
    new_h = int(bw / target_aspect + 0.5)
    if new_h < 1:
        new_h = 1
    if new_h >= bh:
        return None
    trim = (bh - new_h) // 2
    y1 = src.y1 + trim
    y2 = y1 + new_h - 1
    if y2 > src.y2:
        y2 = src.y2
        y1 = y2 - new_h + 1
    if y1 < src.y1:
        y1 = src.y1
    if y2 <= y1:
        return None
    return Box(src.x1, y1, src.x2, y2)


def resize_rgb888_nn(src: np.ndarray, dw: int, dh: int) -> np.ndarray:
    sh, sw, _ = src.shape
    dst = np.empty((dh, dw, 3), dtype=np.uint8)
    for y in range(dh):
        sy = (y * sh) // dh
        for x in range(dw):
            sx = (x * sw) // dw
            dst[y, x] = src[sy, sx]
    return dst


def resize_rgb888_bilinear(src: np.ndarray, dw: int, dh: int) -> np.ndarray:
    sh, sw, _ = src.shape
    if sw <= 1 or sh <= 1:
        return resize_rgb888_nn(src, dw, dh)
    dst = np.empty((dh, dw, 3), dtype=np.uint8)
    for y in range(dh):
        fy = ((y + 0.5) * sh / dh) - 0.5
        y0 = int(math.floor(fy))
        y0 = max(0, min(sh - 1, y0))
        y1 = min(sh - 1, y0 + 1)
        wy = min(1.0, max(0.0, fy - y0))
        for x in range(dw):
            fx = ((x + 0.5) * sw / dw) - 0.5
            x0 = int(math.floor(fx))
            x0 = max(0, min(sw - 1, x0))
            x1 = min(sw - 1, x0 + 1)
            wx = min(1.0, max(0.0, fx - x0))
            p00 = src[y0, x0].astype(np.float32)
            p01 = src[y0, x1].astype(np.float32)
            p10 = src[y1, x0].astype(np.float32)
            p11 = src[y1, x1].astype(np.float32)
            v0 = p00 * (1.0 - wx) + p01 * wx
            v1 = p10 * (1.0 - wx) + p11 * wx
            vv = v0 * (1.0 - wy) + v1 * wy
            dst[y, x] = np.clip(np.round(vv), 0, 255).astype(np.uint8)
    return dst


def resize_rgb888_with_kernel(src: np.ndarray, dw: int, dh: int, kernel: str) -> np.ndarray:
    if kernel == "bilinear":
        return resize_rgb888_bilinear(src, dw, dh)
    return resize_rgb888_nn(src, dw, dh)


def resize_rgb888_letterbox_kernel(src: np.ndarray, dw: int, dh: int, pad: int, kernel: str) -> Tuple[np.ndarray, float]:
    sh, sw, _ = src.shape
    dst = np.full((dh, dw, 3), pad, dtype=np.uint8)
    sx = dw / float(sw)
    sy = dh / float(sh)
    scale = min(sx, sy)
    if scale <= 0.0:
        return dst, 0.0
    scaled_w = int(sw * scale + 0.5)
    scaled_h = int(sh * scale + 0.5)
    scaled_w = max(1, min(dw, scaled_w))
    scaled_h = max(1, min(dh, scaled_h))
    off_x = (dw - scaled_w) // 2
    off_y = (dh - scaled_h) // 2
    tmp = resize_rgb888_with_kernel(src, scaled_w, scaled_h, kernel)
    dst[off_y : off_y + scaled_h, off_x : off_x + scaled_w] = tmp
    return dst, scale


def rgb_to_gray(rgb: np.ndarray) -> np.ndarray:
    return ((77 * rgb[:, :, 0].astype(np.uint16) +
             150 * rgb[:, :, 1].astype(np.uint16) +
             29 * rgb[:, :, 2].astype(np.uint16)) >> 8).astype(np.uint8)


def gray3_from_gray(gray: np.ndarray) -> np.ndarray:
    return np.stack([gray, gray, gray], axis=2).astype(np.uint8)


def median_filter_rgb(rgb: np.ndarray, ksize: int) -> np.ndarray:
    pad = ksize // 2
    h, w, _ = rgb.shape
    padded = np.pad(rgb, ((pad, pad), (pad, pad), (0, 0)), mode="edge")
    patches = []
    for dy in range(ksize):
        for dx in range(ksize):
            patches.append(padded[dy : dy + h, dx : dx + w, :])
    stack = np.stack(patches, axis=0)
    out = np.median(stack, axis=0)
    return np.clip(np.round(out), 0, 255).astype(np.uint8)


def gaussian3_rgb(rgb: np.ndarray) -> np.ndarray:
    p = np.pad(rgb.astype(np.uint16), ((1, 1), (1, 1), (0, 0)), mode="edge")
    out = (
        p[:-2, :-2] +
        (p[:-2, 1:-1] << 1) +
        p[:-2, 2:] +
        (p[1:-1, :-2] << 1) +
        (p[1:-1, 1:-1] << 2) +
        (p[1:-1, 2:] << 1) +
        p[2:, :-2] +
        (p[2:, 1:-1] << 1) +
        p[2:, 2:] + 8
    ) >> 4
    return np.clip(out, 0, 255).astype(np.uint8)


def gaussian3_gray(gray: np.ndarray) -> np.ndarray:
    p = np.pad(gray.astype(np.uint16), ((1, 1), (1, 1)), mode="edge")
    out = (
        p[:-2, :-2] +
        (p[:-2, 1:-1] << 1) +
        p[:-2, 2:] +
        (p[1:-1, :-2] << 1) +
        (p[1:-1, 1:-1] << 2) +
        (p[1:-1, 2:] << 1) +
        p[2:, :-2] +
        (p[2:, 1:-1] << 1) +
        p[2:, 2:] + 8
    ) >> 4
    return np.clip(out, 0, 255).astype(np.uint8)


def apply_luma_map(rgb: np.ndarray, y_new: np.ndarray) -> np.ndarray:
    y_old = rgb_to_gray(rgb)
    scale = np.ones_like(y_old, dtype=np.float32)
    nz = y_old > 0
    scale[nz] = y_new[nz].astype(np.float32) / y_old[nz].astype(np.float32)
    out = np.clip(np.round(rgb.astype(np.float32) * scale[:, :, None]), 0, 255).astype(np.uint8)
    out = np.where(nz[:, :, None], out, y_new[:, :, None].astype(np.uint8))
    return out


def mild_luma_sharpen(rgb: np.ndarray, gain: float = 0.20, thr: int = 4, limit: int = 6) -> np.ndarray:
    gray = rgb_to_gray(rgb)
    blur = gaussian3_gray(gray)
    delta = gray.astype(np.int16) - blur.astype(np.int16)
    adj = np.zeros_like(delta, dtype=np.int16)
    mask = np.abs(delta) >= thr
    adj[mask] = np.clip(np.round(delta[mask].astype(np.float32) * gain), -limit, limit).astype(np.int16)
    y_new = np.clip(gray.astype(np.int16) + adj, 0, 255).astype(np.uint8)
    return apply_luma_map(rgb, y_new)


def ocr_preprocess_rgb888(rgb: np.ndarray, mode: str) -> np.ndarray:
    mode = norm_mode(mode)
    if mode == "none":
        return rgb
    h, w, _ = rgb.shape
    if w <= 1 or h <= 1:
        return rgb

    if mode == "gray":
        return gray3_from_gray(rgb_to_gray(rgb))

    if mode == "median3":
        return median_filter_rgb(rgb, 3)

    if mode == "median5":
        return median_filter_rgb(rgb, 5)

    if mode == "gauss3":
        return gaussian3_rgb(rgb)

    if mode == "gauss3_sharp_mild":
        return mild_luma_sharpen(gaussian3_rgb(rgb), gain=0.20, thr=4, limit=6)

    if mode != "bin":
        raise ValueError(f"unsupported ocr preproc mode: {mode}")

    gray = rgb_to_gray(rgb)
    tmp = np.zeros_like(gray, dtype=np.uint8)
    for y in range(h):
        y0 = max(0, y - 1)
        y1 = min(h - 1, y + 1)
        for x in range(w):
            x0 = max(0, x - 1)
            x1 = min(w - 1, x + 1)
            roi = gray[y0 : y1 + 1, x0 : x1 + 1]
            tmp[y, x] = int(np.mean(roi, dtype=np.float32))

    outv = np.zeros_like(tmp, dtype=np.uint8)
    for y in range(h):
        y0 = max(0, y - 2)
        y1 = min(h - 1, y + 2)
        for x in range(w):
            x0 = max(0, x - 2)
            x1 = min(w - 1, x + 2)
            roi = tmp[y0 : y1 + 1, x0 : x1 + 1]
            thr = int(np.mean(roi, dtype=np.float32))
            outv[y, x] = 255 if int(tmp[y, x]) > (thr - 8) else 0
    return gray3_from_gray(outv)


def prepare_ocr_input_rgb888(
    crop_rgb: np.ndarray,
    in_w: int,
    in_h: int,
    resize_mode: str,
    resize_kernel: str,
    preproc: str,
    channel_order: str,
) -> Tuple[np.ndarray, float]:
    work = ocr_preprocess_rgb888(crop_rgb.copy(), preproc)
    if resize_mode == "letterbox":
        ocr_in, scale = resize_rgb888_letterbox_kernel(work, in_w, in_h, 0, resize_kernel)
        scaled_w = int(work.shape[1] * scale + 0.5)
        scaled_w = max(1, min(in_w, scaled_w))
        occ = scaled_w / float(in_w)
    else:
        ocr_in = resize_rgb888_with_kernel(work, in_w, in_h, resize_kernel)
        occ = 1.0

    if channel_order == "bgr":
        ocr_in = ocr_in[:, :, ::-1].copy()
    return ocr_in, occ


def save_ppm_rgb(path: Path, rgb: np.ndarray) -> None:
    img = Image.fromarray(rgb.astype(np.uint8), mode="RGB")
    img.save(path, format="PPM")


def parse_ccpd_bbox_from_name(image_name: str) -> Optional[Box]:
    stem = Path(image_name).stem
    parts = stem.split("-")
    if len(parts) < 3:
        return None
    bbox = parts[2]
    if "_" not in bbox:
        return None
    p1, p2 = bbox.split("_", 1)
    if "&" not in p1 or "&" not in p2:
        return None
    x1s, y1s = p1.split("&", 1)
    x2s, y2s = p2.split("&", 1)
    try:
        return Box(int(x1s), int(y1s), int(x2s), int(y2s))
    except ValueError:
        return None


def resolve_image_path(csv_path: Path, raw_path: str) -> str:
    p = Path(raw_path)
    if p.is_absolute():
        return str(p)
    return str((csv_path.parent / p).resolve())


def load_samples_from_csv(csv_path: Path) -> List[Sample]:
    out: List[Sample] = []
    with csv_path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        need = {"image_path", "x1", "y1", "x2", "y2"}
        miss = need - set(reader.fieldnames or [])
        if miss:
            raise ValueError(f"CSV missing columns: {sorted(miss)}")
        for row_idx, r in enumerate(reader):
            try:
                box = Box(int(r["x1"]), int(r["y1"]), int(r["x2"]), int(r["y2"]))
            except Exception:
                continue
            sample_id = str(r.get("sample_id", "") or row_idx).strip()
            scenario = str(r.get("scenario", "") or "").strip()
            plate_text = str(r.get("plate_text", "") or "").strip()
            image_path = resolve_image_path(csv_path, str(r["image_path"]).strip())
            out.append(Sample(sample_id, image_path, box, plate_text, scenario))
    return out


def load_samples_from_glob(glob_paths: Sequence[str], ccpd_auto: bool) -> List[Sample]:
    out: List[Sample] = []
    for p in glob_paths:
        path = Path(p)
        if not path.exists():
            continue
        files = sorted(path.rglob("*")) if path.is_dir() else [path]
        for f in files:
            if f.suffix.lower() not in {".jpg", ".jpeg", ".png", ".ppm", ".bmp"}:
                continue
            if not ccpd_auto:
                continue
            box = parse_ccpd_bbox_from_name(f.name)
            if box is None:
                continue
            out.append(Sample(f.stem, str(f.resolve()), box, "", ""))
    return out


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="Generate board-equivalent OCR input dumps (ocrin_*.ppm) on PC."
    )
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--ann-csv", type=Path, help="CSV with image_path,x1,y1,x2,y2 and optional sample_id,plate_text,scenario")
    src.add_argument(
        "--image-paths",
        nargs="+",
        help="Image file(s) or directory(ies). Use with --ccpd-auto to parse bbox from filename.",
    )

    ap.add_argument("--ccpd-auto", action="store_true", help="Parse CCPD bbox from filename for --image-paths")
    ap.add_argument("--out-dir", type=Path, required=True, help="Output directory")
    ap.add_argument("--max-samples", type=int, default=0, help="0 means all")
    ap.add_argument("--skip-existing", action="store_true", help="Skip rewriting ocrin if it already exists")

    ap.add_argument("--ocr-in-w", type=int, default=94)
    ap.add_argument("--ocr-in-h", type=int, default=24)
    ap.add_argument("--ocr-channel-order", choices=["rgb", "bgr"], default="bgr")
    ap.add_argument("--ocr-crop-mode", choices=["fixed", "box", "tight", "box-pad", "match"], default="match")
    ap.add_argument("--ocr-resize-mode", choices=["stretch", "letterbox"], default="letterbox")
    ap.add_argument("--ocr-resize-kernel", choices=["nn", "bilinear"], default="nn")
    ap.add_argument("--ocr-preproc", choices=PREPROC_CHOICES, default="none")
    ap.add_argument("--ocr-min-occ-ratio", type=float, default=0.0)

    ap.add_argument("--write-crop", action="store_true", help="Also write crop_*.ppm")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    out_dir: Path = args.out_dir
    out_dir.mkdir(parents=True, exist_ok=True)

    if args.ann_csv:
        samples = load_samples_from_csv(args.ann_csv)
    else:
        samples = load_samples_from_glob(args.image_paths, args.ccpd_auto)

    if args.max_samples > 0:
        samples = samples[: args.max_samples]

    idx_path = out_dir / "index.csv"
    ds_path = out_dir / "dataset.txt"
    preproc_mode = args.ocr_preproc

    with idx_path.open("w", encoding="utf-8", newline="") as f_idx, ds_path.open("w", encoding="utf-8") as f_ds:
        writer = csv.writer(f_idx)
        writer.writerow(
            [
                "sample_id",
                "image_path",
                "scenario",
                "preproc_mode",
                "preproc_ms",
                "box_x1",
                "box_y1",
                "box_x2",
                "box_y2",
                "crop_x1",
                "crop_y1",
                "crop_x2",
                "crop_y2",
                "occ_ratio",
                "crop_mode_final",
                "crop_path",
                "ocrin_path",
                "plate_text",
            ]
        )

        done = 0
        for i, sample in enumerate(samples):
            img_path = Path(sample.image_path)
            if not img_path.exists():
                continue
            try:
                rgb = np.array(Image.open(img_path).convert("RGB"), dtype=np.uint8)
            except Exception:
                continue
            h, w, _ = rgb.shape
            box = clamp_box(sample.box, w, h)
            crop = compute_ocr_crop_box(box, w, h, args.ocr_crop_mode)
            crop_mode_final = args.ocr_crop_mode

            occ = estimate_ocr_occ_ratio(crop.w, crop.h, args.ocr_in_w, args.ocr_in_h, args.ocr_resize_mode)
            if args.ocr_min_occ_ratio > 0 and occ < args.ocr_min_occ_ratio and args.ocr_crop_mode != "tight":
                if args.ocr_crop_mode == "match":
                    recrop = compute_match_ytrim_crop(crop, args.ocr_in_w, args.ocr_in_h, args.ocr_min_occ_ratio)
                    if recrop is not None:
                        crop = clamp_box(recrop, w, h)
                        crop_mode_final = "match-ytrim"
                else:
                    crop = compute_expand_crop_box(box, w, h, 0.08, 0.16)
                    crop_mode_final = "tight"
                occ = estimate_ocr_occ_ratio(crop.w, crop.h, args.ocr_in_w, args.ocr_in_h, args.ocr_resize_mode)

            crop_rgb = rgb[crop.y1 : crop.y2 + 1, crop.x1 : crop.x2 + 1, :]
            t0 = time.perf_counter()
            ocr_in, occ_exact = prepare_ocr_input_rgb888(
                crop_rgb,
                args.ocr_in_w,
                args.ocr_in_h,
                args.ocr_resize_mode,
                args.ocr_resize_kernel,
                preproc_mode,
                args.ocr_channel_order,
            )
            prep_ms = (time.perf_counter() - t0) * 1000.0

            ocrin_name = f"ocrin_{i:06d}.ppm"
            crop_name = f"crop_{i:06d}.ppm"
            ocrin_path = out_dir / ocrin_name
            crop_path = out_dir / crop_name

            if not (args.skip_existing and ocrin_path.exists()):
                save_ppm_rgb(ocrin_path, ocr_in)
            if args.write_crop:
                if not (args.skip_existing and crop_path.exists()):
                    save_ppm_rgb(crop_path, crop_rgb)
                crop_path_str = str(crop_path.resolve())
            else:
                crop_path_str = ""

            writer.writerow(
                [
                    sample.sample_id,
                    str(img_path.resolve()),
                    sample.scenario,
                    preproc_mode,
                    f"{prep_ms:.3f}",
                    box.x1,
                    box.y1,
                    box.x2,
                    box.y2,
                    crop.x1,
                    crop.y1,
                    crop.x2,
                    crop.y2,
                    f"{occ_exact:.4f}",
                    crop_mode_final,
                    crop_path_str,
                    str(ocrin_path.resolve()),
                    sample.plate_text,
                ]
            )
            f_ds.write(str(ocrin_path.resolve()) + "\n")
            done += 1

    print(f"done={done} out_dir={out_dir}")
    print(f"index={idx_path}")
    print(f"dataset={ds_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
