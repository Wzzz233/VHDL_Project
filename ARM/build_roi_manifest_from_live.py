#!/usr/bin/env python3
"""
Build ROI manifest candidates from board-side live capture outputs.

Inputs:
- pred.csv from run_lpr_kms.sh --pred-log
- capture.log containing [fpga-prep] dump=... frame=... path=...
- optional OCR dump index.csv from --ocr-crop-dump-dir

Output columns are compatible with the ROI denoise experiment manifest:
  sample_id,image_path,x1,y1,x2,y2,plate_text,scenario
Additional review columns are appended for convenience.
"""

from __future__ import annotations

import argparse
import csv
import re
from pathlib import Path
from typing import Dict, List, Optional, Tuple


DUMP_RE = re.compile(r"\[fpga-prep\]\s+dump=\d+/\d+\s+mode=([^\s]+)\s+frame=(\d+)\s+path=(.+)$")


def read_frame_dump_map(log_path: Path) -> Dict[str, Dict[str, str]]:
    out: Dict[str, Dict[str, str]] = {}
    with log_path.open("r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            m = DUMP_RE.search(line.strip())
            if not m:
                continue
            mode, frame_id, path = m.groups()
            out[frame_id] = {
                "dump_mode": mode,
                "image_path": path.strip(),
            }
    return out


def read_pred_rows(path: Path) -> List[Dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        need = {"frame_id", "plate_text_pred", "conf", "x1", "y1", "x2", "y2", "ts_us"}
        got = set(reader.fieldnames or [])
        if not need.issubset(got):
            raise ValueError(f"pred csv missing columns: need={sorted(need)} got={sorted(got)}")
        return list(reader)


def read_ocr_index(path: Optional[Path]) -> Dict[Tuple[str, str, str, str, str], Dict[str, str]]:
    out: Dict[Tuple[str, str, str, str, str], Dict[str, str]] = {}
    if not path or not path.exists():
        return out
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        need = {
            "frame_id",
            "box_x1",
            "box_y1",
            "box_x2",
            "box_y2",
            "crop_path",
            "ocr_input_path",
            "app_text",
            "app_conf",
        }
        got = set(reader.fieldnames or [])
        if not need.issubset(got):
            raise ValueError(f"ocr index missing columns: need={sorted(need)} got={sorted(got)}")
        for row in reader:
            key = (
                (row.get("frame_id", "") or "").strip(),
                (row.get("box_x1", "") or "").strip(),
                (row.get("box_y1", "") or "").strip(),
                (row.get("box_x2", "") or "").strip(),
                (row.get("box_y2", "") or "").strip(),
            )
            out[key] = row
    return out


def safe_int_str(v: str) -> str:
    return str(int(float(v)))


def main() -> int:
    ap = argparse.ArgumentParser(description="Build roi_manifest_candidates.csv from live capture outputs.")
    ap.add_argument("--pred", required=True, help="pred csv from run_lpr_kms.sh --pred-log")
    ap.add_argument("--capture-log", required=True, help="tee log containing [fpga-prep] dump lines")
    ap.add_argument("--ocr-index", default="", help="optional ocr crop dump index.csv")
    ap.add_argument("--out-csv", required=True, help="output candidate manifest csv")
    ap.add_argument("--scenario", default="capture_live", help="scenario label to write into candidate rows")
    ap.add_argument("--plate-text-default", default="", help="initial GT text value; usually keep empty")
    args = ap.parse_args()

    pred_rows = read_pred_rows(Path(args.pred))
    frame_dump_map = read_frame_dump_map(Path(args.capture_log))
    ocr_index = read_ocr_index(Path(args.ocr_index)) if args.ocr_index else {}

    out_path = Path(args.out_csv)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    emitted = 0
    skipped_no_dump = 0
    seen_per_frame: Dict[str, int] = {}
    with out_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow([
            "sample_id",
            "image_path",
            "x1",
            "y1",
            "x2",
            "y2",
            "plate_text",
            "scenario",
            "frame_id",
            "plate_text_pred",
            "plate_type_pred",
            "conf",
            "ts_us",
            "dump_mode",
            "crop_path",
            "ocr_input_path",
            "review_status",
        ])

        for row in pred_rows:
            frame_id = (row.get("frame_id", "") or "").strip()
            dump = frame_dump_map.get(frame_id)
            if not dump:
                skipped_no_dump += 1
                continue
            x1 = safe_int_str(row.get("x1", "0"))
            y1 = safe_int_str(row.get("y1", "0"))
            x2 = safe_int_str(row.get("x2", "0"))
            y2 = safe_int_str(row.get("y2", "0"))
            seen_per_frame[frame_id] = seen_per_frame.get(frame_id, 0) + 1
            key = (frame_id, x1, y1, x2, y2)
            crop_meta = ocr_index.get(key, {})
            sample_id = f"f{int(frame_id):06d}_p{seen_per_frame[frame_id]:02d}"
            writer.writerow([
                sample_id,
                dump["image_path"],
                x1,
                y1,
                x2,
                y2,
                args.plate_text_default,
                args.scenario,
                frame_id,
                (row.get("plate_text_pred", "") or "").strip(),
                (row.get("plate_type_pred", "") or "").strip(),
                (row.get("conf", "") or "").strip(),
                (row.get("ts_us", "") or "").strip(),
                dump.get("dump_mode", ""),
                (crop_meta.get("crop_path", "") or "").strip(),
                (crop_meta.get("ocr_input_path", "") or "").strip(),
                "todo",
            ])
            emitted += 1

    print(f"pred_rows={len(pred_rows)}")
    print(f"frame_dumps={len(frame_dump_map)}")
    print(f"emitted={emitted}")
    print(f"skipped_no_dump={skipped_no_dump}")
    print(f"out_csv={out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
