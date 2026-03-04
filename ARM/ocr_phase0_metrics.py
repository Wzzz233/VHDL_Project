#!/usr/bin/env python3
"""
Phase0 OCR quick metrics from prediction CSV.

CSV columns (from fpga_lpr_display --pred-log):
  frame_id,plate_text_pred,plate_type_pred,conf,x1,y1,x2,y2,ts_us
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter
from pathlib import Path
from typing import Dict, List


def norm_text(s: str) -> str:
    if s is None:
        return ""
    out = []
    for ch in s.strip():
        if ch in {" ", "\t", "-", "_"}:
            continue
        if "a" <= ch <= "z":
            out.append(chr(ord(ch) - 32))
        else:
            out.append(ch)
    return "".join(out)


def read_pred_rows(path: Path) -> List[Dict[str, str]]:
    with path.open("r", encoding="utf-8", newline="") as f:
        r = csv.DictReader(f)
        need = {"frame_id", "plate_text_pred", "conf"}
        got = set(r.fieldnames or [])
        if not need.issubset(got):
            raise ValueError(f"missing columns: need={sorted(need)} got={sorted(got)}")
        return list(r)


def main() -> int:
    ap = argparse.ArgumentParser(description="Compute Phase0 OCR metrics from pred.csv")
    ap.add_argument("--pred", required=True, help="prediction csv path")
    ap.add_argument("--gt-text", default="", help="ground-truth plate text for exact-match ratio")
    ap.add_argument("--out-json", default="", help="optional output json path")
    args = ap.parse_args()

    rows = read_pred_rows(Path(args.pred))
    pred_rows = len(rows)
    gt = norm_text(args.gt_text) if args.gt_text else ""

    nonempty = 0
    len_ge_7 = 0
    exact = 0
    texts: List[str] = []
    blank_vals: List[float] = []
    by_profile: Dict[str, Dict[str, object]] = {}

    for row in rows:
        t_raw = row.get("plate_text_pred", "")
        t = norm_text(t_raw)
        prep_profile = (row.get("prep_profile", "") or "unknown").strip() or "unknown"
        if prep_profile not in by_profile:
            by_profile[prep_profile] = {"n": 0, "nonempty": 0, "len7": 0}
        by_profile[prep_profile]["n"] += 1

        if t:
            nonempty += 1
            if len(t) >= 7:
                len_ge_7 += 1
            if len(t) >= 7:
                by_profile[prep_profile]["len7"] += 1
            by_profile[prep_profile]["nonempty"] += 1
            if gt and t == gt:
                exact += 1
            texts.append(t)
        b = (row.get("blank_top1", "") or "").strip()
        if b:
            try:
                blank_vals.append(float(b))
            except ValueError:
                pass

    denom = pred_rows if pred_rows > 0 else 1
    summary = {
        "pred_rows": pred_rows,
        "ocr_nonempty_ratio": nonempty / denom,
        "len_ge_7_ratio": len_ge_7 / denom,
        "exact_match_ratio": (exact / denom) if gt else None,
        "blank_top1_mean": (sum(blank_vals) / len(blank_vals)) if blank_vals else None,
        "gt_text": gt if gt else None,
        "top_texts": Counter(texts).most_common(10),
    }

    if by_profile:
        profile_summary: Dict[str, Dict[str, object]] = {}
        for k, v in sorted(by_profile.items()):
            n = int(v["n"]) if int(v["n"]) > 0 else 1
            profile_summary[k] = {
                "count": int(v["n"]),
                "ocr_nonempty_ratio": float(v["nonempty"]) / n,
                "len_ge_7_ratio": float(v["len7"]) / n,
            }
        summary["by_prep_profile"] = profile_summary

    print(json.dumps(summary, ensure_ascii=False, indent=2))
    if args.out_json:
        out = Path(args.out_json)
        out.parent.mkdir(parents=True, exist_ok=True)
        out.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
