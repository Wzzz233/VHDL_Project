#!/usr/bin/env python3
"""
Evaluate plate detection/OCR/type predictions against GT CSV.

GT columns:
  frame_id,plate_text_gt,plate_type_gt,x1,y1,x2,y2,scenario

Prediction columns:
  frame_id,plate_text_pred,plate_type_pred,conf,x1,y1,x2,y2
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple


@dataclass
class BoxRow:
    frame_id: str
    text: str
    plate_type: str
    x1: int
    y1: int
    x2: int
    y2: int
    scenario: str = ""


def safe_int(v: str) -> int:
    return int(float(v))


def iou(a: BoxRow, b: BoxRow) -> float:
    x1 = max(a.x1, b.x1)
    y1 = max(a.y1, b.y1)
    x2 = min(a.x2, b.x2)
    y2 = min(a.y2, b.y2)
    if x2 < x1 or y2 < y1:
        return 0.0
    inter = (x2 - x1 + 1) * (y2 - y1 + 1)
    area_a = (a.x2 - a.x1 + 1) * (a.y2 - a.y1 + 1)
    area_b = (b.x2 - b.x1 + 1) * (b.y2 - b.y1 + 1)
    denom = area_a + area_b - inter
    if denom <= 0:
        return 0.0
    return inter / denom


def read_gt(path: Path) -> List[BoxRow]:
    rows: List[BoxRow] = []
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        needed = {"frame_id", "plate_text_gt", "plate_type_gt", "x1", "y1", "x2", "y2", "scenario"}
        if not needed.issubset(set(reader.fieldnames or [])):
            raise ValueError(f"GT missing columns. need={sorted(needed)} got={reader.fieldnames}")
        for r in reader:
            rows.append(
                BoxRow(
                    frame_id=str(r["frame_id"]).strip(),
                    text=str(r["plate_text_gt"]).strip(),
                    plate_type=str(r["plate_type_gt"]).strip(),
                    x1=safe_int(r["x1"]),
                    y1=safe_int(r["y1"]),
                    x2=safe_int(r["x2"]),
                    y2=safe_int(r["y2"]),
                    scenario=str(r["scenario"]).strip(),
                )
            )
    return rows


def read_pred(path: Path) -> List[BoxRow]:
    rows: List[BoxRow] = []
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        needed = {"frame_id", "plate_text_pred", "plate_type_pred", "x1", "y1", "x2", "y2"}
        if not needed.issubset(set(reader.fieldnames or [])):
            raise ValueError(f"Prediction missing columns. need={sorted(needed)} got={reader.fieldnames}")
        for r in reader:
            rows.append(
                BoxRow(
                    frame_id=str(r["frame_id"]).strip(),
                    text=str(r["plate_text_pred"]).strip(),
                    plate_type=str(r["plate_type_pred"]).strip(),
                    x1=safe_int(r["x1"]),
                    y1=safe_int(r["y1"]),
                    x2=safe_int(r["x2"]),
                    y2=safe_int(r["y2"]),
                )
            )
    return rows


def group_by_frame(rows: List[BoxRow]) -> Dict[str, List[BoxRow]]:
    out: Dict[str, List[BoxRow]] = defaultdict(list)
    for r in rows:
        out[r.frame_id].append(r)
    return out


def prf(tp: int, fp: int, fn: int) -> Tuple[float, float, float]:
    prec = tp / (tp + fp) if (tp + fp) else 0.0
    rec = tp / (tp + fn) if (tp + fn) else 0.0
    f1 = 2 * prec * rec / (prec + rec) if (prec + rec) else 0.0
    return prec, rec, f1


def evaluate(gt_rows: List[BoxRow], pred_rows: List[BoxRow], iou_thr: float) -> dict:
    gt_by_frame = group_by_frame(gt_rows)
    pred_by_frame = group_by_frame(pred_rows)
    all_frames = sorted(set(gt_by_frame.keys()) | set(pred_by_frame.keys()))

    tp = fp = fn = 0
    text_ok = type_ok = 0
    matched_total = 0

    scenario_stat: Dict[str, dict] = defaultdict(lambda: {"tp": 0, "fp": 0, "fn": 0, "text_ok": 0, "type_ok": 0})
    type_stat: Dict[str, dict] = defaultdict(lambda: {"gt": 0, "tp": 0, "text_ok": 0})

    for fid in all_frames:
        gts = gt_by_frame.get(fid, [])
        preds = pred_by_frame.get(fid, [])
        pairs: List[Tuple[float, int, int]] = []
        for gi, g in enumerate(gts):
            for pi, p in enumerate(preds):
                v = iou(g, p)
                if v >= iou_thr:
                    pairs.append((v, gi, pi))
        pairs.sort(reverse=True, key=lambda x: x[0])

        used_g = set()
        used_p = set()
        matched: List[Tuple[int, int]] = []
        for _, gi, pi in pairs:
            if gi in used_g or pi in used_p:
                continue
            used_g.add(gi)
            used_p.add(pi)
            matched.append((gi, pi))

        local_tp = len(matched)
        local_fn = len(gts) - local_tp
        local_fp = len(preds) - local_tp

        tp += local_tp
        fn += local_fn
        fp += local_fp
        matched_total += local_tp

        for g in gts:
            type_stat[g.plate_type]["gt"] += 1

        for gi, pi in matched:
            g = gts[gi]
            p = preds[pi]
            scen = g.scenario or "unknown"
            scenario_stat[scen]["tp"] += 1
            type_stat[g.plate_type]["tp"] += 1
            if g.text == p.text:
                text_ok += 1
                scenario_stat[scen]["text_ok"] += 1
                type_stat[g.plate_type]["text_ok"] += 1
            if g.plate_type == p.plate_type:
                type_ok += 1
                scenario_stat[scen]["type_ok"] += 1

        for gi, g in enumerate(gts):
            scen = g.scenario or "unknown"
            if gi not in used_g:
                scenario_stat[scen]["fn"] += 1
        if local_fp > 0:
            scenario_stat["unknown_pred"]["fp"] += local_fp

    precision, recall, f1 = prf(tp, fp, fn)
    text_acc = text_ok / matched_total if matched_total else 0.0
    type_acc = type_ok / matched_total if matched_total else 0.0

    per_scenario = {}
    for scen, s in sorted(scenario_stat.items()):
        s_tp = int(round(s["tp"]))
        s_fp = int(round(s["fp"]))
        s_fn = int(round(s["fn"]))
        sp, sr, sf = prf(s_tp, s_fp, s_fn)
        mt = s_tp
        per_scenario[scen] = {
            "precision": sp,
            "recall": sr,
            "f1": sf,
            "text_acc_on_matched": (s["text_ok"] / mt) if mt else 0.0,
            "type_acc_on_matched": (s["type_ok"] / mt) if mt else 0.0,
            "tp": s_tp,
            "fp": s_fp,
            "fn": s_fn,
        }

    per_plate_type = {}
    for t, s in sorted(type_stat.items()):
        gt_n = int(s["gt"])
        tp_n = int(s["tp"])
        per_plate_type[t] = {
            "gt_count": gt_n,
            "matched_count": tp_n,
            "recall": (tp_n / gt_n) if gt_n else 0.0,
            "text_acc_on_matched": (s["text_ok"] / tp_n) if tp_n else 0.0,
        }

    return {
        "summary": {
            "gt_total": len(gt_rows),
            "pred_total": len(pred_rows),
            "matched_total": matched_total,
            "tp": tp,
            "fp": fp,
            "fn": fn,
            "precision": precision,
            "recall": recall,
            "f1": f1,
            "text_acc_on_matched": text_acc,
            "type_acc_on_matched": type_acc,
            "iou_threshold": iou_thr,
        },
        "per_scenario": per_scenario,
        "per_plate_type": per_plate_type,
    }


def write_markdown(report: dict, path: Path) -> None:
    s = report["summary"]
    lines = [
        "# LPR Evaluation Summary",
        "",
        "## Overall",
        "",
        "| metric | value |",
        "|---|---:|",
        f"| gt_total | {s['gt_total']} |",
        f"| pred_total | {s['pred_total']} |",
        f"| matched_total | {s['matched_total']} |",
        f"| precision | {s['precision']:.4f} |",
        f"| recall | {s['recall']:.4f} |",
        f"| f1 | {s['f1']:.4f} |",
        f"| text_acc_on_matched | {s['text_acc_on_matched']:.4f} |",
        f"| type_acc_on_matched | {s['type_acc_on_matched']:.4f} |",
        "",
        "## Per Scenario",
        "",
        "| scenario | precision | recall | f1 | text_acc | type_acc |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for scen, m in sorted(report["per_scenario"].items()):
        lines.append(
            f"| {scen} | {m['precision']:.4f} | {m['recall']:.4f} | {m['f1']:.4f} | "
            f"{m['text_acc_on_matched']:.4f} | {m['type_acc_on_matched']:.4f} |"
        )
    lines.extend(
        [
            "",
            "## Per Plate Type",
            "",
            "| plate_type | gt_count | matched_count | recall | text_acc_on_matched |",
            "|---|---:|---:|---:|---:|",
        ]
    )
    for t, m in sorted(report["per_plate_type"].items()):
        lines.append(
            f"| {t} | {m['gt_count']} | {m['matched_count']} | {m['recall']:.4f} | {m['text_acc_on_matched']:.4f} |"
        )
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--gt", required=True, help="GT csv path")
    parser.add_argument("--pred", required=True, help="Prediction csv path")
    parser.add_argument("--out-json", default="/tmp/score/eval_summary.json")
    parser.add_argument("--out-md", default="", help="Optional markdown summary path")
    parser.add_argument("--iou-thr", type=float, default=0.30)
    args = parser.parse_args()

    gt_rows = read_gt(Path(args.gt))
    pred_rows = read_pred(Path(args.pred))
    report = evaluate(gt_rows, pred_rows, args.iou_thr)

    out_json = Path(args.out_json)
    out_json.parent.mkdir(parents=True, exist_ok=True)
    out_json.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    if args.out_md:
        out_md = Path(args.out_md)
        out_md.parent.mkdir(parents=True, exist_ok=True)
        write_markdown(report, out_md)

    s = report["summary"]
    print(
        "eval done:",
        f"precision={s['precision']:.4f}",
        f"recall={s['recall']:.4f}",
        f"f1={s['f1']:.4f}",
        f"text_acc={s['text_acc_on_matched']:.4f}",
        f"type_acc={s['type_acc_on_matched']:.4f}",
    )
    print("json:", out_json)
    if args.out_md:
        print("md:", args.out_md)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
