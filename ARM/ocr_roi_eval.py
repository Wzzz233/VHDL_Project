#!/usr/bin/env python3
"""
Evaluate OCR-only ROI experiment outputs.

Input CSV columns (from ocr_roi_batch):
  sample_id,preproc_mode,plate_text_gt,plate_text_pred,conf,blank_top1,
  preproc_ms,infer_ms,image_path,scenario
"""

from __future__ import annotations

import argparse
import csv
import json
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple


@dataclass
class PredRow:
    sample_id: str
    mode: str
    gt: str
    pred: str
    conf: Optional[float]
    blank_top1: Optional[float]
    preproc_ms: Optional[float]
    infer_ms: Optional[float]
    image_path: str
    scenario: str


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


def parse_float(s: str) -> Optional[float]:
    raw = (s or "").strip()
    if not raw:
        return None
    try:
        return float(raw)
    except ValueError:
        return None


def read_pred_rows(path: Path) -> List[PredRow]:
    rows: List[PredRow] = []
    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        need = {
            "sample_id",
            "preproc_mode",
            "plate_text_gt",
            "plate_text_pred",
            "conf",
            "blank_top1",
            "preproc_ms",
            "infer_ms",
            "image_path",
            "scenario",
        }
        got = set(reader.fieldnames or [])
        if not need.issubset(got):
            raise ValueError(f"missing columns in {path}: need={sorted(need)} got={sorted(got)}")
        for row in reader:
            rows.append(
                PredRow(
                    sample_id=(row.get("sample_id", "") or "").strip(),
                    mode=(row.get("preproc_mode", "") or "").strip(),
                    gt=norm_text(row.get("plate_text_gt", "") or ""),
                    pred=norm_text(row.get("plate_text_pred", "") or ""),
                    conf=parse_float(row.get("conf", "")),
                    blank_top1=parse_float(row.get("blank_top1", "")),
                    preproc_ms=parse_float(row.get("preproc_ms", "")),
                    infer_ms=parse_float(row.get("infer_ms", "")),
                    image_path=(row.get("image_path", "") or "").strip(),
                    scenario=(row.get("scenario", "") or "").strip(),
                )
            )
    return rows


def align_text(gt: str, pred: str) -> List[Tuple[str, str, str, Optional[int]]]:
    n = len(gt)
    m = len(pred)
    dp = [[0] * (m + 1) for _ in range(n + 1)]
    for i in range(1, n + 1):
        dp[i][0] = i
    for j in range(1, m + 1):
        dp[0][j] = j
    for i in range(1, n + 1):
        for j in range(1, m + 1):
            cost = 0 if gt[i - 1] == pred[j - 1] else 1
            dp[i][j] = min(
                dp[i - 1][j] + 1,
                dp[i][j - 1] + 1,
                dp[i - 1][j - 1] + cost,
            )

    ops: List[Tuple[str, str, str, Optional[int]]] = []
    i = n
    j = m
    while i > 0 or j > 0:
        if i > 0 and j > 0 and gt[i - 1] == pred[j - 1] and dp[i][j] == dp[i - 1][j - 1]:
            ops.append(("match", gt[i - 1], pred[j - 1], i - 1))
            i -= 1
            j -= 1
        elif i > 0 and j > 0 and dp[i][j] == dp[i - 1][j - 1] + 1:
            ops.append(("sub", gt[i - 1], pred[j - 1], i - 1))
            i -= 1
            j -= 1
        elif i > 0 and dp[i][j] == dp[i - 1][j] + 1:
            ops.append(("del", gt[i - 1], "", i - 1))
            i -= 1
        else:
            ops.append(("ins", "", pred[j - 1], None))
            j -= 1
    ops.reverse()
    return ops


def safe_div(num: float, den: float) -> float:
    return num / den if den else 0.0


def position_bucket(gt_pos: int) -> str:
    if gt_pos == 0:
        return "pos1_province"
    if gt_pos == 1:
        return "pos2_alpha"
    return "pos3plus_alnum"


def compute_metrics(rows: List[PredRow], include_examples: bool = True) -> Dict[str, object]:
    exact = 0
    total = 0
    total_gt_chars = 0
    correct_chars = 0
    substitutions = 0
    deletions = 0
    insertions = 0
    blank_vals: List[float] = []
    preproc_vals: List[float] = []
    infer_vals: List[float] = []
    sub_counter: Counter[Tuple[str, str]] = Counter()
    del_counter: Counter[str] = Counter()
    ins_counter: Counter[str] = Counter()
    pos_total: Counter[str] = Counter()
    pos_correct: Counter[str] = Counter()
    errors: List[Dict[str, str]] = []

    for row in rows:
        if not row.gt:
            continue
        total += 1
        if row.gt == row.pred:
            exact += 1
        total_gt_chars += len(row.gt)
        if row.blank_top1 is not None:
            blank_vals.append(row.blank_top1)
        if row.preproc_ms is not None:
            preproc_vals.append(row.preproc_ms)
        if row.infer_ms is not None:
            infer_vals.append(row.infer_ms)

        ops = align_text(row.gt, row.pred)
        for op, gt_ch, pred_ch, gt_pos in ops:
            if op in {"match", "sub", "del"} and gt_pos is not None:
                bucket = position_bucket(gt_pos)
                pos_total[bucket] += 1
                if op == "match":
                    pos_correct[bucket] += 1
            if op == "match":
                correct_chars += 1
            elif op == "sub":
                substitutions += 1
                sub_counter[(gt_ch, pred_ch)] += 1
            elif op == "del":
                deletions += 1
                del_counter[gt_ch] += 1
            elif op == "ins":
                insertions += 1
                ins_counter[pred_ch] += 1

        if include_examples and row.gt != row.pred:
            errors.append(
                {
                    "sample_id": row.sample_id,
                    "scenario": row.scenario,
                    "gt": row.gt,
                    "pred": row.pred,
                    "image_path": row.image_path,
                }
            )

    metrics = {
        "sample_count": total,
        "exact_plate_acc": safe_div(exact, total),
        "char_acc": safe_div(correct_chars, total_gt_chars),
        "deletion_rate": safe_div(deletions, total_gt_chars),
        "insertion_rate": safe_div(insertions, total_gt_chars),
        "substitution_rate": safe_div(substitutions, total_gt_chars),
        "blank_top1_mean": safe_div(sum(blank_vals), len(blank_vals)) if blank_vals else None,
        "avg_preproc_ms": safe_div(sum(preproc_vals), len(preproc_vals)) if preproc_vals else None,
        "avg_infer_ms": safe_div(sum(infer_vals), len(infer_vals)) if infer_vals else None,
        "position_accuracy": {
            key: safe_div(pos_correct[key], pos_total[key])
            for key in ["pos1_province", "pos2_alpha", "pos3plus_alnum"]
        },
        "top_substitutions": [[a, b, c] for (a, b), c in sub_counter.most_common(15)],
        "top_deletions": [[a, c] for a, c in del_counter.most_common(15)],
        "top_insertions": [[a, c] for a, c in ins_counter.most_common(15)],
        "error_examples": errors[:20],
        "_sub_counter": sub_counter,
        "_del_counter": del_counter,
        "_ins_counter": ins_counter,
    }
    return metrics


def strip_internal_counters(metrics: Dict[str, object]) -> Dict[str, object]:
    out = dict(metrics)
    out.pop("_sub_counter", None)
    out.pop("_del_counter", None)
    out.pop("_ins_counter", None)
    return out


def compute_mode_report(rows: List[PredRow], sanity_prefix: str) -> Dict[str, object]:
    sanity_prefix = (sanity_prefix or "").lower()
    if sanity_prefix:
        sanity_rows = [r for r in rows if (r.scenario or "").lower().startswith(sanity_prefix)]
        main_rows = [r for r in rows if not (r.scenario or "").lower().startswith(sanity_prefix)]
    else:
        sanity_rows = []
        main_rows = list(rows)
    if not main_rows:
        main_rows = list(rows)

    by_scenario = {}
    scenarios = sorted({r.scenario or "unknown" for r in rows})
    for scen in scenarios:
        scen_rows = [r for r in rows if (r.scenario or "unknown") == scen]
        by_scenario[scen] = strip_internal_counters(compute_metrics(scen_rows, include_examples=False))

    overall = compute_metrics(rows)
    main = compute_metrics(main_rows)
    sanity = compute_metrics(sanity_rows) if sanity_rows else None
    conf_basis = main if main_rows else overall

    return {
        "mode": rows[0].mode if rows else "unknown",
        "overall": strip_internal_counters(overall),
        "main": strip_internal_counters(main),
        "sanity": strip_internal_counters(sanity) if sanity else None,
        "by_scenario": by_scenario,
        "confusion_basis": {
            "top_substitutions": [[a, b, c] for (a, b), c in conf_basis["_sub_counter"].most_common(50)],
            "top_deletions": [[a, c] for a, c in conf_basis["_del_counter"].most_common(50)],
            "top_insertions": [[a, c] for a, c in conf_basis["_ins_counter"].most_common(50)],
        },
    }


def write_confusions_csv(path: Path, report: Dict[str, object]) -> None:
    basis = report["confusion_basis"]
    with path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["type", "src", "dst", "count"])
        for src, dst, count in basis["top_substitutions"]:
            writer.writerow(["substitution", src, dst, count])
        for src, count in basis["top_deletions"]:
            writer.writerow(["deletion", src, "", count])
        for dst, count in basis["top_insertions"]:
            writer.writerow(["insertion", "", dst, count])


def better_rank_key(report: Dict[str, object]) -> Tuple[float, float, float]:
    main = report["main"]
    prep = main["avg_preproc_ms"] if main["avg_preproc_ms"] is not None else 1e9
    return (-main["exact_plate_acc"], -main["char_acc"], prep)


def choose_best(reports: List[Dict[str, object]]) -> Optional[Dict[str, object]]:
    if not reports:
        return None
    ranked = sorted(reports, key=better_rank_key)
    top_exact = ranked[0]["main"]["exact_plate_acc"]
    tied = [r for r in ranked if (top_exact - r["main"]["exact_plate_acc"]) < 0.01]
    tied.sort(key=lambda r: (-r["main"]["char_acc"], r["main"]["avg_preproc_ms"] or 1e9, -r["main"]["exact_plate_acc"]))
    return tied[0]


def qualifies_against_raw(report: Dict[str, object], raw_report: Optional[Dict[str, object]]) -> bool:
    if not raw_report or report["mode"] == "raw":
        return False
    main = report["main"]
    raw_main = raw_report["main"]
    exact_gain = main["exact_plate_acc"] - raw_main["exact_plate_acc"]
    char_gain = main["char_acc"] - raw_main["char_acc"]
    sanity = report.get("sanity")
    raw_sanity = raw_report.get("sanity")
    sanity_ok = True
    if sanity and raw_sanity:
        sanity_ok = sanity["exact_plate_acc"] >= (raw_sanity["exact_plate_acc"] - 0.01)
    return ((exact_gain >= 0.03) or (char_gain >= 0.015)) and sanity_ok


def recommendation_for(mode: str, avg_preproc_ms: Optional[float], qualifies: bool) -> str:
    if not qualifies:
        return "stop_preproc"
    if mode in {"gray3", "median3"}:
        return "arm_roi_first"
    if mode == "gauss3":
        return "arm_roi_first_then_recheck"
    if mode in {"median5", "gauss3_sharp_mild"} and (avg_preproc_ms or 0.0) >= 0.5:
        return "candidate_fpga_roi_after_arm"
    return "arm_roi_first"


def write_summary_markdown(path: Path, reports: List[Dict[str, object]], raw_report: Optional[Dict[str, object]], best: Optional[Dict[str, object]]) -> None:
    lines: List[str] = []
    lines.append("# OCR ROI Denoise Experiment Summary")
    lines.append("")
    lines.append("| mode | main exact | main char | sanity exact | del rate | prep ms | infer ms | note |")
    lines.append("| --- | ---: | ---: | ---: | ---: | ---: | ---: | --- |")

    raw_main_exact = raw_report["main"]["exact_plate_acc"] if raw_report else None
    raw_main_char = raw_report["main"]["char_acc"] if raw_report else None
    raw_sanity_exact = raw_report["sanity"]["exact_plate_acc"] if raw_report and raw_report.get("sanity") else None

    for report in sorted(reports, key=better_rank_key):
        mode = report["mode"]
        main = report["main"]
        sanity = report.get("sanity")
        qualifies = qualifies_against_raw(report, raw_report)
        note = recommendation_for(mode, main["avg_preproc_ms"], qualifies) if mode != "raw" else "baseline"
        lines.append(
            "| {mode} | {exact:.4f} | {char:.4f} | {sanity_exact} | {delr:.4f} | {prep} | {infer} | {note} |".format(
                mode=mode,
                exact=main["exact_plate_acc"],
                char=main["char_acc"],
                sanity_exact=(f"{sanity['exact_plate_acc']:.4f}" if sanity else "-"),
                delr=main["deletion_rate"],
                prep=(f"{main['avg_preproc_ms']:.3f}" if main["avg_preproc_ms"] is not None else "-"),
                infer=(f"{main['avg_infer_ms']:.3f}" if main["avg_infer_ms"] is not None else "-"),
                note=note,
            )
        )

    if best:
        lines.append("")
        lines.append(f"Best candidate: `{best['mode']}`")
        if raw_report and best["mode"] != "raw":
            exact_gain = best["main"]["exact_plate_acc"] - (raw_main_exact or 0.0)
            char_gain = best["main"]["char_acc"] - (raw_main_char or 0.0)
            qualifies = qualifies_against_raw(best, raw_report)
            lines.append(f"- main exact gain vs raw: {exact_gain:+.4f}")
            lines.append(f"- main char gain vs raw: {char_gain:+.4f}")
            lines.append(f"- recommended next step: {recommendation_for(best['mode'], best['main']['avg_preproc_ms'], qualifies)}")
    else:
        lines.append("")
        lines.append("Best candidate: none")

    path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def main() -> int:
    ap = argparse.ArgumentParser(description="Evaluate OCR ROI denoise experiment outputs.")
    ap.add_argument("--pred", action="append", required=True, help="prediction csv from ocr_roi_batch; repeat for each mode")
    ap.add_argument("--out-dir", required=True, help="output directory for reports")
    ap.add_argument("--sanity-prefix", default="sanity", help="scenario prefix treated as sanity subset")
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    reports: List[Dict[str, object]] = []
    for pred_path in args.pred:
        rows = read_pred_rows(Path(pred_path))
        if not rows:
            continue
        report = compute_mode_report(rows, args.sanity_prefix)
        mode = report["mode"]
        report_path = out_dir / f"report_{mode}.json"
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding="utf-8")
        write_confusions_csv(out_dir / f"confusions_{mode}.csv", report)
        reports.append(report)

    raw_report = next((r for r in reports if r["mode"] == "raw"), None)
    best = choose_best(reports)
    write_summary_markdown(out_dir / "summary_ranked.md", reports, raw_report, best)

    print(json.dumps({
        "report_count": len(reports),
        "best_mode": None if not best else best["mode"],
        "summary_markdown": str((out_dir / "summary_ranked.md").resolve()),
    }, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
