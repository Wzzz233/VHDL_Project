#!/usr/bin/env python3
"""Run v5.1 timing-closure multiseed and snapshot artifacts.

This script automates:
1) P0 baseline snapshot from current report_timing outputs.
2) P1 implementation-only multiseed runs (no RTL/FDC edit).
3) Parsing key timing metrics and FDC syntax gate counts.
4) Top50 slow-max path ownership classification.
"""

from __future__ import annotations

import argparse
import csv
import datetime as dt
import os
import re
import shutil
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Tuple


PDS_SHELL = Path(r"C:\pango\PDS_2022.2-SP6.4\bin\pds_shell.exe")
TOP_MODULE = "pcie_dma_ddr3_cam1"
RPT_NAME = "pcie_dma_ddr3_cam1.rtr"

FDC_GATE_PATTERNS = [
    "Timing-4003",
    "CommandTiming-0057",
    "SDC-2023",
    "ConstraintEditor-0046",
    "SDC-2017",
]


@dataclass
class PdsInputs:
    design_files: List[str]
    ip_idf_files: List[str]
    fdc_file: str


@dataclass
class TimingRow:
    wns: float
    tns: float
    failing_endpoints: int
    total_endpoints: int


@dataclass
class KeyMetrics:
    pclk_pclk_wns: Optional[float]
    ddrphy_ddrphy_wns: Optional[float]
    pclk_div2_wns: Optional[float]
    pclk_div2_tns: Optional[float]
    pclk_div2_fe: Optional[int]
    ref_to_div2_fe: Optional[int]
    div2_to_ref_fe: Optional[int]
    fdc_gate_total: int


def read_text(path: Path) -> str:
    return path.read_text(encoding="utf-8", errors="ignore")


def write_text(path: Path, content: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(content, encoding="utf-8")


def unique_keep_order(items: Iterable[str]) -> List[str]:
    seen = set()
    out = []
    for item in items:
        if item not in seen:
            seen.add(item)
            out.append(item)
    return out


def parse_pds_inputs(pds_path: Path) -> PdsInputs:
    text = read_text(pds_path)
    lines = text.splitlines()
    section: Optional[str] = None
    design_files: List[str] = []
    ip_idf_files: List[str] = []
    fdc_files: List[str] = []

    for line in lines:
        if "(_widget wgt_my_design_src" in line:
            section = "design"
            continue
        if "(_widget wgt_my_ips_src" in line:
            section = "ips"
            continue
        if "(_widget wgt_import_logic_con_file" in line:
            section = "fdc"
            continue
        if section and "(_widget " in line and "(_widget wgt_" in line:
            if (
                "wgt_my_design_src" not in line
                and "wgt_my_ips_src" not in line
                and "wgt_import_logic_con_file" not in line
            ):
                section = None

        if section == "design":
            m = re.search(r'\(_file "([^"]+)"', line)
            if m:
                design_files.append(m.group(1))
        elif section == "ips":
            m = re.search(r'\(_ip "([^"]+\.idf)"', line)
            if m:
                ip_idf_files.append(m.group(1))
        elif section == "fdc":
            m = re.search(r'\(_file "([^"]+\.fdc)"', line)
            if m:
                fdc_files.append(m.group(1))

    design_files = unique_keep_order(design_files)
    ip_idf_files = unique_keep_order(ip_idf_files)
    fdc_files = unique_keep_order(fdc_files)
    if not design_files:
        raise RuntimeError(f"No design files found in {pds_path}")
    if not ip_idf_files:
        raise RuntimeError(f"No IP .idf files found in {pds_path}")
    if not fdc_files:
        raise RuntimeError(f"No FDC file found in {pds_path}")

    return PdsInputs(
        design_files=design_files,
        ip_idf_files=ip_idf_files,
        fdc_file=fdc_files[0],
    )


def build_flow_commands(root: Path, run_dir: Path, seed: int, pds_inputs: PdsInputs) -> str:
    rel_run = run_dir.relative_to(root).as_posix()
    lines: List[str] = []
    lines.append(f'set_dir -impl_directory "{rel_run}"')
    lines.append("set_arch -family Logos2 -device PG2L50H -speedgrade -6 -package FBG484")
    for f in pds_inputs.design_files:
        lines.append(f'add_design "{f}"')
    for ip in pds_inputs.ip_idf_files:
        lines.append(f'add_design "{ip}"')
    lines.append(f'add_constraint "{pds_inputs.fdc_file}"')
    lines.append(f"compile -top_module {TOP_MODULE}")
    lines.append("synthesize -ads -selected_syn_tool_opt 2")
    lines.append("dev_map")
    lines.append(
        f"pnr -gplace_seed {seed} -groute_seed {seed + 1000} "
        "-fix_hold_violation_in_route TRUE"
    )
    lines.append("report_timing -force_to_run")
    lines.append("exit")
    return "\n".join(lines) + "\n"


def parse_setup_summary(rtr_text: str) -> Dict[Tuple[str, str], TimingRow]:
    rows: Dict[Tuple[str, str], TimingRow] = {}
    lines = rtr_text.splitlines()
    in_slow_setup = False
    after_header = False
    for line in lines:
        if line.strip() == "Setup Summary(Slow Corner):":
            in_slow_setup = True
            after_header = False
            continue
        if in_slow_setup and line.startswith("===================================================================================================="):
            break
        if not in_slow_setup:
            continue
        if line.startswith(" Launch Clock"):
            after_header = True
            continue
        if not after_header:
            continue
        if line.startswith("----") or not line.strip():
            continue
        m = re.match(
            r"^\s*(\S+)\s+(\S+)\s+(-?\d+\.\d+)\s+(-?\d+\.\d+)\s+(\d+)\s+(\d+)\s*$",
            line,
        )
        if not m:
            continue
        launch, capture, wns, tns, fe, te = m.groups()
        rows[(launch, capture)] = TimingRow(
            wns=float(wns),
            tns=float(tns),
            failing_endpoints=int(fe),
            total_endpoints=int(te),
        )
    return rows


def classify_owner(startpoint: str, endpoint: str) -> str:
    joined = f"{startpoint} {endpoint}"
    if "u_DDR3/u_ddrphy_top" in joined or "u_DDR3/u_ips_ddrc_top" in joined:
        return "IP_DDR_PHY"
    if "u_ips2l_pcie_wrap/U_IPS2L_PCIE_TOP" in joined:
        return "IP_PCIE_HARD"
    return "RTL_OWNED"


def parse_top_max_paths(rtr_text: str) -> List[dict]:
    lines = rtr_text.splitlines()
    out = []
    n = len(lines)
    i = 0
    while i < n:
        line = lines[i]
        if not line.startswith("Startpoint  :"):
            i += 1
            continue

        startpoint = line.split(":", 1)[1].strip()
        endpoint = ""
        path_group = ""
        path_type = ""
        path_class = ""
        slack: Optional[float] = None

        j = i + 1
        while j < n and j < i + 220:
            l = lines[j]
            if l.startswith("Endpoint    :"):
                endpoint = l.split(":", 1)[1].strip()
            elif l.startswith("Path Group  :"):
                path_group = l.split(":", 1)[1].strip()
            elif l.startswith("Path Type   :"):
                path_type = l.split(":", 1)[1].strip()
            elif l.startswith("Path Class  :"):
                path_class = l.split(":", 1)[1].strip()
            elif "Slack (" in l:
                ms = re.search(r"Slack \((?:VIOLATED|MET)\)\s+(-?\d+\.\d+)", l)
                if ms:
                    slack = float(ms.group(1))
                break
            j += 1

        if path_type == "max (slow corner)" and slack is not None:
            owner = classify_owner(startpoint, endpoint)
            out.append(
                {
                    "slack": slack,
                    "owner": owner,
                    "path_group": path_group,
                    "path_class": path_class,
                    "start": startpoint,
                    "end": endpoint,
                }
            )
        i = max(i + 1, j)
    out.sort(key=lambda x: x["slack"])
    return out


def parse_fdc_gate_errors(run_log_text: str) -> Dict[str, int]:
    counts = {}
    for patt in FDC_GATE_PATTERNS:
        counts[patt] = run_log_text.count(patt)
    return counts


def extract_key_metrics(rtr_path: Path, run_log_path: Path) -> KeyMetrics:
    rtr_text = read_text(rtr_path)
    run_text = read_text(run_log_path) if run_log_path.exists() else ""
    table = parse_setup_summary(rtr_text)
    fdc_counts = parse_fdc_gate_errors(run_text)

    def row(key: Tuple[str, str]) -> Optional[TimingRow]:
        return table.get(key)

    pclk = row(("pclk", "pclk"))
    ddrphy = row(("ddrphy_sysclk", "ddrphy_sysclk"))
    pdiv2 = row(("pclk_div2", "pclk_div2"))
    ref_to_div2 = row(("ref_clk", "pclk_div2"))
    div2_to_ref = row(("pclk_div2", "ref_clk"))

    return KeyMetrics(
        pclk_pclk_wns=None if pclk is None else pclk.wns,
        ddrphy_ddrphy_wns=None if ddrphy is None else ddrphy.wns,
        pclk_div2_wns=None if pdiv2 is None else pdiv2.wns,
        pclk_div2_tns=None if pdiv2 is None else pdiv2.tns,
        pclk_div2_fe=None if pdiv2 is None else pdiv2.failing_endpoints,
        ref_to_div2_fe=None if ref_to_div2 is None else ref_to_div2.failing_endpoints,
        div2_to_ref_fe=None if div2_to_ref is None else div2_to_ref.failing_endpoints,
        fdc_gate_total=sum(fdc_counts.values()),
    )


def write_top50_reports(rtr_path: Path, out_dir: Path) -> Tuple[int, int]:
    rtr_text = read_text(rtr_path)
    paths = parse_top_max_paths(rtr_text)
    top50 = paths[:50]

    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / "top50_path_classification.csv"
    txt_path = out_dir / "top50_path_classification.txt"

    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(
            f,
            fieldnames=["slack", "owner", "path_group", "path_class", "start", "end"],
            quoting=csv.QUOTE_ALL,
        )
        w.writeheader()
        w.writerows(top50)

    owner_counts: Dict[str, int] = {"RTL_OWNED": 0, "IP_DDR_PHY": 0, "IP_PCIE_HARD": 0}
    for p in top50:
        owner_counts[p["owner"]] = owner_counts.get(p["owner"], 0) + 1
    ip_count = owner_counts.get("IP_DDR_PHY", 0) + owner_counts.get("IP_PCIE_HARD", 0)

    lines = [
        "Top50 Slow-Max Path Classification",
        "",
        f"Total top50 paths: {len(top50)}",
        f"RTL_OWNED: {owner_counts.get('RTL_OWNED', 0)}",
        f"IP_DDR_PHY: {owner_counts.get('IP_DDR_PHY', 0)}",
        f"IP_PCIE_HARD: {owner_counts.get('IP_PCIE_HARD', 0)}",
        f"IP_OWNED_RATIO: {ip_count}/{len(top50)} ({(100.0 * ip_count / max(1, len(top50))):.2f}%)",
        "",
    ]
    for idx, p in enumerate(top50, start=1):
        lines.append(
            f"{idx:02d}. slack={p['slack']:.3f} owner={p['owner']} "
            f"group={p['path_group']} class={p['path_class']}"
        )
        lines.append(f"    start={p['start']}")
        lines.append(f"    end={p['end']}")
    write_text(txt_path, "\n".join(lines) + "\n")
    return ip_count, len(top50)


def snapshot_baseline(
    root: Path,
    source_rtr: Path,
    source_run_log: Path,
    tag: str,
) -> Path:
    out_dir = root / "timing_baseline" / tag
    out_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source_rtr, out_dir / RPT_NAME)
    if source_run_log.exists():
        shutil.copy2(source_run_log, out_dir / "run.log")

    key = extract_key_metrics(source_rtr, source_run_log)
    write_text(out_dir / "timing_summary.txt", read_text(source_rtr))
    ip_count, top_total = write_top50_reports(source_rtr, out_dir)
    run_text = read_text(source_run_log) if source_run_log.exists() else ""
    fdc_counts = parse_fdc_gate_errors(run_text)

    warning_counts_txt = "\n".join([f"{k}: {v}" for k, v in fdc_counts.items()]) + "\n"
    write_text(out_dir / "warning_counts.txt", warning_counts_txt)

    prev_capture = root / "timing_baseline" / "v5_20260318_1249" / "capture_300_frames.csv"
    if prev_capture.exists():
        shutil.copy2(prev_capture, out_dir / "capture_300_frames.csv")
    else:
        write_text(
            out_dir / "capture_300_frames.csv",
            "frame_idx,bytes,crc32_hex,nonzero_bytes,nonzero_ratio\n",
        )

    manifest = "\n".join(
        [
            "# V5.1 Baseline Manifest",
            "",
            f"- Snapshot time: {dt.datetime.now().isoformat(timespec='seconds')}",
            f"- Source timing report: `{source_rtr.as_posix()}`",
            f"- Source run log: `{source_run_log.as_posix()}`",
            "",
            "## Key Metrics",
            f"- pclk->pclk WNS: `{key.pclk_pclk_wns}`",
            f"- ddrphy_sysclk->ddrphy_sysclk WNS: `{key.ddrphy_ddrphy_wns}`",
            f"- pclk_div2->pclk_div2 WNS/TNS/FE: `{key.pclk_div2_wns}` / `{key.pclk_div2_tns}` / `{key.pclk_div2_fe}`",
            f"- ref_clk->pclk_div2 FE: `{key.ref_to_div2_fe}`",
            f"- pclk_div2->ref_clk FE: `{key.div2_to_ref_fe}`",
            "",
            "## Gates",
            f"- FDC parser/import gate total: `{key.fdc_gate_total}`",
            "- Patterns: Timing-4003, CommandTiming-0057, SDC-2023, ConstraintEditor-0046, SDC-2017",
            "",
            "## Top50 Ownership",
            f"- IP_OWNED ratio: `{ip_count}/{top_total}`",
            "",
            "## Board Regression Placeholder",
            "- `capture_300_frames.csv` retained as baseline comparison placeholder.",
            "",
        ]
    )
    write_text(out_dir / "BASELINE_MANIFEST.md", manifest)
    return out_dir


def run_seed(root: Path, pds_path: Path, pds_inputs: PdsInputs, seed: int, out_dir: Path) -> Tuple[bool, str]:
    out_dir.mkdir(parents=True, exist_ok=True)
    cmd_script = build_flow_commands(root, out_dir, seed, pds_inputs)
    cmd_path = out_dir / "run_seed.tcl"
    write_text(cmd_path, cmd_script)

    proc = subprocess.run(
        [str(PDS_SHELL)],
        input=cmd_script,
        cwd=str(root),
        capture_output=True,
        text=True,
    )
    shell_log = out_dir / "pds_shell.log"
    shell_text = proc.stdout + ("\n" + proc.stderr if proc.stderr else "")
    write_text(shell_log, shell_text)

    rtr = out_dir / "report_timing" / RPT_NAME
    run_log = out_dir / "report_timing" / "run.log"
    if not run_log.exists():
        run_log = out_dir / "run.log"
    if proc.returncode != 0:
        return False, f"returncode={proc.returncode}"
    if "E: Flow-0189" in shell_text:
        return False, "Flow-0189"
    if not rtr.exists():
        return False, "missing_rtr"
    if not run_log.exists():
        return False, "missing_run_log"
    return True, "ok"


def safe_float(v: Optional[float]) -> float:
    return -1e9 if v is None else v


def safe_inv_int(v: Optional[int]) -> int:
    return 10**9 if v is None else v


def main() -> int:
    parser = argparse.ArgumentParser(description="Run v5.1 timing multiseed closure flow")
    parser.add_argument("--root", default=".", help="Project root")
    parser.add_argument("--pds", default="pcie_dma_test.pds", help="PDS file used for input inventory")
    parser.add_argument("--seeds", default="101,102,103,104,105", help="Comma-separated seed list")
    parser.add_argument("--baseline-tag", default="v5_1_20260318_1319", help="Baseline snapshot tag")
    parser.add_argument(
        "--source-rtr",
        default=f"report_timing/{RPT_NAME}",
        help="Source timing report path for baseline snapshot",
    )
    parser.add_argument(
        "--source-run-log",
        default="report_timing/run.log",
        help="Source run.log path for baseline snapshot",
    )
    parser.add_argument("--run-tag", default="", help="Closure run directory tag")
    parser.add_argument(
        "--reuse-run-root",
        default="",
        help="Reuse an existing closure_runs/<tag> directory and only re-parse results",
    )
    args = parser.parse_args()

    root = Path(args.root).resolve()
    pds_path = (root / args.pds).resolve()
    if not PDS_SHELL.exists():
        raise RuntimeError(f"pds_shell not found: {PDS_SHELL}")
    if not pds_path.exists():
        raise RuntimeError(f"PDS not found: {pds_path}")

    seeds = [int(x.strip()) for x in args.seeds.split(",") if x.strip()]
    if not seeds:
        raise RuntimeError("No seeds provided")

    src_rtr = (root / args.source_rtr).resolve()
    src_run = (root / args.source_run_log).resolve()
    if not src_rtr.exists():
        raise RuntimeError(f"Missing source timing report: {src_rtr}")

    baseline_dir = snapshot_baseline(root, src_rtr, src_run, args.baseline_tag)
    print(f"[v5.1] baseline snapshot: {baseline_dir}")

    pds_inputs = parse_pds_inputs(pds_path)
    if args.reuse_run_root:
        run_root = (root / args.reuse_run_root).resolve()
        if not run_root.exists():
            raise RuntimeError(f"reuse run root does not exist: {run_root}")
    else:
        stamp = dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        run_tag = args.run_tag or f"v5_1_multiseed_{stamp}"
        run_root = root / "closure_runs" / run_tag
        run_root.mkdir(parents=True, exist_ok=True)

    summary_rows = []
    for seed in seeds:
        seed_dir = run_root / f"seed_{seed}"
        if args.reuse_run_root:
            print(f"[v5.1] reuse seed {seed} -> {seed_dir}")
            rtr = seed_dir / "report_timing" / RPT_NAME
            run_log = seed_dir / "report_timing" / "run.log"
            if not run_log.exists():
                run_log = seed_dir / "run.log"
            if rtr.exists() and run_log.exists():
                ok, reason = True, "ok"
            else:
                ok, reason = False, "missing_artifacts"
        else:
            print(f"[v5.1] run seed {seed} -> {seed_dir}")
            ok, reason = run_seed(root, pds_path, pds_inputs, seed, seed_dir)
        row = {
            "seed": seed,
            "status": "OK" if ok else "FAIL",
            "reason": reason,
            "pclk_pclk_wns": "",
            "ddrphy_ddrphy_wns": "",
            "pclk_div2_wns": "",
            "pclk_div2_tns": "",
            "pclk_div2_fe": "",
            "ref_to_div2_fe": "",
            "div2_to_ref_fe": "",
            "fdc_gate_total": "",
            "ip_owned_top50_ratio_pct": "",
        }
        if ok:
            rtr = seed_dir / "report_timing" / RPT_NAME
            run_log = seed_dir / "report_timing" / "run.log"
            if not run_log.exists():
                run_log = seed_dir / "run.log"
            key = extract_key_metrics(rtr, run_log)
            ip_count, total = write_top50_reports(rtr, seed_dir)
            ip_ratio = 100.0 * ip_count / max(1, total)
            row.update(
                {
                    "pclk_pclk_wns": key.pclk_pclk_wns,
                    "ddrphy_ddrphy_wns": key.ddrphy_ddrphy_wns,
                    "pclk_div2_wns": key.pclk_div2_wns,
                    "pclk_div2_tns": key.pclk_div2_tns,
                    "pclk_div2_fe": key.pclk_div2_fe,
                    "ref_to_div2_fe": key.ref_to_div2_fe,
                    "div2_to_ref_fe": key.div2_to_ref_fe,
                    "fdc_gate_total": key.fdc_gate_total,
                    "ip_owned_top50_ratio_pct": f"{ip_ratio:.2f}",
                }
            )
        summary_rows.append(row)

    csv_path = run_root / "multiseed_summary.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=list(summary_rows[0].keys()))
        w.writeheader()
        w.writerows(summary_rows)
    print(f"[v5.1] summary: {csv_path}")

    ok_rows = [r for r in summary_rows if r["status"] == "OK"]
    best_seed = None
    if ok_rows:
        ranked = sorted(
            ok_rows,
            key=lambda r: (
                1 if float(r["pclk_pclk_wns"]) >= 0 else 0,
                1 if float(r["ddrphy_ddrphy_wns"]) >= 0 else 0,
                safe_float(float(r["pclk_div2_wns"])),
                safe_float(float(r["pclk_div2_tns"])),
                -safe_inv_int(int(r["pclk_div2_fe"])),
            ),
            reverse=True,
        )
        best_seed = int(ranked[0]["seed"])

    if best_seed is not None:
        best_dir = run_root / f"seed_{best_seed}"
        write_text(run_root / "BEST_SEED.txt", f"{best_seed}\n")
        best_note = [
            "# v5.1 Best Seed",
            "",
            f"- best_seed: {best_seed}",
            f"- run_dir: `{best_dir.as_posix()}`",
            "",
            "Selection priority:",
            "1) clear pclk->pclk",
            "2) clear ddrphy_sysclk->ddrphy_sysclk",
            "3) improve pclk_div2 WNS",
            "4) improve pclk_div2 TNS/FE",
            "",
        ]
        write_text(run_root / "BEST_SEED.md", "\n".join(best_note))
        print(f"[v5.1] best seed: {best_seed}")
    else:
        print("[v5.1] no successful seed run")
        return 2

    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except Exception as exc:
        print(f"[v5.1] ERROR: {exc}", file=sys.stderr)
        sys.exit(1)
