#!/usr/bin/env python3
"""
Phase S blocker audit for OV5640 register init table.

Checks:
1) Register-index continuity vs state-machine traversal.
2) Duplicate writes and key-register final values.
3) Risk markers: default 0xFFFFFF write path and unused initial_en gate.
"""

from __future__ import annotations

import argparse
import re
from collections import Counter, defaultdict
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple


ENTRY_RE = re.compile(r"(\d+)\s*:\s*reg_data\s*<=\s*24'h([0-9a-fA-F]{6})")
LIMIT_RE = re.compile(r"if\s*\(\s*reg_index\s*<\s*(\d+)\s*\)")


@dataclass
class Entry:
    idx: int
    addr: int
    data: int


KEY_REGS = {
    0x4300: "Output format",
    0x501F: "ISP format mux",
    0x3503: "AEC/AGC auto mode",
    0x3814: "X subsample/inc",
    0x3815: "Y subsample/inc",
    0x3820: "Mirror/flip/timing",
    0x3821: "Mirror/flip/timing",
    0x5180: "AWB control",
}


def parse_entries(text: str) -> List[Entry]:
    entries: List[Entry] = []
    for m in ENTRY_RE.finditer(text):
        idx = int(m.group(1))
        raw = int(m.group(2), 16)
        addr = (raw >> 8) & 0xFFFF
        data = raw & 0xFF
        entries.append(Entry(idx=idx, addr=addr, data=data))
    return entries


def detect_reg_limit(text: str) -> int:
    m = LIMIT_RE.search(text)
    if not m:
        return -1
    return int(m.group(1))


def summarize(entries: List[Entry], reg_limit: int, text: str) -> Tuple[bool, List[str]]:
    findings: List[str] = []
    trusted = True

    if not entries:
        return False, ["No reg_data entries parsed from reg_config.v."]

    idx_min = min(e.idx for e in entries)
    idx_max = max(e.idx for e in entries)
    present = {e.idx for e in entries}

    if reg_limit > 0:
        expected = list(range(0, reg_limit))
    else:
        expected = list(range(idx_min, idx_max + 1))

    missing = [i for i in expected if i not in present]
    if missing:
        trusted = False
        findings.append(
            f"Missing table indices: {len(missing)} (first 20: {missing[:20]}). "
            "State machine still traverses full range."
        )

    default_ffff = "default:reg_data<=24'hffffff" in text.replace(" ", "").lower()
    if default_ffff and missing:
        trusted = False
        findings.append("Missing indices will emit default 0xFFFFFF pseudo-write (invalid register payload).")

    addr_counter = Counter(e.addr for e in entries)
    dup_addrs = [(a, c) for a, c in addr_counter.items() if c > 1]
    if dup_addrs:
        findings.append(
            "Duplicate register writes exist (sample): "
            + ", ".join(f"0x{a:04x} x{c}" for a, c in sorted(dup_addrs)[:8])
        )

    final_map: Dict[int, Entry] = {}
    for e in sorted(entries, key=lambda x: x.idx):
        final_map[e.addr] = e

    for addr, name in KEY_REGS.items():
        if addr not in final_map:
            trusted = False
            findings.append(f"Missing key register 0x{addr:04x} ({name}).")

    # initial_en is declared but expected to be used as sequencing gate.
    token_hits = len(re.findall(r"\binitial_en\b", text))
    if token_hits <= 1:
        trusted = False
        findings.append("initial_en appears unused in sequencing logic (power-on gate bypass risk).")

    # ack is wired from i2c_com but not consumed by state machine.
    ack_hits = len(re.findall(r"\back\b", text))
    if ack_hits <= 1:
        findings.append("I2C ACK is not consumed for retry/abort decisions.")

    findings.insert(
        0,
        f"Parsed entries={len(entries)}, idx_range=[{idx_min}, {idx_max}], reg_index_limit={reg_limit}.",
    )
    return trusted, findings


def print_key_registers(entries: List[Entry]) -> None:
    final_map: Dict[int, Entry] = defaultdict(lambda: Entry(idx=-1, addr=0, data=0))
    for e in sorted(entries, key=lambda x: x.idx):
        final_map[e.addr] = e
    print("Key register final writes:")
    for addr, name in KEY_REGS.items():
        if addr in final_map and final_map[addr].idx >= 0:
            e = final_map[addr]
            print(f"  0x{addr:04x} ({name}): idx={e.idx} data=0x{e.data:02x}")
        else:
            print(f"  0x{addr:04x} ({name}): MISSING")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", default="../hdl/reg_config.v", help="Path to reg_config.v")
    parser.add_argument(
        "--strict",
        action="store_true",
        help="Return non-zero when table is not trusted for image-quality root-cause analysis",
    )
    args = parser.parse_args()

    path = Path(args.file)
    if not path.exists():
        print(f"ERR: file not found: {path}")
        return 2

    text = path.read_text(encoding="utf-8", errors="ignore")
    entries = parse_entries(text)
    reg_limit = detect_reg_limit(text)
    trusted, findings = summarize(entries, reg_limit, text)

    print("=== Phase S Register Table Audit ===")
    print(f"file={path}")
    print(f"trusted_for_iq_analysis={'YES' if trusted else 'NO'}")
    for line in findings:
        print(f"- {line}")
    print_key_registers(entries)

    if args.strict and not trusted:
        return 3
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

