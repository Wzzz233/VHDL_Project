#!/usr/bin/env python3
"""Minimal local web control panel for pplcnet_bgp_live.

The server intentionally does not touch /dev/fpga_dma0. It only starts/stops
the live binary and serves preview/status files written by pplcnet_bgp_live.
"""

from __future__ import annotations

import argparse
import json
import os
import signal
import subprocess
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse


ROOT = Path(__file__).resolve().parent
STATIC = ROOT / "static"

DEFAULT_CMD = [
    "./pplcnet_bgp_live",
    "--plate-model", "/userdata/model/best_int8_scorex256.rknn",
    "--ocr-blue-model", "/userdata/model/pplcnet_blue_v2_rk3568_fp16.rknn",
    "--ocr-green-model", "/userdata/model/pplcnet_green_v2_b1plus_rk3568_fp16.rknn",
    "--ocr-yellow-model", "/userdata/model/pplcnet_yellow_crpd_v1b_rk3568_fp16.rknn",
    "--ocr-police-model", "/userdata/model/pplcnet_police_v4_warmblue_rk3568_fp16.rknn",
    "--ocr-embassy-model", "/userdata/model/pplcnet_embassy_v1_rk3568_fp16.rknn",
    "--plate-type-classifier-model", "/userdata/model/plate_type_classifier_6cls_resnet18_warped_nocrop_rk3568_fp16_opt0.rknn",
    "--ocr-blue-keys", "/userdata/model/special_keys.txt",
    "--ocr-green-keys", "/userdata/model/pplcnet_green_keys.txt",
    "--ocr-yellow-keys", "/userdata/model/yellow_normal_keys.txt",
    "--ocr-police-keys", "/userdata/model/police_keys.txt",
    "--ocr-embassy-keys", "/userdata/model/embassy_keys.txt",
    "--ocr-keys", "/userdata/model/special_keys.txt",
    "--det-resize", "letterbox",
    "--ocr-preproc", "gray",
    "--fps", "30",
    "--display-sync", "1",
    "--display-every", "2",
    "--det-score-scale", "256",
    "--min-plate-conf", "0.15",
    "--plate-nms-iou", "0.65",
    "--plate-max-det", "4",
]


class AppState:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.lock = threading.Lock()
        self.proc: subprocess.Popen[bytes] | None = None
        self.log_path = args.log_path
        self.display_every = args.display_every
        self.fps = args.fps
        self.no_infer = False
        self.preview_fps = args.preview_fps
        self.dma_pre_delay_us = 0

    def build_cmd(self) -> list[str]:
        cmd = list(DEFAULT_CMD)
        cmd[cmd.index("--fps") + 1] = str(self.fps)
        cmd[cmd.index("--display-every") + 1] = str(self.display_every)
        cmd.extend(["--web-preview-dir", str(self.args.preview_dir)])
        cmd.extend(["--web-preview-fps", str(self.preview_fps)])
        if self.dma_pre_delay_us > 0:
            cmd.extend(["--dma-pre-delay-us", str(self.dma_pre_delay_us)])
        if self.no_infer:
            cmd.append("--no-infer")
        if self.args.sudo:
            cmd = ["sudo", "-n"] + cmd
        return cmd

    def is_running(self) -> bool:
        return self.proc is not None and self.proc.poll() is None

    def start(self) -> tuple[bool, str]:
        with self.lock:
            if self.is_running():
                return True, "already running"
            self.args.preview_dir.mkdir(parents=True, exist_ok=True)
            self.log_path.parent.mkdir(parents=True, exist_ok=True)
            cmd = self.build_cmd()
            log = self.log_path.open("ab", buffering=0)
            try:
                self.proc = subprocess.Popen(
                    cmd,
                    cwd=str(self.args.arm_dir),
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    preexec_fn=os.setsid,
                )
            except Exception as exc:  # pragma: no cover - board dependent
                log.close()
                self.proc = None
                return False, str(exc)
            return True, "started"

    def stop(self) -> tuple[bool, str]:
        with self.lock:
            proc = self.proc
            if proc is None or proc.poll() is not None:
                self.proc = None
                return True, "not running"
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGINT)
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                proc.wait(timeout=3)
            finally:
                self.proc = None
            return True, "stopped"

    def config(self, values: dict[str, str]) -> None:
        with self.lock:
            if "display_every" in values:
                self.display_every = max(1, min(120, int(values["display_every"])))
            if "fps" in values:
                self.fps = max(1, min(120, int(values["fps"])))
            if "preview_fps" in values:
                self.preview_fps = max(0, min(30, int(values["preview_fps"])))
            if "dma_pre_delay_us" in values:
                self.dma_pre_delay_us = max(0, min(1000000, int(values["dma_pre_delay_us"])))
            if "no_infer" in values:
                self.no_infer = values["no_infer"] in ("1", "true", "on")

    def snapshot(self) -> dict[str, object]:
        proc = self.proc
        return {
            "running": self.is_running(),
            "pid": proc.pid if proc and proc.poll() is None else None,
            "display_every": self.display_every,
            "fps": self.fps,
            "preview_fps": self.preview_fps,
            "dma_pre_delay_us": self.dma_pre_delay_us,
            "no_infer": self.no_infer,
            "preview_dir": str(self.args.preview_dir),
            "log_path": str(self.log_path),
        }


def read_tail(path: Path, limit: int = 20000) -> str:
    if not path.exists():
        return ""
    with path.open("rb") as f:
        f.seek(0, os.SEEK_END)
        size = f.tell()
        f.seek(max(0, size - limit), os.SEEK_SET)
        return f.read().decode("utf-8", errors="replace")


class Handler(BaseHTTPRequestHandler):
    state: AppState

    def log_message(self, fmt: str, *args: object) -> None:
        return

    def send_json(self, obj: object, status: int = 200) -> None:
        data = json.dumps(obj, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def send_file(self, path: Path, content_type: str) -> None:
        if not path.exists():
            self.send_error(HTTPStatus.NOT_FOUND)
            return
        data = path.read_bytes()
        self.send_response(200)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self) -> None:
        parsed = urlparse(self.path)
        if parsed.path == "/":
            self.send_file(STATIC / "index.html", "text/html; charset=utf-8")
        elif parsed.path == "/app.js":
            self.send_file(STATIC / "app.js", "application/javascript; charset=utf-8")
        elif parsed.path == "/style.css":
            self.send_file(STATIC / "style.css", "text/css; charset=utf-8")
        elif parsed.path == "/api/status":
            status_path = self.state.args.preview_dir / "status.json"
            live_status = {}
            if status_path.exists():
                try:
                    live_status = json.loads(status_path.read_text(encoding="utf-8"))
                except Exception:
                    live_status = {}
            self.send_json({"server": self.state.snapshot(), "live": live_status})
        elif parsed.path == "/api/results":
            self.send_file(self.state.args.preview_dir / "results.json", "application/json; charset=utf-8")
        elif parsed.path == "/api/frame.bmp":
            self.send_file(self.state.args.preview_dir / "latest.bmp", "image/bmp")
        elif parsed.path == "/api/log":
            self.send_json({"log": read_tail(self.state.log_path)})
        else:
            self.send_error(HTTPStatus.NOT_FOUND)

    def do_POST(self) -> None:
        parsed = urlparse(self.path)
        length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(length).decode("utf-8", errors="replace")
        values = {k: v[-1] for k, v in parse_qs(body).items()}
        if parsed.path == "/api/start":
            ok, msg = self.state.start()
            self.send_json({"ok": ok, "message": msg, "status": self.state.snapshot()})
        elif parsed.path == "/api/stop":
            ok, msg = self.state.stop()
            self.send_json({"ok": ok, "message": msg, "status": self.state.snapshot()})
        elif parsed.path == "/api/config":
            try:
                restart = values.pop("restart", "0") in ("1", "true", "on")
                was_running = self.state.is_running()
                if restart and was_running:
                    self.state.stop()
                self.state.config(values)
                if restart and was_running:
                    self.state.start()
                self.send_json({"ok": True, "status": self.state.snapshot()})
            except Exception as exc:
                self.send_json({"ok": False, "message": str(exc)}, status=400)
        else:
            self.send_error(HTTPStatus.NOT_FOUND)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="0.0.0.0")
    parser.add_argument("--port", type=int, default=8080)
    parser.add_argument("--arm-dir", type=Path, default=Path.cwd())
    parser.add_argument("--preview-dir", type=Path, default=Path("/tmp/bgp_web"))
    parser.add_argument("--log-path", type=Path, default=Path("/tmp/bgp_web/live.log"))
    parser.add_argument("--display-every", type=int, default=2)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--preview-fps", type=int, default=5)
    parser.add_argument("--sudo", action="store_true", help="start pplcnet_bgp_live via sudo -n")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    Handler.state = AppState(args)
    httpd = ThreadingHTTPServer((args.host, args.port), Handler)
    print(f"web control listening on http://{args.host}:{args.port}")
    try:
        httpd.serve_forever()
    finally:
        Handler.state.stop()


if __name__ == "__main__":
    main()
