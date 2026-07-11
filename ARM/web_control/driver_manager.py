#!/usr/bin/env python3
"""Exclusive plate-live / pedestrian-image mode management."""

from __future__ import annotations

import contextlib
import os
import signal
import subprocess
import threading
import time
from pathlib import Path
from typing import Iterator


class DriverManagerError(RuntimeError):
    pass


class DriverManager:
    def __init__(
        self,
        plate_script: Path,
        runtime_dir: Path = Path("/run/pplcnet-board"),
        log_dir: Path = Path("/var/log/pplcnet-board"),
        model_dir: Path = Path("/userdata/model"),
        startup_grace: float = 1.0,
        stop_timeout: float = 4.0,
    ) -> None:
        self.plate_script = plate_script
        self.runtime_dir = runtime_dir
        self.log_dir = log_dir
        self.model_dir = model_dir
        self.startup_grace = startup_grace
        self.stop_timeout = stop_timeout
        self.pid_path = runtime_dir / "plate.pid"
        self.mode_path = runtime_dir / "mode"
        self.log_path = log_dir / "plate.log"
        self._lock = threading.RLock()
        self._plate_process: subprocess.Popen[bytes] | None = None
        runtime_dir.mkdir(parents=True, exist_ok=True)
        log_dir.mkdir(parents=True, exist_ok=True)
        if not self.mode_path.exists():
            self._write_mode("plate")

    def _write_mode(self, mode: str) -> None:
        temporary = self.mode_path.with_suffix(".tmp")
        temporary.write_text(mode + "\n", encoding="ascii")
        os.replace(temporary, self.mode_path)

    def _read_mode(self) -> str:
        try:
            mode = self.mode_path.read_text(encoding="ascii").strip()
        except OSError:
            mode = "plate"
        return mode if mode in ("plate", "pedestrian") else "plate"

    def _read_pid(self) -> int | None:
        try:
            text = self.pid_path.read_text(encoding="ascii").strip()
            pid = int(text)
        except (OSError, ValueError):
            return None
        return pid if pid > 1 else None

    @staticmethod
    def _alive(pid: int | None) -> bool:
        if pid is None:
            return False
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return False
        except PermissionError:
            return True
        return True

    def _stop_plate_locked(self) -> None:
        pid = self._read_pid()
        if not self._alive(pid):
            self.pid_path.unlink(missing_ok=True)
            return
        assert pid is not None
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            self.pid_path.unlink(missing_ok=True)
            return
        deadline = time.monotonic() + self.stop_timeout
        while time.monotonic() < deadline:
            if not self._alive(pid):
                if self._plate_process and self._plate_process.pid == pid:
                    self._plate_process.wait(timeout=1.0)
                    self._plate_process = None
                self.pid_path.unlink(missing_ok=True)
                return
            time.sleep(0.05)
        try:
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        if self._plate_process and self._plate_process.pid == pid:
            try:
                self._plate_process.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                pass
            self._plate_process = None
        self.pid_path.unlink(missing_ok=True)

    def _start_plate_locked(self) -> None:
        pid = self._read_pid()
        if self._alive(pid):
            return
        if not self.plate_script.is_file():
            raise DriverManagerError(f"车牌启动脚本不存在: {self.plate_script}")
        environment = os.environ.copy()
        environment["MODEL_DIR"] = str(self.model_dir)
        with self.log_path.open("ab", buffering=0) as log:
            try:
                process = subprocess.Popen(
                    [str(self.plate_script)],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                    env=environment,
                )
            except OSError as exc:
                raise DriverManagerError("无法启动实时车牌驱动") from exc
        self.pid_path.write_text(f"{process.pid}\n", encoding="ascii")
        self._plate_process = process
        time.sleep(self.startup_grace)
        return_code = process.poll()
        if return_code is not None:
            self.pid_path.unlink(missing_ok=True)
            self._plate_process = None
            detail = self.log_path.read_text("utf-8", errors="replace")[-1000:]
            raise DriverManagerError(f"实时车牌驱动启动失败({return_code}): {detail}")

    def status(self) -> dict[str, object]:
        with self._lock:
            mode = self._read_mode()
            running = self._alive(self._read_pid())
            return {
                "ok": True,
                "mode": mode,
                "plate_running": running,
                "pedestrian_on_demand": True,
            }

    def set_mode(self, mode: str) -> dict[str, object]:
        if mode not in ("plate", "pedestrian"):
            raise DriverManagerError("模式必须是 plate 或 pedestrian")
        with self._lock:
            if mode == "plate":
                self._start_plate_locked()
            else:
                self._stop_plate_locked()
            self._write_mode(mode)
            return self.status()

    @contextlib.contextmanager
    def image_session(self, mode: str) -> Iterator[None]:
        if mode not in ("plate", "pedestrian"):
            raise DriverManagerError("模式必须是 plate 或 pedestrian")
        with self._lock:
            current = self._read_mode()
            if current != mode:
                if mode == "plate":
                    self._start_plate_locked()
                else:
                    self._stop_plate_locked()
                self._write_mode(mode)
            restart_plate = mode == "plate" and self._alive(self._read_pid())
            if restart_plate:
                self._stop_plate_locked()
            try:
                yield
            finally:
                if restart_plate and self._read_mode() == "plate":
                    self._start_plate_locked()
