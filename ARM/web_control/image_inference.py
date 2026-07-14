#!/usr/bin/env python3
"""Single-image inference adapter for the board web control."""

from __future__ import annotations

import dataclasses
import base64
import json
import socket
import subprocess
import tempfile
import threading
import time
from pathlib import Path
from typing import Any


class ImageInferenceError(RuntimeError):
    pass


class ImageInferenceBusy(ImageInferenceError):
    pass


def completed_plate_result(result: dict[str, Any]) -> bool:
    sequence = result.get("sequence")
    return isinstance(sequence, int) and not isinstance(sequence, bool) and sequence > 0


@dataclasses.dataclass(frozen=True)
class ImageInferenceConfig:
    arm_root: Path = Path("/home/linaro/ARM")
    model_root: Path = Path("/userdata/model")
    plate_driver: Path = Path("/home/linaro/ARM/pplcnet_bgp_live")
    plate_type_model: Path | None = None
    pedestrian_driver: Path = Path("/home/linaro/ARM/cplus-rk3568-driver")
    gst_launch: str = "gst-launch-1.0"
    timeout: float = 45.0


def _socket_json(path: Path, payload: dict[str, Any], timeout: float) -> dict[str, Any]:
    request = json.dumps(payload, separators=(",", ":")).encode("utf-8") + b"\n"
    with socket.socket(socket.AF_UNIX, socket.SOCK_STREAM) as client:
        client.settimeout(timeout)
        client.connect(str(path))
        client.sendall(request)
        stream = client.makefile("rb")
        line = stream.readline(1024 * 1024)
    if not line:
        raise ImageInferenceError("单图识别进程没有返回结果")
    try:
        value = json.loads(line.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ImageInferenceError("单图识别进程返回了无效结果") from exc
    if not isinstance(value, dict):
        raise ImageInferenceError("单图识别结果格式错误")
    return value


class ImageInferenceRunner:
    def __init__(self, config: ImageInferenceConfig) -> None:
        self.config = config
        self._lock = threading.Lock()

    def _decode(self, image_path: Path, raw_path: Path) -> None:
        command = [
            self.config.gst_launch,
            "-q",
            "filesrc",
            f"location={image_path}",
            "!",
            "decodebin",
            "!",
            "videoconvert",
            "!",
            "videoscale",
            "add-borders=true",
            "!",
            "video/x-raw,format=BGRx,width=1280,height=720,pixel-aspect-ratio=1/1",
            "!",
            "filesink",
            f"location={raw_path}",
        ]
        try:
            completed = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=min(self.config.timeout, 20.0),
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise ImageInferenceError("板端图片解码失败") from exc
        expected = 1280 * 720 * 4
        if completed.returncode != 0 or not raw_path.is_file() or raw_path.stat().st_size != expected:
            detail = completed.stderr.decode("utf-8", "replace").strip()[-500:]
            raise ImageInferenceError(f"无法把图片转换为 1280x720 BGRx: {detail}")

    def _plate_command(self, raw_path: Path, socket_path: Path) -> list[str]:
        model = self.config.model_root
        plate_type_model = self.config.plate_type_model or (
            model / "plate_type_classifier_5color_resnet18_warped_nocrop_rk3568_fp16_opt0.rknn"
        )
        return [
            str(self.config.plate_driver),
            "--plate-model", str(model / "best_fp16.rknn"),
            "--ocr-blue-model", str(model / "pplcnet_blue_v3_rk3568_fp16.rknn"),
            "--ocr-green-model", str(model / "pplcnet_green_v2_b1plus_rk3568_fp16.rknn"),
            "--ocr-yellow-model", str(model / "pplcnet_yellow_all_single_v1_rk3568_fp16.rknn"),
            "--ocr-police-model", str(model / "pplcnet_police_v5_whiteexpand_rk3568_fp16.rknn"),
            "--ocr-embassy-model", str(model / "pplcnet_black_unified_v1_rk3568_fp16.rknn"),
            "--plate-type-classifier-model", str(plate_type_model),
            "--ocr-blue-keys", str(model / "special_keys.txt"),
            "--ocr-green-keys", str(model / "pplcnet_green_keys.txt"),
            "--ocr-yellow-keys", str(model / "yellow_keys.txt"),
            "--ocr-police-keys", str(model / "police_keys.txt"),
            "--ocr-embassy-keys", str(model / "black_unified_keys.txt"),
            "--ocr-keys", str(model / "special_keys.txt"),
            "--det-resize", "letterbox",
            "--ocr-preproc", "gray",
            "--fps", "60",
            "--det-score-scale", "1",
            "--min-plate-conf", "0.25",
            "--plate-nms-iou", "0.45",
            "--plate-max-det", "16",
            "--source", "fpga",
            "--input-bgrx", str(raw_path),
            "--frames", "600",
            "--no-display",
            "--control-socket", str(socket_path),
        ]

    def _run_plate(self, raw_path: Path, work: Path) -> dict[str, Any]:
        socket_path = work / "plate.sock"
        log_path = work / "plate.log"
        deadline = time.monotonic() + self.config.timeout
        with log_path.open("wb") as log:
            try:
                process = subprocess.Popen(
                    self._plate_command(raw_path, socket_path),
                    cwd=self.config.arm_root,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                )
            except OSError as exc:
                raise ImageInferenceError("无法启动单图车牌识别程序") from exc
            try:
                while time.monotonic() < deadline:
                    if socket_path.exists():
                        try:
                            result = _socket_json(socket_path, {"op": "results"}, 1.0)
                        except (OSError, ImageInferenceError):
                            result = {}
                        if completed_plate_result(result):
                            return result
                    if process.poll() is not None:
                        break
                    time.sleep(0.05)
            finally:
                if process.poll() is None:
                    process.terminate()
                    try:
                        process.wait(timeout=2.0)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=2.0)
        detail = log_path.read_text("utf-8", errors="replace")[-1000:]
        raise ImageInferenceError(f"车牌图片识别没有完成: {detail}")

    def _encode_bgrx_jpeg(self, raw_path: Path, jpeg_path: Path) -> bytes:
        command = [
            self.config.gst_launch,
            "-q",
            "filesrc",
            f"location={raw_path}",
            "blocksize=3686400",
            "num-buffers=1",
            "!",
            "video/x-raw,format=BGRx,width=1280,height=720,framerate=1/1",
            "!",
            "videoconvert",
            "!",
            "jpegenc",
            "quality=80",
            "!",
            "filesink",
            f"location={jpeg_path}",
        ]
        try:
            completed = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=min(self.config.timeout, 20.0),
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise ImageInferenceError("mask 结果图编码失败") from exc
        data = jpeg_path.read_bytes() if jpeg_path.is_file() else b""
        if completed.returncode != 0 or not data.startswith(b"\xff\xd8") or len(data) > 4 * 1024 * 1024:
            detail = completed.stderr.decode("utf-8", "replace").strip()[-500:]
            raise ImageInferenceError(f"mask 结果图编码失败: {detail}")
        return data

    def _run_pedestrian(self, raw_path: Path, work: Path) -> tuple[dict[str, Any], bytes]:
        model = self.config.model_root
        mask_bgrx_path = work / "pedestrian-mask.bgrx"
        mask_jpeg_path = work / "pedestrian-mask.jpg"
        command = [
            str(self.config.pedestrian_driver),
            "--det-model", str(model / "yolov5nu_coco_rk3568_fp16_20260710.rknn"),
            "--seg-model", str(model / "mapillary_cplus_ground_4class_v2_rk3568_fp16_20260710.rknn"),
            "--input-bgrx", str(raw_path),
            "--width", "1280",
            "--height", "720",
            "--display", "0",
            "--always-segment",
            "--output-mask-bgrx", str(mask_bgrx_path),
        ]
        try:
            completed = subprocess.run(
                command,
                cwd=self.config.arm_root,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                timeout=self.config.timeout,
                check=False,
                text=True,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise ImageInferenceError("行人检测程序启动或运行失败") from exc
        for line in reversed(completed.stdout.splitlines()):
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(value, dict):
                expected = 1280 * 720 * 4
                if not mask_bgrx_path.is_file() or mask_bgrx_path.stat().st_size != expected:
                    raise ImageInferenceError("CPlus 没有生成 mask 结果图，请更新板端 CPlus 驱动")
                return value, self._encode_bgrx_jpeg(mask_bgrx_path, mask_jpeg_path)
        detail = completed.stdout[-1000:]
        raise ImageInferenceError(f"行人检测没有返回 JSON 结果: {detail}")

    def run(self, mode: str, image: bytes, media_type: str) -> dict[str, Any]:
        if mode not in ("plate", "pedestrian"):
            raise ImageInferenceError("识别模式必须是 plate 或 pedestrian")
        if media_type not in ("image/jpeg", "image/png"):
            raise ImageInferenceError("只支持 JPEG 或 PNG 图片")
        if not self._lock.acquire(blocking=False):
            raise ImageInferenceBusy("已有图片正在识别，请稍后再试")
        suffix = ".jpg" if media_type == "image/jpeg" else ".png"
        try:
            with tempfile.TemporaryDirectory(prefix="pplcnet-upload-") as directory:
                work = Path(directory)
                image_path = work / ("input" + suffix)
                raw_path = work / "input.bgrx"
                image_path.write_bytes(image)
                self._decode(image_path, raw_path)
                rendered_jpeg: bytes | None = None
                if mode == "plate":
                    result = self._run_plate(raw_path, work)
                else:
                    result, rendered_jpeg = self._run_pedestrian(raw_path, work)
                response = {
                    "ok": True,
                    "mode": mode,
                    "frame": {"width": 1280, "height": 720},
                    "results": result,
                }
                if rendered_jpeg is not None:
                    response["rendered_image"] = {
                        "content_type": "image/jpeg",
                        "base64": base64.b64encode(rendered_jpeg).decode("ascii"),
                    }
                return response
        finally:
            self._lock.release()
