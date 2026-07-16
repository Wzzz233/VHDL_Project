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


def is_suspected_decision(decision: Any) -> bool:
    """A target decision is a suspected violation when it starts with 'suspected'.

    The cplus driver emits the full string 'suspected_crossing_road_outside_zebra'
    (not the bare 'suspected'), so an equality check would never match.
    """
    return isinstance(decision, str) and decision.startswith("suspected")


def plate_input_index(input_sequence: int, input_count: int, repeat: int) -> int:
    if input_sequence <= 0 or input_count <= 0 or repeat <= 0:
        raise ValueError("invalid static plate input sequence")
    return ((input_sequence - 1) // repeat) % input_count


@dataclasses.dataclass(frozen=True)
class ImageInferenceConfig:
    arm_root: Path = Path("/home/linaro/ARM")
    model_root: Path = Path("/userdata/model")
    plate_driver: Path = Path("/home/linaro/ARM/pplcnet_bgp_live")
    plate_type_model: Path | None = None
    pedestrian_driver: Path = Path("/home/linaro/ARM/cplus-rk3568-driver")
    gst_launch: str = "gst-launch-1.0"
    timeout: float = 45.0
    plate_max_passes: int = 9
    plate_overlap: float = 0.35
    plate_edge_pad_ratio: float = 0.12
    plate_input_repeat: int = 75


@dataclasses.dataclass(frozen=True)
class PlateImagePass:
    name: str
    x: int
    y: int
    width: int
    height: int
    pad_left: int = 0
    pad_top: int = 0
    pad_right: int = 0
    pad_bottom: int = 0

    @property
    def virtual_width(self) -> int:
        return self.pad_left + self.width + self.pad_right

    @property
    def virtual_height(self) -> int:
        return self.pad_top + self.height + self.pad_bottom


def image_dimensions(data: bytes) -> tuple[int, int]:
    """Read JPEG/PNG dimensions without adding an image-library dependency."""
    if data.startswith(b"\x89PNG\r\n\x1a\n") and len(data) >= 24:
        width = int.from_bytes(data[16:20], "big")
        height = int.from_bytes(data[20:24], "big")
        if width > 0 and height > 0:
            return width, height
        raise ValueError("invalid PNG dimensions")

    if data.startswith(b"\xff\xd8"):
        sof_markers = {
            0xC0, 0xC1, 0xC2, 0xC3, 0xC5, 0xC6, 0xC7,
            0xC9, 0xCA, 0xCB, 0xCD, 0xCE, 0xCF,
        }
        offset = 2
        while offset + 4 <= len(data):
            while offset < len(data) and data[offset] != 0xFF:
                offset += 1
            while offset < len(data) and data[offset] == 0xFF:
                offset += 1
            if offset >= len(data):
                break
            marker = data[offset]
            offset += 1
            if marker in (0xD8, 0xD9) or 0xD0 <= marker <= 0xD7:
                continue
            if offset + 2 > len(data):
                break
            segment_length = int.from_bytes(data[offset:offset + 2], "big")
            if segment_length < 2 or offset + segment_length > len(data):
                break
            if marker in sof_markers and segment_length >= 7:
                height = int.from_bytes(data[offset + 3:offset + 5], "big")
                width = int.from_bytes(data[offset + 5:offset + 7], "big")
                if width > 0 and height > 0:
                    return width, height
                break
            offset += segment_length
    raise ValueError("unable to read image dimensions")


def _window_starts(full: int, window: int, overlap: float) -> list[int]:
    if window >= full:
        return [0]
    step = max(1, int(round(window * (1.0 - overlap))))
    starts = list(range(0, full - window + 1, step))
    last = full - window
    if starts[-1] != last:
        starts.append(last)
    return starts


def _bounded_window_starts(full: int, window: int, count: int) -> list[int]:
    """Cover both ends without gaps when the normal overlap plan exceeds a cap."""
    if window >= full or count <= 1:
        return [0]
    return [
        int(round(index * (full - window) / (count - 1)))
        for index in range(count)
    ]


def _outside_in(items: list[PlateImagePass]) -> list[PlateImagePass]:
    ordered: list[PlateImagePass] = []
    left, right = 0, len(items) - 1
    while left <= right:
        ordered.append(items[left])
        if left != right:
            ordered.append(items[right])
        left += 1
        right -= 1
    return ordered


def _outer_padding(
    x: int,
    y: int,
    width: int,
    height: int,
    image_width: int,
    image_height: int,
    ratio: float,
) -> tuple[int, int, int, int]:
    horizontal = max(1, int(round(width * ratio)))
    vertical = max(1, int(round(height * ratio)))
    return (
        horizontal if x == 0 else 0,
        vertical if y == 0 else 0,
        horizontal if x + width == image_width else 0,
        vertical if y + height == image_height else 0,
    )


def plan_plate_passes(
    width: int,
    height: int,
    max_passes: int = 9,
    overlap: float = 0.35,
    edge_pad_ratio: float = 0.12,
) -> list[PlateImagePass]:
    if width <= 0 or height <= 0:
        raise ValueError("image dimensions must be positive")

    passes = [PlateImagePass("global", 0, 0, width, height)]
    aspect = width / height
    aspect_passes: list[PlateImagePass] = []
    base_width, base_height = width, height
    if aspect > 16.0 / 9.0 + 0.05:
        base_width = min(width, max(1, int(round(height * 16.0 / 9.0))))
        for index, x in enumerate(_window_starts(width, base_width, overlap)):
            padding = _outer_padding(
                x, 0, base_width, height, width, height, edge_pad_ratio
            )
            aspect_passes.append(
                PlateImagePass(
                    f"aspect-{index}", x, 0, base_width, height, *padding
                )
            )
    elif aspect < 16.0 / 9.0 - 0.05:
        base_height = min(height, max(1, int(round(width * 9.0 / 16.0))))
        for index, y in enumerate(_window_starts(height, base_height, overlap)):
            padding = _outer_padding(
                0, y, width, base_height, width, height, edge_pad_ratio
            )
            aspect_passes.append(
                PlateImagePass(
                    f"aspect-{index}", 0, y, width, base_height, *padding
                )
            )

    detail_passes: list[PlateImagePass] = []
    if width >= 1280 and height >= 720:
        detail_width = max(1, int(round(base_width * 0.70)))
        detail_height = max(1, int(round(base_height * 0.70)))
        corners = (
            ("tl", 0, 0),
            ("tr", width - detail_width, 0),
            ("bl", 0, height - detail_height),
            ("br", width - detail_width, height - detail_height),
        )
        for name, x, y in corners:
            padding = _outer_padding(
                x, y, detail_width, detail_height,
                width, height, edge_pad_ratio
            )
            detail_passes.append(
                PlateImagePass(
                    f"detail-{name}", x, y, detail_width, detail_height,
                    *padding,
                )
            )

    # Typical photos are bounded at five passes. Very long/tall images keep enough
    # aspect windows to cover the complete long axis, up to the configured limit.
    extreme = aspect > 2.4 or aspect < 0.75
    candidates: list[PlateImagePass] = []
    if extreme:
        budget = max(0, max_passes - 1)
        if len(aspect_passes) > budget and budget > 1:
            if base_width < width:
                starts = _bounded_window_starts(width, base_width, budget)
                aspect_passes = [
                    PlateImagePass(
                        f"aspect-capped-{index}", x, 0, base_width, height,
                        *_outer_padding(
                            x, 0, base_width, height,
                            width, height, edge_pad_ratio
                        ),
                    )
                    for index, x in enumerate(starts)
                ]
            else:
                starts = _bounded_window_starts(height, base_height, budget)
                aspect_passes = [
                    PlateImagePass(
                        f"aspect-capped-{index}", 0, y, width, base_height,
                        *_outer_padding(
                            0, y, width, base_height,
                            width, height, edge_pad_ratio
                        ),
                    )
                    for index, y in enumerate(starts)
                ]
        candidates.extend(_outside_in(aspect_passes))
        candidates.extend(detail_passes)
    if not extreme:
        for index in range(max(len(aspect_passes), len(detail_passes))):
            if index < len(aspect_passes):
                candidates.append(aspect_passes[index])
            if index < len(detail_passes):
                candidates.append(detail_passes[index])
        max_passes = min(max_passes, 5)

    for image_pass in candidates:
        if len(passes) >= max(1, max_passes):
            break
        key = (image_pass.x, image_pass.y, image_pass.width, image_pass.height,
               image_pass.pad_left, image_pass.pad_top,
               image_pass.pad_right, image_pass.pad_bottom)
        if any(
            key == (p.x, p.y, p.width, p.height, p.pad_left, p.pad_top,
                    p.pad_right, p.pad_bottom)
            for p in passes
        ):
            continue
        passes.append(image_pass)
    return passes


def _box_geometry(box: list[float]) -> tuple[float, float]:
    return max(0.0, box[2] - box[0]), max(0.0, box[3] - box[1])


def _box_similarity(first: list[float], second: list[float]) -> float:
    ix1 = max(first[0], second[0])
    iy1 = max(first[1], second[1])
    ix2 = min(first[2], second[2])
    iy2 = min(first[3], second[3])
    intersection = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    first_width, first_height = _box_geometry(first)
    second_width, second_height = _box_geometry(second)
    first_area = first_width * first_height
    second_area = second_width * second_height
    if intersection <= 0.0 or min(first_area, second_area) <= 0.0:
        return 0.0
    union = first_area + second_area - intersection
    iou = intersection / union if union > 0 else 0.0
    containment = intersection / min(first_area, second_area)
    return max(iou, containment * 0.70)


def map_plate_box(
    box: list[float], image_pass: PlateImagePass, image_width: int, image_height: int
) -> list[int]:
    if len(box) != 4:
        raise ValueError("plate box must contain four values")
    if image_pass.name == "global":
        scale = min(1280.0 / image_width, 720.0 / image_height)
        pad_x = (1280.0 - image_width * scale) / 2.0
        pad_y = (720.0 - image_height * scale) / 2.0
        mapped = [
            (float(box[0]) - pad_x) / scale,
            (float(box[1]) - pad_y) / scale,
            (float(box[2]) - pad_x) / scale,
            (float(box[3]) - pad_y) / scale,
        ]
    else:
        virtual_width = float(image_pass.virtual_width)
        virtual_height = float(image_pass.virtual_height)
        mapped = [
            image_pass.x + float(box[0]) * virtual_width / 1280.0 - image_pass.pad_left,
            image_pass.y + float(box[1]) * virtual_height / 720.0 - image_pass.pad_top,
            image_pass.x + float(box[2]) * virtual_width / 1280.0 - image_pass.pad_left,
            image_pass.y + float(box[3]) * virtual_height / 720.0 - image_pass.pad_top,
        ]
    return [
        max(0, min(image_width, int(round(mapped[0])))),
        max(0, min(image_height, int(round(mapped[1])))),
        max(0, min(image_width, int(round(mapped[2])))),
        max(0, min(image_height, int(round(mapped[3])))),
    ]


def merge_plate_results(
    pass_results: list[tuple[PlateImagePass, dict[str, Any]]],
    image_width: int,
    image_height: int,
    max_detections: int = 24,
) -> dict[str, Any]:
    if not pass_results:
        raise ImageInferenceError("车牌图片识别没有返回可用结果")

    merged = dict(pass_results[0][1])
    global_candidates: list[dict[str, Any]] = []
    extra_candidates: list[tuple[bool, float, float, dict[str, Any]]] = []
    for pass_index, (image_pass, result) in enumerate(pass_results):
        detections = result.get("detections", [])
        if not isinstance(detections, list):
            continue
        for detection in detections:
            if not isinstance(detection, dict):
                continue
            raw_box = detection.get("box")
            if not isinstance(raw_box, list) or len(raw_box) != 4:
                raw_box = [
                    detection.get("x1"),
                    detection.get("y1"),
                    detection.get("x2"),
                    detection.get("y2"),
                ]
            try:
                numeric_box = [float(value) for value in raw_box]
                mapped_box = map_plate_box(
                    numeric_box, image_pass, image_width, image_height
                )
            except (TypeError, ValueError):
                continue
            if mapped_box[2] <= mapped_box[0] or mapped_box[3] <= mapped_box[1]:
                continue
            candidate = dict(detection)
            candidate["box"] = mapped_box
            candidate["x1"], candidate["y1"], candidate["x2"], candidate["y2"] = mapped_box
            candidate["image_pass"] = image_pass.name
            if pass_index == 0:
                global_candidates.append(candidate)
                continue

            edge = 18.0
            touches_internal_edge = (
                (image_pass.x > 0 and numeric_box[0] <= edge)
                or (image_pass.y > 0 and numeric_box[1] <= edge)
                or (
                    image_pass.x + image_pass.width < image_width
                    and numeric_box[2] >= 1280.0 - edge
                )
                or (
                    image_pass.y + image_pass.height < image_height
                    and numeric_box[3] >= 720.0 - edge
                )
            )
            width, height = _box_geometry([float(value) for value in mapped_box])
            try:
                confidence = float(candidate.get("det_conf", 0.0))
            except (TypeError, ValueError):
                confidence = 0.0
            extra_candidates.append(
                (touches_internal_edge, -(width * height), -confidence, candidate)
            )

    accepted = list(global_candidates)
    extra_candidates.sort(key=lambda item: item[:3])
    for _, _, _, candidate in extra_candidates:
        candidate_box = [float(value) for value in candidate["box"]]
        if any(
            _box_similarity(
                candidate_box, [float(value) for value in existing["box"]]
            ) >= 0.40
            for existing in accepted
        ):
            continue
        accepted.append(candidate)
        if len(accepted) >= max_detections:
            break

    merged["detections"] = accepted[:max_detections]
    merged["pass_count"] = len(pass_results)
    merged["strategy"] = "global-preserving-adaptive"
    return merged


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

    def _run_decode_command(
        self, command: list[str], raw_path: Path, timeout: float | None = None
    ) -> None:
        run_timeout = min(timeout or self.config.timeout, 20.0)
        if run_timeout <= 0:
            raise ImageInferenceError("图片识别已超时")
        try:
            completed = subprocess.run(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=run_timeout,
                check=False,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise ImageInferenceError("板端图片解码失败") from exc
        expected = 1280 * 720 * 4
        if completed.returncode != 0 or not raw_path.is_file() or raw_path.stat().st_size != expected:
            detail = completed.stderr.decode("utf-8", "replace").strip()[-500:]
            raise ImageInferenceError(f"无法把图片转换为 1280x720 BGRx: {detail}")

    def _decode(self, image_path: Path, raw_path: Path, timeout: float | None = None) -> None:
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
        self._run_decode_command(command, raw_path, timeout)

    def _decode_plate_pass(
        self,
        image_path: Path,
        raw_path: Path,
        image_pass: PlateImagePass,
        image_width: int,
        image_height: int,
        timeout: float | None = None,
    ) -> None:
        if image_pass.name == "global":
            self._decode(image_path, raw_path, timeout)
            return

        command = [
            self.config.gst_launch,
            "-q",
            "filesrc", f"location={image_path}",
            "!", "decodebin",
            "!", "videoconvert",
            "!", "videocrop",
            f"left={image_pass.x}",
            f"right={max(0, image_width - image_pass.x - image_pass.width)}",
            f"top={image_pass.y}",
            f"bottom={max(0, image_height - image_pass.y - image_pass.height)}",
            "!", "videoscale",
        ]
        if any((image_pass.pad_left, image_pass.pad_top,
                image_pass.pad_right, image_pass.pad_bottom)):
            content_width = max(
                1, int(round(1280.0 * image_pass.width / image_pass.virtual_width))
            )
            content_height = max(
                1, int(round(720.0 * image_pass.height / image_pass.virtual_height))
            )
            output_left = max(
                0, int(round(1280.0 * image_pass.pad_left / image_pass.virtual_width))
            )
            output_top = max(
                0, int(round(720.0 * image_pass.pad_top / image_pass.virtual_height))
            )
            output_right = max(0, 1280 - output_left - content_width)
            output_bottom = max(0, 720 - output_top - content_height)
            command.extend([
                "!",
                f"video/x-raw,format=BGRx,width={content_width},height={content_height},pixel-aspect-ratio=1/1",
                "!", "videobox",
                f"left={-output_left}", f"right={-output_right}",
                f"top={-output_top}", f"bottom={-output_bottom}",
                "!", "video/x-raw,format=BGRx,width=1280,height=720,pixel-aspect-ratio=1/1",
            ])
        else:
            command.extend([
                "!", "video/x-raw,format=BGRx,width=1280,height=720,pixel-aspect-ratio=1/1",
            ])
        command.extend(["!", "filesink", f"location={raw_path}"])
        self._run_decode_command(command, raw_path, timeout)

    def _plate_command(
        self,
        raw_path: Path | None,
        socket_path: Path,
        *,
        raw_list_path: Path | None = None,
        input_repeat: int | None = None,
    ) -> list[str]:
        model = self.config.model_root
        plate_type_model = self.config.plate_type_model or (
            model / "plate_type_classifier_5color_large_green_v2_rk3568_fp16_opt0.rknn"
        )
        if (raw_path is None) == (raw_list_path is None):
            raise ValueError("exactly one static plate input must be selected")
        command = [
            str(self.config.plate_driver),
            "--plate-model", str(model / "best_fp16_5color_largegreen_scorex256_rk3568.rknn"),
            "--ocr-blue-model", str(model / "pplcnet_blue_v3_rk3568_fp16.rknn"),
            "--ocr-green-model", str(model / "pplcnet_green_v2_b1plus_rk3568_fp16.rknn"),
            "--ocr-yellow-model", str(model / "pplcnet_yellow_all_single_v1_rk3568_fp16.rknn"),
            "--ocr-police-model", str(model / "pplcnet_police_v5_whiteexpand_rk3568_fp16.rknn"),
            "--ocr-embassy-model", str(model / "pplcnet_black_unified_v1_rk3568_fp16.rknn"),
            "--plate-type-classifier-model", str(plate_type_model),
            "--plate-type-classifier-min-conf", "0.95",
            "--plate-type-classifier-special-min-conf", "0.70",
            "--ocr-blue-keys", str(model / "special_keys.txt"),
            "--ocr-green-keys", str(model / "pplcnet_green_keys.txt"),
            "--ocr-yellow-keys", str(model / "yellow_keys.txt"),
            "--ocr-police-keys", str(model / "police_keys.txt"),
            "--ocr-embassy-keys", str(model / "black_unified_keys.txt"),
            "--ocr-keys", str(model / "special_keys.txt"),
            "--det-resize", "letterbox",
            "--ocr-preproc", "gray",
            "--fps", "30",
            "--det-score-scale", "256",
            "--min-plate-conf", "0.35",
            "--plate-nms-iou", "0.35",
            "--plate-max-det", "24",
            "--source", "fpga",
        ]
        if raw_list_path is not None:
            command.extend([
                "--input-bgrx-list", str(raw_list_path),
                "--input-bgrx-repeat", str(
                    input_repeat or self.config.plate_input_repeat
                ),
                "--frames", "0",
            ])
        else:
            command.extend(["--input-bgrx", str(raw_path), "--frames", "600"])
        command.extend([
            "--no-display",
            "--control-socket", str(socket_path),
        ])
        return command

    def _run_plate(
        self, raw_path: Path, work: Path, timeout: float | None = None
    ) -> dict[str, Any]:
        socket_path = work / "plate.sock"
        log_path = work / "plate.log"
        deadline = time.monotonic() + (timeout or self.config.timeout)
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

    def _run_plate_batch(
        self,
        raw_paths: list[Path],
        work: Path,
        timeout: float | None = None,
    ) -> dict[int, dict[str, Any]]:
        if not raw_paths:
            raise ImageInferenceError("没有可供车牌识别的图片窗口")
        repeat = self.config.plate_input_repeat
        manifest_path = work / "plate-inputs.txt"
        socket_path = work / "plate.sock"
        log_path = work / "plate.log"
        manifest_path.write_text(
            "".join(f"{path}\n" for path in raw_paths), encoding="utf-8"
        )
        deadline = time.monotonic() + (timeout or self.config.timeout)
        results: dict[int, dict[str, Any]] = {}
        last_result_sequence = 0
        with log_path.open("wb") as log:
            try:
                process = subprocess.Popen(
                    self._plate_command(
                        None,
                        socket_path,
                        raw_list_path=manifest_path,
                        input_repeat=repeat,
                    ),
                    cwd=self.config.arm_root,
                    stdout=log,
                    stderr=subprocess.STDOUT,
                )
            except OSError as exc:
                raise ImageInferenceError("无法启动批量车牌识别程序") from exc
            try:
                while time.monotonic() < deadline and len(results) < len(raw_paths):
                    if socket_path.exists():
                        try:
                            result = _socket_json(
                                socket_path, {"op": "results"}, 1.0
                            )
                        except (OSError, ImageInferenceError):
                            result = {}
                        sequence = result.get("sequence")
                        input_sequence = result.get("input_sequence")
                        if (
                            completed_plate_result(result)
                            and isinstance(sequence, int)
                            and sequence != last_result_sequence
                            and isinstance(input_sequence, int)
                            and not isinstance(input_sequence, bool)
                        ):
                            last_result_sequence = sequence
                            try:
                                index = plate_input_index(
                                    input_sequence, len(raw_paths), repeat
                                )
                            except ValueError:
                                pass
                            else:
                                results.setdefault(index, result)
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
        if results:
            return results
        detail = log_path.read_text("utf-8", errors="replace")[-1000:]
        raise ImageInferenceError(f"批量车牌图片识别没有完成: {detail}")

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

    def _run_pedestrian_batch(
        self, frames_dir: Path, output_dir: Path, work: Path
    ) -> list[dict[str, Any]]:
        """Run the cplus driver once over a directory of BGRX frames.

        Loads the models a single time and processes every frame, which is
        much faster than restarting the driver per frame. Returns a list of
        {result, mask_jpeg} ordered by frame index.
        """
        model = self.config.model_root
        output_dir.mkdir(parents=True, exist_ok=True)
        command = [
            str(self.config.pedestrian_driver),
            "--det-model", str(model / "yolov5nu_coco_rk3568_fp16_20260710.rknn"),
            "--seg-model", str(model / "mapillary_cplus_ground_4class_v2_rk3568_fp16_20260710.rknn"),
            "--input-frames-dir", str(frames_dir),
            "--output-frames-dir", str(output_dir),
            "--width", "1280",
            "--height", "720",
            "--display", "0",
            "--always-segment",
        ]
        try:
            completed = subprocess.run(
                command,
                cwd=self.config.arm_root,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=self.config.timeout,
                check=False,
                text=True,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            raise ImageInferenceError("行人批量检测程序启动或运行失败") from exc
        if completed.returncode != 0:
            detail = (completed.stderr or completed.stdout)[-1000:]
            raise ImageInferenceError(f"行人批量检测失败: {detail}")
        # Parse every JSON line keyed by frame index.
        results_by_frame: dict[int, dict[str, Any]] = {}
        for line in completed.stdout.splitlines():
            line = line.strip()
            if not line or not line.startswith("{"):
                continue
            try:
                value = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(value, dict) and isinstance(value.get("frame"), int):
                results_by_frame[value["frame"]] = value
        if not results_by_frame:
            detail = completed.stdout[-1000:]
            raise ImageInferenceError(f"行人批量检测没有返回 JSON 结果: {detail}")
        expected = 1280 * 720 * 4
        ordered: list[dict[str, Any]] = []
        for index in sorted(results_by_frame):
            result = results_by_frame[index]
            bgrx_path = output_dir / f"frame{index:05d}.bgrx"
            jpeg_path = output_dir / f"frame{index:05d}.jpg"
            if not bgrx_path.is_file() or bgrx_path.stat().st_size != expected:
                # Skip frames without a rendered overlay but keep the JSON.
                ordered.append({"result": result, "mask_jpeg": None})
                continue
            ordered.append({"result": result, "mask_jpeg": self._encode_bgrx_jpeg(bgrx_path, jpeg_path)})
        return ordered

    def run(self, mode: str, image: bytes, media_type: str, attach_source_image: bool = False) -> dict[str, Any]:
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
                image_path.write_bytes(image)
                rendered_jpeg: bytes | None = None
                if mode == "plate":
                    try:
                        image_width, image_height = image_dimensions(image)
                    except ValueError as exc:
                        raise ImageInferenceError("无法读取上传图片的尺寸") from exc
                    image_passes = plan_plate_passes(
                        image_width,
                        image_height,
                        max_passes=self.config.plate_max_passes,
                        overlap=self.config.plate_overlap,
                        edge_pad_ratio=self.config.plate_edge_pad_ratio,
                    )
                    deadline = time.monotonic() + self.config.timeout
                    decoded_passes: list[tuple[PlateImagePass, Path]] = []
                    pass_warnings: list[str] = []
                    for index, image_pass in enumerate(image_passes):
                        remaining = deadline - time.monotonic()
                        if index > 0 and remaining < 2.0:
                            pass_warnings.append("后续局部检查因总时限跳过")
                            break
                        pass_work = work / f"plate-pass-{index}"
                        pass_work.mkdir()
                        raw_path = pass_work / "input.bgrx"
                        try:
                            self._decode_plate_pass(
                                image_path,
                                raw_path,
                                image_pass,
                                image_width,
                                image_height,
                                timeout=remaining,
                            )
                            decoded_passes.append((image_pass, raw_path))
                        except ImageInferenceError as exc:
                            if index == 0:
                                raise
                            pass_warnings.append(
                                f"{image_pass.name} 局部检查未完成: {exc}"
                            )
                    remaining = deadline - time.monotonic()
                    if remaining <= 0:
                        raise ImageInferenceError("图片识别已超时")
                    batch_results = self._run_plate_batch(
                        [raw_path for _, raw_path in decoded_passes],
                        work,
                        timeout=remaining,
                    )
                    if 0 not in batch_results:
                        raise ImageInferenceError("全图检查没有返回结果")
                    pass_results = [
                        (image_pass, batch_results[index])
                        for index, (image_pass, _) in enumerate(decoded_passes)
                        if index in batch_results
                    ]
                    missing_names = [
                        image_pass.name
                        for index, (image_pass, _) in enumerate(decoded_passes)
                        if index not in batch_results
                    ]
                    if missing_names:
                        pass_warnings.append(
                            "以下局部检查未返回结果: " + ", ".join(missing_names)
                        )
                    for _, raw_path in decoded_passes:
                        raw_path.unlink(missing_ok=True)
                    result = merge_plate_results(
                        pass_results, image_width, image_height
                    )
                    result["planned_pass_count"] = len(image_passes)
                    if pass_warnings:
                        result["pass_warnings"] = pass_warnings
                    frame_width, frame_height = image_width, image_height
                else:
                    raw_path = work / "input.bgrx"
                    self._decode(image_path, raw_path)
                    result, rendered_jpeg = self._run_pedestrian(raw_path, work)
                    frame_width, frame_height = 1280, 720
                response = {
                    "ok": True,
                    "mode": mode,
                    "frame": {"width": frame_width, "height": frame_height},
                    "results": result,
                }
                if rendered_jpeg is not None:
                    response["rendered_image"] = {
                        "content_type": "image/jpeg",
                        "base64": base64.b64encode(rendered_jpeg).decode("ascii"),
                    }
                if attach_source_image:
                    if mode == "plate":
                        source_data = image
                        source_content_type = media_type
                    else:
                        source_data = self._encode_bgrx_jpeg(
                            raw_path, work / "source.jpg"
                        )
                        source_content_type = "image/jpeg"
                    response["source_image"] = {
                        "content_type": source_content_type,
                        "base64": base64.b64encode(source_data).decode("ascii"),
                    }
                return response
        finally:
            self._lock.release()

    def run_from_path(self, mode: str, file_path: Path) -> dict[str, Any]:
        """Run inference on a photo already on the board (e.g. SD card)."""
        if mode not in ("plate", "pedestrian"):
            raise ImageInferenceError("识别模式必须是 plate 或 pedestrian")
        if not file_path.is_file():
            raise ImageInferenceError(f"照片文件不存在: {file_path}")
        suffix = file_path.suffix.lower()
        if suffix in (".jpg", ".jpeg"):
            media_type = "image/jpeg"
        elif suffix == ".png":
            media_type = "image/png"
        else:
            raise ImageInferenceError("只支持 JPEG 或 PNG 照片")
        try:
            image = file_path.read_bytes()
        except OSError as exc:
            raise ImageInferenceError(f"读取照片失败: {exc}") from exc
        if not image:
            raise ImageInferenceError("照片文件为空")
        return self.run(mode, image, media_type, attach_source_image=True)


class VideoInferenceError(ImageInferenceError):
    pass


@dataclasses.dataclass(frozen=True)
class VideoInferenceConfig:
    sample_fps: float = 1.0
    max_frames: int = 600
    max_keyframes: int = 24
    extract_timeout: float = 300.0


class VideoInferenceRunner:
    """Offline video inference.

    Extracts BGRx frames with gstreamer at a reduced cadence, then runs the
    cplus pedestrian driver on each frame in single-frame offline mode. The
    driver is restarted per frame (model reload), so this is an offline batch
    path, not a realtime one.
    """

    def __init__(
        self,
        image_runner: ImageInferenceRunner,
        config: VideoInferenceConfig | None = None,
    ) -> None:
        self.image_runner = image_runner
        self.config = config or VideoInferenceConfig()

    def _frame_pipeline(self, video_path: Path, sample_fps: float, frames_dir: Path) -> list[str]:
        # Order matters for 1080p sources on RK3568: decodebin already selects
        # mppvideodec (hardware H.264 decode), but a full-resolution
        # videoconvert+videoscale on every 30 fps frame is CPU-bound. We drop
        # the frame rate FIRST (videorate on NV12 straight from the decoder),
        # so the expensive convert/scale only runs on the sampled frames.
        framerate = max(1, int(round(sample_fps))) if sample_fps >= 1.0 else 1
        location = str(frames_dir / "frame%05d.bgrx")
        return [
            self.image_runner.config.gst_launch,
            "-q",
            "filesrc",
            f"location={video_path}",
            "!",
            "decodebin",
            "!",
            "videorate",
            "!",
            f"video/x-raw,framerate={framerate}/1",
            "!",
            "videoconvert",
            "!",
            "videoscale",
            "add-borders=true",
            "!",
            "video/x-raw,format=BGRx,width=1280,height=720",
            "!",
            "multifilesink",
            f"location={location}",
        ]

    def _extract_frames(
        self,
        video_path: Path,
        sample_fps: float,
        frames_dir: Path,
        progress_callback,
    ) -> tuple[list[Path], bool]:
        # Stream the gstreamer pipeline so the caller sees extraction progress
        # instead of blocking for the whole timeout. multifilesink writes one
        # file per frame, so we count files in frames_dir as a progress signal.
        if progress_callback:
            progress_callback(0, 0, "正在抽取视频帧")
        command = self._frame_pipeline(video_path, sample_fps, frames_dir)
        try:
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
            )
        except OSError as exc:
            raise VideoInferenceError("视频抽帧启动失败") from exc
        deadline = time.monotonic() + self.config.extract_timeout
        last_seen = 0
        while True:
            if process.poll() is not None:
                break
            if time.monotonic() > deadline:
                process.kill()
                try:
                    process.wait(timeout=5.0)
                except subprocess.TimeoutExpired:
                    pass
                stderr = process.stderr.read() if process.stderr else b""
                detail = stderr.decode("utf-8", "replace").strip()[-800:]
                raise VideoInferenceError(f"视频抽帧超时（已抽 {last_seen} 帧）: {detail}")
            seen = sum(1 for _ in frames_dir.glob("frame*.bgrx"))
            if seen != last_seen:
                last_seen = seen
                if progress_callback:
                    progress_callback(seen, 0, f"正在抽取视频帧，已抽 {seen} 帧")
            time.sleep(0.5)
        stderr = process.stderr.read() if process.stderr else b""
        if process.returncode != 0:
            detail = stderr.decode("utf-8", "replace").strip()[-800:]
            raise VideoInferenceError(f"视频抽帧失败: {detail}")
        expected = 1280 * 720 * 4
        frames = sorted(frames_dir.glob("frame*.bgrx"))
        valid = [frame for frame in frames if frame.stat().st_size == expected]
        if not valid:
            detail = stderr.decode("utf-8", "replace").strip()[-800:]
            raise VideoInferenceError(
                f"视频没有抽到可用帧，可能格式不支持: {detail}" if detail else "视频没有抽到可用帧，可能格式不支持"
            )
        return valid, False

    def run_video(
        self,
        video_path: Path,
        sample_fps: float,
        progress_callback=None,
    ) -> dict[str, Any]:
        if not video_path.is_file():
            raise VideoInferenceError(f"视频文件不存在: {video_path}")
        if sample_fps <= 0:
            sample_fps = self.config.sample_fps
        framerate = max(1, int(round(sample_fps))) if sample_fps >= 1.0 else 1
        step = max(1, int(round(framerate / sample_fps))) if sample_fps < 1.0 else 1
        if not self.image_runner._lock.acquire(blocking=False):
            raise ImageInferenceBusy("已有推理任务正在运行，请稍后再试")
        try:
            with tempfile.TemporaryDirectory(prefix="cplus-video-") as directory:
                work = Path(directory)
                frames_dir = work / "frames"
                frames_dir.mkdir()
                frames, _ = self._extract_frames(
                    video_path, sample_fps, frames_dir, progress_callback
                )
                sampled = frames[::step]
                if len(sampled) > self.config.max_frames:
                    sampled = sampled[: self.config.max_frames]
                truncated = len(sampled) >= self.config.max_frames
                total = len(sampled)
                # Batch inference: one driver process loads the models once and
                # processes every frame, instead of restarting per frame.
                if progress_callback:
                    progress_callback(0, total, f"正在批量推理 {total} 帧")
                batch = self.image_runner._run_pedestrian_batch(
                    frames_dir, work / "out", work
                )
                events: list[dict[str, Any]] = []
                frames_out: list[dict[str, Any]] = []
                violation_frames = 0
                skipped_frames = 0
                decision_counts: dict[str, int] = {}
                # batch is ordered by frame index; align with sampled[].
                for index, entry in enumerate(batch):
                    if index >= total:
                        break
                    result = entry.get("result", {})
                    mask_jpeg = entry.get("mask_jpeg")
                    targets = result.get("targets") if isinstance(result, dict) else None
                    if not isinstance(targets, list):
                        targets = []
                    if mask_jpeg is None:
                        skipped_frames += 1
                    frame_targets: list[dict[str, Any]] = []
                    frame_violation = False
                    for target in targets:
                        if not isinstance(target, dict):
                            continue
                        decision = str(target.get("decision", ""))
                        reason = str(target.get("reason", ""))
                        decision_counts[decision] = decision_counts.get(decision, 0) + 1
                        suspected = is_suspected_decision(decision)
                        frame_target = {
                            "type": target.get("type"),
                            "score": target.get("score"),
                            "decision": decision,
                            "reason": reason,
                            "box": target.get("box"),
                            "ground": target.get("ground"),
                            "suspected": suspected,
                        }
                        frame_targets.append(frame_target)
                        if suspected:
                            frame_violation = True
                            events.append(
                                {
                                    "frame_index": index,
                                    "time_sec": round(index / sample_fps, 3),
                                    "type": target.get("type"),
                                    "score": target.get("score"),
                                    "decision": decision,
                                    "reason": reason,
                                    "box": target.get("box"),
                                    "ground": target.get("ground"),
                                }
                            )
                    if frame_violation:
                        violation_frames += 1
                    frames_out.append(
                        {
                            "frame_index": index,
                            "time_sec": round(index / sample_fps, 3),
                            "violation": frame_violation,
                            "jpeg": mask_jpeg,
                            "targets": frame_targets,
                        }
                    )
                    if progress_callback:
                        progress_callback(
                            index + 1,
                            total,
                            f"正在推理 {index + 1}/{total} 帧",
                        )
                summary = {
                    "total_frames": total,
                    "violation_frames": violation_frames,
                    "violation_count": len(events),
                    "skipped_frames": skipped_frames,
                    "decision_counts": decision_counts,
                    "sample_fps": sample_fps,
                    "truncated": truncated,
                }
                return {
                    "events": events,
                    "frames": frames_out,
                    "summary": summary,
                }
        finally:
            self.image_runner._lock.release()
