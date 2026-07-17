#!/usr/bin/env python3
"""Integration-style tests for the HTTPS handler without board hardware."""

from __future__ import annotations

import base64
import contextlib
import http.client
import json
import os
import socketserver
import tempfile
import threading
import time
import types
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any

import server


FAKE_JPEG = base64.b64decode(
    "/9j/4AAQSkZJRgABAgAAAQABAAD//gAQTGF2YzYxLjE5LjEwMQD/2wBDAAgICAkICQsLCwsLCw0M"
    "DQ0NDQ0NDQ0NDQ0ODg4REREODg4NDQ4OEBARERITEhERERETExQUFBgYFxccHB0iIin/xABLAAEB"
    "AAAAAAAAAAAAAAAAAAAABwEBAAAAAAAAAAAAAAAAAAAABBABAAAAAAAAAAAAAAAAAAAAABEBAAAA"
    "AAAAAAAAAAAAAAAAAP/AABEIABAAEAMBIgACEQADEQD/2gAMAwEAAhEDEQA/AJWAQA//2Q=="
)


class FakeControlHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        request = json.loads(self.rfile.readline().decode("utf-8"))
        self.server.requests.append(request)  # type: ignore[attr-defined]
        op = request.get("op")
        if op == "frame":
            header = {
                "ok": True,
                "content_type": "image/jpeg",
                "content_length": len(FAKE_JPEG),
                "source_generation": self.server.source_generation,  # type: ignore[attr-defined]
            }
            self.wfile.write(json.dumps(header).encode("utf-8") + b"\n" + FAKE_JPEG)
        elif op == "status":
            response = {
                "ok": True,
                "desired_source": "phone",
                "active_source": "fpga",
                "failover_reason": "phone_stale",
                "source_generation": self.server.source_generation,  # type: ignore[attr-defined]
                "fps": {"input": 15.0, "decode": 14.8, "infer": 9.7},
                "frame": {"age_ms": -1},
                "dropped": {"input": 3, "decode": 2, "infer": 1, "display": 1},
            }
            self.wfile.write(json.dumps(response).encode("utf-8") + b"\n")
        elif op == "results":
            response = {
                "ok": True,
                "source_generation": 7,
                "frame": {"width": 1280, "height": 720},
                "detections": [
                    {
                        "x1": 220,
                        "y1": 260,
                        "x2": 520,
                        "y2": 350,
                        "text": "TEST001",
                        "route": "blue",
                        "ocr_conf": 0.96,
                    }
                ],
            }
            self.wfile.write(json.dumps(response).encode("utf-8") + b"\n")
        elif op in ("source", "pipeline"):
            self.wfile.write(json.dumps({"ok": True, "request": request}).encode("utf-8") + b"\n")
        else:
            self.wfile.write(json.dumps({"ok": False, "error": "unsupported op"}).encode("utf-8") + b"\n")


class FakeControlServer(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True

    def __init__(self, path: str) -> None:
        self.requests: list[dict[str, Any]] = []
        self.source_generation = 7
        super().__init__(path, FakeControlHandler)


class FakeWhipHandler(BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt: str, *args: object) -> None:
        return

    def _body(self) -> bytes:
        return self.rfile.read(int(self.headers.get("Content-Length", "0")))

    def _record(self, body: bytes) -> None:
        self.server.requests.append(  # type: ignore[attr-defined]
            {
                "method": self.command,
                "path": self.path,
                "body": body,
                "content_type": self.headers.get("Content-Type"),
                "if_match": self.headers.get("If-Match"),
            }
        )

    def do_POST(self) -> None:
        body = self._body()
        self._record(body)
        answer = b"v=0\r\na=fake-answer\r\n"
        host, port = self.server.server_address  # type: ignore[attr-defined]
        self.send_response(201)
        self.send_header("Content-Type", "application/sdp")
        self.send_header("Content-Length", str(len(answer)))
        self.send_header("Location", f"http://{host}:{port}/phone/whip/session-1?token=test")
        self.send_header("ETag", '"session-etag"')
        self.send_header("Link", '<stun:stun.example.test>; rel="ice-server"')
        self.end_headers()
        self.wfile.write(answer)

    def do_PATCH(self) -> None:
        body = self._body()
        self._record(body)
        self.send_response(204)
        self.send_header("Content-Length", "0")
        self.send_header("ETag", '"new-etag"')
        self.end_headers()

    def do_DELETE(self) -> None:
        body = self._body()
        self._record(body)
        self.send_response(204)
        self.send_header("Content-Length", "0")
        self.end_headers()


class FakeWhipServer(ThreadingHTTPServer):
    daemon_threads = True

    def __init__(self) -> None:
        self.requests: list[dict[str, Any]] = []
        super().__init__(("127.0.0.1", 0), FakeWhipHandler)


class FakeImageRunner:
    def __init__(self) -> None:
        self.requests: list[tuple[str, bytes, str]] = []
        self.path_requests: list[tuple[str, str]] = []

    def run(self, mode: str, image: bytes, media_type: str, attach_source_image: bool = False) -> dict[str, Any]:
        self.requests.append((mode, image, media_type))
        response = {
            "ok": True,
            "mode": mode,
            "frame": {"width": 1280, "height": 720},
            "results": {"detections": [{"text": "TEST001"}]},
        }
        if attach_source_image:
            response["source_image"] = {
                "content_type": "image/jpeg",
                "base64": base64.b64encode(FAKE_JPEG).decode("ascii"),
            }
        return response

    def run_from_path(self, mode: str, file_path: Path) -> dict[str, Any]:
        self.path_requests.append((mode, str(file_path)))
        return {
            "ok": True,
            "mode": mode,
            "frame": {"width": 1280, "height": 720},
            "results": {"targets": [{"decision": "not_suspected", "reason": "SIDEWALK_SUPPRESSED", "box": [1, 2, 3, 4]}]},
            "source_image": {
                "content_type": "image/jpeg",
                "base64": base64.b64encode(FAKE_JPEG).decode("ascii"),
            },
        }


class FakeVideoRunner:
    def __init__(self) -> None:
        self.calls: list[tuple[str, float]] = []

    def run_video(self, video_path, sample_fps, progress_callback=None):
        self.calls.append((str(video_path), sample_fps))
        if progress_callback is not None:
            progress_callback(0, 0, "extracting")
            progress_callback(1, 2, "frame 1/2")
        return {
            "events": [
                {
                    "frame_index": 0,
                    "time_sec": 0.0,
                    "decision": "suspected_crossing_road_outside_zebra",
                    "reason": "road_dominant_without_zebra",
                }
            ],
            "frames": [
                {
                    "frame_index": 0,
                    "time_sec": 0.0,
                    "violation": True,
                    "jpeg": FAKE_JPEG,
                    "targets": [
                        {"decision": "suspected_crossing_road_outside_zebra",
                         "reason": "road_dominant_without_zebra", "suspected": True},
                    ],
                },
                {
                    "frame_index": 1,
                    "time_sec": 1.0,
                    "violation": False,
                    "jpeg": FAKE_JPEG,
                    "targets": [],
                },
            ],
            "summary": {
                "total_frames": 2,
                "violation_frames": 1,
                "violation_count": 1,
                "skipped_frames": 0,
                "full_analysis_frames": 1,
                "fast_path_frames": 1,
                "decision_counts": {"suspected": 1},
                "sample_fps": sample_fps,
                "truncated": False,
            },
        }


class FakeDriverManager:
    def __init__(self) -> None:
        self.mode = "plate"
        self.sessions: list[str] = []

    def status(self) -> dict[str, Any]:
        return {
            "ok": True,
            "mode": self.mode,
            "plate_running": self.mode == "plate",
            "pedestrian_on_demand": True,
        }

    def set_mode(self, mode: str) -> dict[str, Any]:
        self.mode = mode
        return self.status()

    @contextlib.contextmanager
    def image_session(self, mode: str):
        self.mode = mode
        self.sessions.append(mode)
        yield


class FakeFixedVideoStore:
    def __init__(self) -> None:
        self.started = []
        self.prepared = []
        self.job = {
            "id": "fixed-1", "path": "clip.mp4", "name": "clip.mp4",
            "status": "ready", "message": "Mask 已生成", "processed": 0,
            "mask_ready": True, "display": "hdmi", "error": None,
        }

    def prepare(self, path):
        self.prepared.append(path)
        self.job["path"] = path
        return dict(self.job)

    def start(self, job_id):
        self.started.append(job_id)
        self.job["status"] = "running"
        return dict(self.job)

    def get(self, job_id):
        return dict(self.job) if job_id == "fixed-1" else None


class FakeHdmiVideoRunner:
    def __init__(self) -> None:
        self.image_runner = types.SimpleNamespace(_lock=threading.Lock())
        self.preview_process = object()
        self.stopped = []
        self.stream_calls = []
        self.stream_error = None

    def prepare_fixed_video(self, video_path, work):
        mask_path = work / "fixed-mask.bin"
        mask_path.write_bytes(b"mask")
        return {"mask_path": mask_path, "preview_process": self.preview_process}

    def _stop_process(self, process):
        self.stopped.append(process)

    def run_fixed_video_stream(
        self, video_path, mask_path, result_callback, progress_callback,
        acquire_lock=True,
    ):
        self.stream_calls.append((video_path, mask_path, acquire_lock))
        if self.stream_error is not None:
            raise self.stream_error
        result_callback({"frame": 0}, 0.0)
        progress_callback(1, 0, "done")
        return {"processed": 1, "truncated": False}


class WebControlTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tempdir = tempfile.TemporaryDirectory(prefix="bgp-web-test-")
        self.socket_path = Path(self.tempdir.name) / "control.sock"
        self.control = FakeControlServer(str(self.socket_path))
        self.control_thread = threading.Thread(target=self.control.serve_forever, daemon=True)
        self.control_thread.start()

        self.whip = FakeWhipServer()
        self.whip_thread = threading.Thread(target=self.whip.serve_forever, daemon=True)
        self.whip_thread.start()

        self.image_runner = FakeImageRunner()
        self.driver_manager = FakeDriverManager()
        self.video_runner = FakeVideoRunner()
        self.fixed_video_store = FakeFixedVideoStore()

        self.sd_root = Path(self.tempdir.name) / "sdcard"
        self.sd_root.mkdir()
        (self.sd_root / "photo.jpg").write_bytes(FAKE_JPEG)
        (self.sd_root / "clip.mp4").write_bytes(b"fakevideo")
        (self.sd_root / "notes.txt").write_bytes(b"notes")
        subdir = self.sd_root / "sub"
        subdir.mkdir()
        (subdir / "a.png").write_bytes(b"pngdata")

        self.video_store = server.VideoJobStore(self.video_runner, self.driver_manager, self.sd_root)

        config = server.AppConfig(
            control_socket=self.socket_path,
            control_timeout=1.0,
            mediamtx_host="127.0.0.1",
            mediamtx_port=self.whip.server_address[1],
            mediamtx_whip_path="/phone/whip",
            mediamtx_timeout=1.0,
            frame_max_fps=8.0,
            static_root=Path(__file__).resolve().parent / "static",
            sd_root=self.sd_root,
            image_runner=self.image_runner,
            driver_manager=self.driver_manager,
            video_runner=self.video_runner,
            video_store=self.video_store,
            fixed_video_store=self.fixed_video_store,
        )
        self.web = server.create_server(("127.0.0.1", 0), server.AppState(config))
        self.web_thread = threading.Thread(target=self.web.serve_forever, daemon=True)
        self.web_thread.start()

    def tearDown(self) -> None:
        self.web.shutdown()
        self.web.server_close()
        self.whip.shutdown()
        self.whip.server_close()
        self.control.shutdown()
        self.control.server_close()
        if self.socket_path.exists():
            os.unlink(self.socket_path)
        self.tempdir.cleanup()

    def request(
        self,
        method: str,
        path: str,
        body: bytes | None = None,
        headers: dict[str, str] | None = None,
    ) -> tuple[int, dict[str, str], bytes]:
        conn = http.client.HTTPConnection("127.0.0.1", self.web.server_address[1], timeout=2)
        try:
            conn.request(method, path, body=body, headers=headers or {})
            response = conn.getresponse()
            return response.status, {key.lower(): value for key, value in response.getheaders()}, response.read()
        finally:
            conn.close()

    def test_static_application_and_security_headers(self) -> None:
        status, headers, body = self.request("GET", "/")
        self.assertEqual(status, 200)
        self.assertIn(b"cameraStage", body)
        self.assertEqual(headers["x-content-type-options"], "nosniff")
        self.assertIn("camera=(self)", headers["permissions-policy"])
        self.assertEqual(headers["cache-control"], "no-store")

    def test_status_results_and_binary_frame_proxy(self) -> None:
        status, _, body = self.request("GET", "/api/v1/status")
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)["active_source"], "fpga")

        status, _, body = self.request("GET", "/api/v1/results")
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)["frame"]["width"], 1280)
        self.assertEqual(json.loads(body)["detections"][0]["ocr_conf"], 0.96)

        status, headers, body = self.request("GET", "/api/v1/frame.jpg")
        self.assertEqual(status, 200)
        self.assertEqual(headers["content-type"], "image/jpeg")
        self.assertEqual(headers["x-source-generation"], "7")
        self.assertEqual(body, FAKE_JPEG)
        self.request("GET", "/api/v1/frame.jpg")

        frame_requests = [item for item in self.control.requests if item.get("op") == "frame"]
        self.assertEqual(len(frame_requests), 1, "server-side cache must cap upstream preview at the configured rate")

        self.control.source_generation = 8
        status, _, body = self.request("GET", "/api/v1/status")
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)["source_generation"], 8)
        status, headers, body = self.request("GET", "/api/v1/frame.jpg")
        self.assertEqual(status, 200)
        self.assertEqual(headers["x-source-generation"], "8")
        self.assertEqual(body, FAKE_JPEG)
        frame_requests = [item for item in self.control.requests if item.get("op") == "frame"]
        self.assertEqual(
            len(frame_requests),
            2,
            "a status generation change must invalidate the JPEG cache",
        )

        self.control.source_generation = 0
        client = server.ControlClient(self.socket_path, timeout=1.0)
        with self.assertRaises(server.ControlProtocolError):
            client.request_frame()

    def test_source_and_pipeline_commands_are_strict(self) -> None:
        status, _, response = self.request("GET", "/api/v1/mode")
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(response)["mode"], "plate")

        mode = json.dumps({"mode": "pedestrian"}).encode("utf-8")
        status, _, response = self.request(
            "PUT",
            "/api/v1/mode",
            mode,
            {"Content-Type": "application/json"},
        )
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(response)["mode"], "pedestrian")

        body = json.dumps({"source": "phone"}).encode("utf-8")
        status, _, response = self.request(
            "PUT",
            "/api/v1/source",
            body,
            {"Content-Type": "application/json", "Content-Length": str(len(body))},
        )
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(response)["request"], {"op": "source", "source": "phone"})

        status, _, response = self.request(
            "POST",
            "/api/v1/pipeline/restart",
            b"{}",
            {"Content-Type": "application/json"},
        )
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(response)["request"], {"op": "pipeline", "action": "restart"})

        invalid = json.dumps({"source": "auto"}).encode("utf-8")
        status, _, _ = self.request(
            "PUT",
            "/api/v1/source",
            invalid,
            {"Content-Type": "application/json", "Content-Length": str(len(invalid))},
        )
        self.assertEqual(status, 400)

        status, _, _ = self.request(
            "POST", "/api/v1/pipeline/start", b"{}", {"Content-Type": "application/json"}
        )
        self.assertEqual(status, 404)

    def test_fixed_video_is_hdmi_control_only(self) -> None:
        payload = json.dumps({"path": "clip.mp4"}).encode("utf-8")
        status, _, body = self.request(
            "POST", "/api/v1/sd/fixed-video/prepare", payload,
            {"Content-Type": "application/json"},
        )
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)["id"], "fixed-1")
        self.assertEqual(json.loads(body)["display"], "hdmi")
        self.assertEqual(self.fixed_video_store.prepared, ["clip.mp4"])

        status, _, body = self.request("GET", "/api/v1/sd/fixed-video/fixed-1")
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)["status"], "ready")

        status, _, _ = self.request("GET", "/api/v1/sd/fixed-video/fixed-1/first.jpg")
        self.assertEqual(status, 404)

        status, _, _ = self.request("GET", "/api/v1/sd/fixed-video/fixed-1/mask")
        self.assertEqual(status, 404)

        payload = b"{}"
        status, _, body = self.request(
            "POST", "/api/v1/sd/fixed-video/fixed-1/infer", payload,
            {"Content-Type": "application/json"},
        )
        self.assertEqual(status, 200)
        self.assertEqual(self.fixed_video_store.started, ["fixed-1"])

        status, _, _ = self.request("GET", "/api/v1/sd/media?path=clip.mp4")
        self.assertEqual(status, 404)

    def test_fixed_video_holds_hdmi_then_restores_plate(self) -> None:
        runner = FakeHdmiVideoRunner()
        manager = FakeDriverManager()
        store = server.FixedVideoStore(runner, manager, self.sd_root)

        ready = store.prepare("clip.mp4")
        self.assertEqual(ready["status"], "ready")
        self.assertEqual(ready["display"], "hdmi")
        self.assertEqual(manager.mode, "pedestrian")
        self.assertTrue(runner.image_runner._lock.locked())

        running = store.start(ready["id"])
        self.assertEqual(running["status"], "running")
        final = None
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            final = store.get(ready["id"])
            if final and final["status"] != "running":
                break
            time.sleep(0.01)

        self.assertIsNotNone(final)
        self.assertEqual(final["status"], "done")
        self.assertEqual(final["processed"], 1)
        self.assertEqual(manager.mode, "plate")
        self.assertEqual(runner.stopped, [runner.preview_process])
        self.assertEqual(len(runner.stream_calls), 1)
        self.assertFalse(runner.stream_calls[0][2])
        self.assertFalse(runner.image_runner._lock.locked())

    def test_fixed_video_failure_still_restores_plate(self) -> None:
        runner = FakeHdmiVideoRunner()
        runner.stream_error = server.ImageInferenceError("video stalled")
        manager = FakeDriverManager()
        store = server.FixedVideoStore(runner, manager, self.sd_root)

        ready = store.prepare("clip.mp4")
        store.start(ready["id"])
        final = None
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            final = store.get(ready["id"])
            if final and final["status"] != "running":
                break
            time.sleep(0.01)

        self.assertIsNotNone(final)
        self.assertEqual(final["status"], "error")
        self.assertIn("video stalled", final["error"])
        self.assertEqual(manager.mode, "plate")
        self.assertFalse(runner.image_runner._lock.locked())

        status, _, _ = self.request(
            "POST", "/api/v1/pipeline/pause", b"", {"Content-Length": "0"}
        )
        self.assertEqual(status, 415)
        status, _, _ = self.request(
            "POST",
            "/api/v1/pipeline/pause",
            b"{}",
            {"Content-Type": "application/json", "Origin": "https://attacker.example"},
        )
        self.assertEqual(status, 403)

        extra = json.dumps({"source": "fpga", "unexpected": True}).encode("utf-8")
        status, _, _ = self.request(
            "PUT",
            "/api/v1/source",
            extra,
            {"Content-Type": "application/json"},
        )
        self.assertEqual(status, 400)

    def test_image_upload_modes_and_validation(self) -> None:
        status, _, body = self.request(
            "POST",
            "/api/v1/image-inference",
            FAKE_JPEG,
            {"Content-Type": "image/jpeg", "X-Inference-Mode": "plate"},
        )
        self.assertEqual(status, 200)
        response = json.loads(body)
        self.assertEqual(response["mode"], "plate")
        self.assertEqual(response["results"]["detections"][0]["text"], "TEST001")
        self.assertEqual(self.image_runner.requests[-1], ("plate", FAKE_JPEG, "image/jpeg"))
        self.assertEqual(self.driver_manager.sessions[-1], "plate")

        status, _, body = self.request(
            "POST",
            "/api/v1/image-inference",
            b"png",
            {"Content-Type": "image/png", "X-Inference-Mode": "pedestrian"},
        )
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)["mode"], "pedestrian")
        self.assertEqual(self.driver_manager.sessions[-1], "pedestrian")

        status, _, _ = self.request(
            "POST",
            "/api/v1/image-inference",
            FAKE_JPEG,
            {"Content-Type": "image/jpeg", "X-Inference-Mode": "unknown"},
        )
        self.assertEqual(status, 400)

        status, _, _ = self.request(
            "POST",
            "/api/v1/image-inference",
            b"text",
            {"Content-Type": "text/plain", "X-Inference-Mode": "plate"},
        )
        self.assertEqual(status, 415)

    def test_sd_list_and_photo_inference(self) -> None:
        status, _, body = self.request("GET", "/api/v1/sd/list")
        self.assertEqual(status, 200)
        listing = json.loads(body)
        names = {entry["name"]: entry["type"] for entry in listing["entries"]}
        self.assertEqual(names["photo.jpg"], "photo")
        self.assertEqual(names["clip.mp4"], "video")
        self.assertEqual(names["sub"], "dir")

        status, _, body = self.request("GET", "/api/v1/sd/list?path=sub")
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)["path"], "sub")

        status, _, _ = self.request("GET", "/api/v1/sd/list?path=../etc")
        self.assertEqual(status, 400)

        payload = json.dumps({"path": "photo.jpg", "mode": "pedestrian"}).encode("utf-8")
        status, _, body = self.request(
            "POST",
            "/api/v1/sd/photo-inference",
            payload,
            {"Content-Type": "application/json"},
        )
        self.assertEqual(status, 200)
        response = json.loads(body)
        self.assertEqual(response["mode"], "pedestrian")
        self.assertIn("source_image", response)
        self.assertEqual(self.driver_manager.sessions[-1], "pedestrian")

        payload = json.dumps({"path": "photo.jpg", "mode": "unknown"}).encode("utf-8")
        status, _, _ = self.request(
            "POST",
            "/api/v1/sd/photo-inference",
            payload,
            {"Content-Type": "application/json"},
        )
        self.assertEqual(status, 400)

        payload = json.dumps({"path": "nope.jpg", "mode": "plate"}).encode("utf-8")
        status, _, _ = self.request(
            "POST",
            "/api/v1/sd/photo-inference",
            payload,
            {"Content-Type": "application/json"},
        )
        self.assertEqual(status, 404)

        payload = json.dumps({"path": "sub", "mode": "plate"}).encode("utf-8")
        status, _, _ = self.request(
            "POST",
            "/api/v1/sd/photo-inference",
            payload,
            {"Content-Type": "application/json"},
        )
        self.assertEqual(status, 404)

    def test_sd_video_inference_job(self) -> None:
        payload = json.dumps({"path": "clip.mp4", "sample_fps": 1}).encode("utf-8")
        status, _, body = self.request(
            "POST",
            "/api/v1/sd/video-inference",
            payload,
            {"Content-Type": "application/json"},
        )
        self.assertEqual(status, 202)
        job_id = json.loads(body)["job_id"]

        deadline = time.monotonic() + 5
        job = None
        while time.monotonic() < deadline:
            status, _, body = self.request("GET", f"/api/v1/sd/video-jobs/{job_id}")
            self.assertEqual(status, 200)
            job = json.loads(body)
            if job["status"] != "running":
                break
            time.sleep(0.05)
        self.assertIsNotNone(job)
        self.assertEqual(job["status"], "done")
        self.assertEqual(job["summary"]["violation_count"], 1)
        self.assertEqual(job["frame_count"], 2)
        self.assertEqual(self.driver_manager.sessions[-1], "pedestrian")

        status, headers, body = self.request(
            "GET", f"/api/v1/sd/video-jobs/{job_id}/frames/0.jpg"
        )
        self.assertEqual(status, 200)
        self.assertEqual(headers["content-type"], "image/jpeg")
        self.assertEqual(body, FAKE_JPEG)

        status, _, body = self.request(
            "GET", f"/api/v1/sd/video-jobs/{job_id}/frames/0/result"
        )
        self.assertEqual(status, 200)
        result = json.loads(body)
        self.assertTrue(result["violation"])
        self.assertEqual(len(result["targets"]), 1)

        status, _, body = self.request("GET", "/api/v1/sd/video-jobs")
        self.assertEqual(status, 200)
        self.assertEqual(json.loads(body)["jobs"][0]["id"], job_id)

        status, _, _ = self.request(
            "GET", f"/api/v1/sd/video-jobs/{job_id}/frames/99.jpg"
        )
        self.assertEqual(status, 404)

        payload = json.dumps({"path": "clip.mp4", "sample_fps": 20}).encode("utf-8")
        status, _, _ = self.request(
            "POST",
            "/api/v1/sd/video-inference",
            payload,
            {"Content-Type": "application/json"},
        )
        self.assertEqual(status, 400)

    def test_whip_post_patch_delete_and_location_rewrite(self) -> None:
        offer = b"v=0\r\na=fake-offer\r\n"
        status, headers, answer = self.request(
            "POST",
            "/whip/phone",
            offer,
            {"Content-Type": "application/sdp", "Accept": "application/sdp"},
        )
        self.assertEqual(status, 201)
        self.assertEqual(answer, b"v=0\r\na=fake-answer\r\n")
        self.assertEqual(headers["location"], "/whip/phone/session-1?token=test")
        self.assertEqual(headers["etag"], '"session-etag"')
        self.assertIn("ice-server", headers["link"])

        fragment = b"a=ice-ufrag:test\r\n"
        status, headers, _ = self.request(
            "PATCH",
            headers["location"],
            fragment,
            {"Content-Type": "application/trickle-ice-sdpfrag", "If-Match": '"session-etag"'},
        )
        self.assertEqual(status, 204)
        self.assertEqual(headers["etag"], '"new-etag"')

        status, _, _ = self.request(
            "DELETE",
            "/whip/phone/session-1?token=test",
            b"",
            {"Content-Length": "0", "If-Match": '"new-etag"'},
        )
        self.assertEqual(status, 204)
        self.assertEqual([item["method"] for item in self.whip.requests], ["POST", "PATCH", "DELETE"])
        self.assertEqual(self.whip.requests[1]["path"], "/phone/whip/session-1?token=test")
        self.assertEqual(self.whip.requests[1]["if_match"], '"session-etag"')

        status, _, _ = self.request(
            "POST",
            "/whip/phone/session-1",
            offer,
            {"Content-Type": "application/sdp"},
        )
        self.assertEqual(status, 404)
        status, _, _ = self.request(
            "PATCH",
            "/whip/phone",
            fragment,
            {"Content-Type": "application/trickle-ice-sdpfrag"},
        )
        self.assertEqual(status, 404)

    def test_whip_options_and_unavailable_control(self) -> None:
        status, headers, body = self.request("OPTIONS", "/whip/phone", b"", {"Content-Length": "0"})
        self.assertEqual(status, 204)
        self.assertEqual(body, b"")
        self.assertEqual(headers["accept-post"], "application/sdp")

        missing_state = server.AppState(
            server.AppConfig(control_socket=Path(self.tempdir.name) / "missing.sock", static_root=Path("."))
        )
        missing_web = server.create_server(("127.0.0.1", 0), missing_state)
        thread = threading.Thread(target=missing_web.serve_forever, daemon=True)
        thread.start()
        try:
            conn = http.client.HTTPConnection("127.0.0.1", missing_web.server_address[1], timeout=2)
            conn.request("GET", "/api/v1/status")
            response = conn.getresponse()
            self.assertEqual(response.status, 503)
            self.assertEqual(json.loads(response.read())["error"]["code"], "control_unavailable")
            conn.close()
        finally:
            missing_web.shutdown()
            missing_web.server_close()


if __name__ == "__main__":
    unittest.main(verbosity=2)
