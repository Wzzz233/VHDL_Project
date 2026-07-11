#!/usr/bin/env python3
"""End-to-end synthetic camera test through browser WHIP/WebRTC and RTSP."""

from __future__ import annotations

import argparse
import base64
import json
import os
import re
import shlex
import signal
import socket
import socketserver
import subprocess
import sys
import tempfile
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


ROOT = Path(__file__).resolve().parent
MEDIAMTX_VERSION = "1.19.2"
TEST_JPEG = base64.b64decode(
    "/9j/4AAQSkZJRgABAgAAAQABAAD//gAQTGF2YzYxLjE5LjEwMQD/2wBDAAgICAkICQsLCwsLCw0M"
    "DQ0NDQ0NDQ0NDQ0ODg4REREODg4NDQ4OEBARERITEhERERETExQUFBgYFxccHB0iIin/xABLAAEB"
    "AAAAAAAAAAAAAAAAAAAABwEBAAAAAAAAAAAAAAAAAAAABBABAAAAAAAAAAAAAAAAAAAAABEBAAAA"
    "AAAAAAAAAAAAAAAAAP/AABEIABAAEAMBIgACEQADEQD/2gAMAwEAAhEDEQA/AJWAQA//2Q=="
)


class TestFailure(RuntimeError):
    pass


@dataclass(frozen=True)
class Ports:
    web: int = 8443
    whip: int = 8889
    rtsp: int = 8554
    webrtc_udp: int = 8189


def parse_command(value: str, label: str) -> list[str]:
    command = shlex.split(value)
    if not command:
        raise TestFailure(f"{label} command is empty")
    return command


def render_mediamtx_config(ports: Ports) -> str:
    return f"""logLevel: info
logDestinations: [stdout]
api: no
metrics: no
pprof: no
playback: no
rtmp: no
hls: no
srt: no
moq: no
rtsp: yes
rtspAddress: 127.0.0.1:{ports.rtsp}
rtspTransports: [tcp]
webrtc: yes
webrtcAddress: 127.0.0.1:{ports.whip}
webrtcLocalUDPAddress: 127.0.0.1:{ports.webrtc_udp}
webrtcLocalTCPAddress: ""
webrtcAdditionalHosts: [127.0.0.1]
paths:
  phone:
    source: publisher
"""


def build_gst_y4m_command(command: list[str], output: Path, frames: int) -> list[str]:
    return [
        *command,
        "-q",
        "videotestsrc",
        f"num-buffers={frames}",
        "pattern=ball",
        "!",
        "video/x-raw,format=I420,width=1280,height=720,framerate=15/1",
        "!",
        "y4menc",
        "!",
        "filesink",
        f"location={output}",
    ]


def build_ffmpeg_command(command: list[str], rtsp_url: str, frames: int) -> list[str]:
    return [
        *command,
        "-hide_banner",
        "-loglevel",
        "info",
        "-progress",
        "pipe:1",
        "-nostats",
        "-rtsp_transport",
        "tcp",
        "-timeout",
        "3000000",
        "-i",
        rtsp_url,
        "-map",
        "0:v:0",
        "-an",
        "-frames:v",
        str(frames),
        "-f",
        "null",
        "-",
    ]


def build_chromium_args(y4m_path: Path) -> list[str]:
    return [
        "--no-sandbox",
        "--disable-dev-shm-usage",
        "--autoplay-policy=no-user-gesture-required",
        "--use-fake-ui-for-media-stream",
        "--use-fake-device-for-media-stream",
        f"--use-file-for-fake-video-capture={y4m_path}",
    ]


def assert_ffmpeg_output(
    stdout: str,
    stderr: str,
    expected_frames: int = 30,
) -> tuple[int, int]:
    video_match = re.search(
        r"Video:\s+h264(?:\s|\(|,)[^\n]*?(\d+)x(\d+)",
        stderr,
        re.IGNORECASE,
    )
    if not video_match:
        raise TestFailure("RTSP input codec is not H.264")
    width, height = (int(video_match.group(1)), int(video_match.group(2)))
    if width <= 0 or height <= 0 or abs((width / height) - (16 / 9)) > 0.02:
        raise TestFailure(f"RTSP input resolution {width}x{height} is not 16:9")
    frame_values = [
        int(value)
        for value in re.findall(r"^frame=(\d+)\s*$", stdout, re.MULTILINE)
    ]
    if not frame_values or max(frame_values) < expected_frames:
        decoded = max(frame_values) if frame_values else 0
        raise TestFailure(
            f"ffmpeg decoded {decoded} frames, expected at least {expected_frames}"
        )
    return width, height


def decode_rtsp_with_retry(
    command: list[str],
    rtsp_url: str,
    frames: int,
    timeout: float,
) -> tuple[int, int]:
    deadline = time.monotonic() + timeout
    last_output = ""
    while time.monotonic() < deadline:
        remaining = deadline - time.monotonic()
        try:
            result = subprocess.run(
                build_ffmpeg_command(command, rtsp_url, frames),
                capture_output=True,
                text=True,
                timeout=max(1.0, min(8.0, remaining)),
                check=False,
            )
        except OSError as exc:
            raise TestFailure(f"ffmpeg RTSP decode failed to run: {exc}") from exc
        except subprocess.TimeoutExpired as exc:
            last_output = (
                (exc.stdout or b"").decode("utf-8", errors="replace")
                if isinstance(exc.stdout, bytes)
                else (exc.stdout or "")
            )
            last_output += (
                (exc.stderr or b"").decode("utf-8", errors="replace")
                if isinstance(exc.stderr, bytes)
                else (exc.stderr or "")
            )
        else:
            last_output = result.stdout + result.stderr
            if result.returncode == 0:
                return assert_ffmpeg_output(
                    result.stdout,
                    result.stderr,
                    expected_frames=frames,
                )
        if time.monotonic() < deadline:
            time.sleep(min(0.5, max(0.0, deadline - time.monotonic())))
    raise TestFailure(
        f"RTSP path did not become decodable within {timeout:.1f}s\n"
        + last_output[-12000:]
    )


class FakeControlHandler(socketserver.StreamRequestHandler):
    def handle(self) -> None:
        try:
            request = json.loads(self.rfile.readline().decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            self.wfile.write(b'{"ok":false,"error":"invalid JSON"}\n')
            return
        op = request.get("op")
        if op == "frame":
            header = {
                "ok": True,
                "content_type": "image/jpeg",
                "content_length": len(TEST_JPEG),
                "source_generation": self.server.source_generation,  # type: ignore[attr-defined]
            }
            self.wfile.write(
                json.dumps(header, separators=(",", ":")).encode("utf-8")
                + b"\n"
                + TEST_JPEG
            )
            return
        if op == "source":
            source = request.get("source")
            if source in ("fpga", "phone"):
                if source != self.server.active_source:  # type: ignore[attr-defined]
                    self.server.source_generation += 1  # type: ignore[attr-defined]
                self.server.desired_source = source  # type: ignore[attr-defined]
                self.server.active_source = source  # type: ignore[attr-defined]
                self.wfile.write(b'{"ok":true}\n')
            else:
                self.wfile.write(b'{"ok":false,"error":"invalid source"}\n')
            return
        if op == "status":
            response = {
                "ok": True,
                "desired_source": self.server.desired_source,  # type: ignore[attr-defined]
                "active_source": self.server.active_source,  # type: ignore[attr-defined]
                "failover_reason": "desired_phone",
                "source_generation": self.server.source_generation,  # type: ignore[attr-defined]
                "pipeline": "running",
                "fpga_healthy": True,
                "phone_healthy": True,
                "frame": {"width": 1280, "height": 720, "age_ms": -1},
                "fps": {"input": 15.0, "decode": 15.0, "infer": 0.0},
                "frames": {"input": 0, "decoded": 0, "inferred": 0},
                "dropped": {"input": 1, "decode": 2, "infer": 3, "display": 4},
            }
            self.wfile.write(
                json.dumps(response, separators=(",", ":")).encode("utf-8")
                + b"\n"
            )
            return
        if op == "results":
            response = {
                "ok": True,
                "valid": False,
                "source_generation": self.server.source_generation,  # type: ignore[attr-defined]
                "sequence": 0,
                "input_sequence": 0,
                "frame_timestamp_us": 0,
                "infer_ms": 0.0,
                "frame": {"width": 1280, "height": 720},
                "detections": [],
            }
            self.wfile.write(
                json.dumps(response, separators=(",", ":")).encode("utf-8")
                + b"\n"
            )
            return
        if op == "pipeline":
            self.wfile.write(b'{"ok":true}\n')
            return
        self.wfile.write(b'{"ok":false,"error":"unsupported op"}\n')


class FakeControlServer(socketserver.ThreadingUnixStreamServer):
    daemon_threads = True

    def __init__(self, path: Path) -> None:
        self.desired_source = "fpga"
        self.active_source = "fpga"
        self.source_generation = 1
        super().__init__(str(path), FakeControlHandler)


class ManagedProcess:
    def __init__(
        self,
        label: str,
        command: list[str],
        log_path: Path,
        env: dict[str, str] | None = None,
    ) -> None:
        self.label = label
        self.command = command
        self.log_path = log_path
        self.env = env
        self.process: subprocess.Popen[bytes] | None = None
        self._log: Any = None

    def start(self) -> None:
        self._log = self.log_path.open("wb")
        try:
            self.process = subprocess.Popen(
                self.command,
                stdout=self._log,
                stderr=subprocess.STDOUT,
                env=self.env,
                start_new_session=True,
            )
        except OSError as exc:
            self._log.close()
            self._log = None
            raise TestFailure(f"cannot start {self.label}: {exc}") from exc

    def poll(self) -> int | None:
        return self.process.poll() if self.process else None

    def stop(self) -> None:
        process = self.process
        if process and process.poll() is None:
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                if process.poll() is None:
                    try:
                        os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                    process.wait(timeout=3)
        if self._log:
            self._log.close()
            self._log = None

    def tail(self, limit: int = 12000) -> str:
        if not self.log_path.exists():
            return ""
        data = self.log_path.read_bytes()
        return data[-limit:].decode("utf-8", errors="replace")


def require_tcp_port_free(port: int) -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
        try:
            sock.bind(("127.0.0.1", port))
        except OSError as exc:
            raise TestFailure(f"TCP port {port} is already in use") from exc


def require_udp_port_free(port: int) -> None:
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            sock.bind(("127.0.0.1", port))
        except OSError as exc:
            raise TestFailure(f"UDP port {port} is already in use") from exc


def wait_for_tcp(
    port: int,
    process: ManagedProcess,
    timeout: float,
) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if process.poll() is not None:
            raise TestFailure(
                f"{process.label} exited before opening port {port}\n"
                + process.tail()
            )
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.25):
                return
        except OSError:
            time.sleep(0.1)
    raise TestFailure(
        f"{process.label} did not open TCP port {port}\n" + process.tail()
    )


def run_checked(
    label: str,
    command: list[str],
    timeout: float,
    env: dict[str, str] | None = None,
) -> subprocess.CompletedProcess[str]:
    try:
        result = subprocess.run(
            command,
            capture_output=True,
            text=True,
            timeout=timeout,
            env=env,
            check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        raise TestFailure(f"{label} failed to run: {exc}") from exc
    if result.returncode != 0:
        raise TestFailure(
            f"{label} exited with status {result.returncode}\n"
            + (result.stdout + result.stderr)[-12000:]
        )
    return result


def make_certificate(
    command: list[str],
    certificate: Path,
    private_key: Path,
    timeout: float,
) -> None:
    run_checked(
        "OpenSSL certificate generation",
        [
            *command,
            "req",
            "-x509",
            "-newkey",
            "rsa:2048",
            "-nodes",
            "-sha256",
            "-days",
            "1",
            "-subj",
            "/CN=127.0.0.1/O=PPLCNet WHIP Test",
            "-addext",
            "subjectAltName=IP:127.0.0.1,DNS:localhost",
            "-keyout",
            str(private_key),
            "-out",
            str(certificate),
        ],
        timeout,
    )


def run_browser_and_decode(
    args: argparse.Namespace,
    y4m_path: Path,
    base_url: str,
    rtsp_url: str,
) -> tuple[int, int]:
    try:
        from playwright.sync_api import TimeoutError as PlaywrightTimeoutError
        from playwright.sync_api import sync_playwright
    except ImportError as exc:
        raise TestFailure(
            "Python Playwright is required: python3 -m pip install playwright"
        ) from exc

    console_messages: list[str] = []
    with sync_playwright() as playwright:
        browser = playwright.chromium.launch(
            executable_path=args.chromium_executable,
            headless=not args.headful,
            args=build_chromium_args(y4m_path),
        )
        context = browser.new_context(
            ignore_https_errors=True,
            permissions=["camera"],
        )
        page = context.new_page()
        page.on(
            "console",
            lambda message: console_messages.append(
                f"{message.type}: {message.text}"
            ),
        )
        try:
            page.goto(
                base_url,
                wait_until="domcontentloaded",
                timeout=int(args.browser_timeout * 1000),
            )
            page.wait_for_function(
                "() => typeof window.startPhoneCamera === 'function'",
                timeout=int(args.browser_timeout * 1000),
            )
            page.wait_for_function(
                """() => (
                    document.getElementById("frameAge").textContent === "--" &&
                    document.getElementById("inputDrops").textContent === "1" &&
                    document.getElementById("decodeDrops").textContent === "2" &&
                    document.getElementById("inferDrops").textContent === "3" &&
                    document.getElementById("displayDrops").textContent === "4"
                )""",
                timeout=int(args.browser_timeout * 1000),
            )
            page.evaluate("void window.startPhoneCamera()")
            page.wait_for_function(
                """() => {
                    const state = document.getElementById("phoneCameraState");
                    return (
                        state &&
                        state.textContent === "正在发送" &&
                        typeof publisher !== "undefined" &&
                        publisher &&
                        publisher.peer.connectionState === "connected"
                    );
                }""",
                timeout=int(args.browser_timeout * 1000),
            )
            state = page.locator("#phoneCameraState").inner_text()
            if state != "正在发送":
                raise TestFailure(f"browser publisher state is {state!r}")

            page.wait_for_function(
                """() => (
                    document.getElementById("desiredSource").textContent ===
                    "手机" &&
                    document.getElementById("framePreview").dataset.ready ===
                    "true"
                )""",
                timeout=int(args.browser_timeout * 1000),
            )
            old_generation = page.evaluate("latestStatusGeneration")
            old_epoch = page.evaluate("visualEpoch")
            stale_results = json.dumps(
                {
                    "ok": True,
                    "source_generation": int(old_generation),
                    "frame": {"width": 1280, "height": 720},
                    "detections": [
                        {
                            "x1": 220,
                            "y1": 260,
                            "x2": 520,
                            "y2": 350,
                            "text": "STALE001",
                            "route": "blue",
                            "ocr_conf": 0.96,
                        }
                    ],
                },
                separators=(",", ":"),
            )
            page.route(
                "**/api/v1/results",
                lambda route: route.fulfill(
                    status=200,
                    content_type="application/json",
                    body=stale_results,
                ),
            )
            page.route(
                "**/api/v1/frame.jpg*",
                lambda route: route.fulfill(
                    status=200,
                    headers={
                        "Content-Type": "image/jpeg",
                        "X-Source-Generation": str(old_generation),
                    },
                    body=TEST_JPEG,
                ),
            )
            page.wait_for_function(
                """() => {
                    const canvas = document.getElementById("detectionOverlay");
                    const pixels = canvas.getContext("2d").getImageData(
                        0, 0, canvas.width, canvas.height
                    ).data;
                    return (
                        document.getElementById("detectionCount").textContent ===
                        "1" &&
                        pixels.some((value, index) => index % 4 === 3 && value)
                    );
                }""",
                timeout=int(args.browser_timeout * 1000),
            )
            page.evaluate("setDesiredSource('fpga')")
            page.wait_for_function(
                """(expected) => {
                    const canvas = document.getElementById("detectionOverlay");
                    const pixels = canvas.getContext("2d").getImageData(
                        0, 0, canvas.width, canvas.height
                    ).data;
                    return (
                        visualEpoch > expected.epoch &&
                        latestStatusGeneration !== expected.generation &&
                        document.getElementById("desiredSource").textContent ===
                        "OV5640" &&
                        document.getElementById("detectionCount").textContent ===
                        "0" &&
                        currentFrameUrl === null &&
                        !document.getElementById("framePreview").hasAttribute(
                            "src"
                        ) &&
                        document.getElementById("framePreview").dataset.ready ===
                        "false" &&
                        document.getElementById("frameState").hidden === false &&
                        document.getElementById("frameState").textContent ===
                        "等待新源画面" &&
                        !pixels.some(
                            (value, index) => index % 4 === 3 && value
                        )
                    );
                }""",
                arg={"epoch": old_epoch, "generation": old_generation},
                timeout=int(args.browser_timeout * 1000),
            )
            page.unroute("**/api/v1/results")
            page.unroute("**/api/v1/frame.jpg*")
            page.wait_for_function(
                """() => (
                    document.getElementById("framePreview").dataset.ready ===
                    "true"
                )""",
                timeout=int(args.browser_timeout * 1000),
            )

            page.evaluate(
                """() => {
                    window.__whipTestOldPeer = publisher.peer;
                    publisher.peer.close();
                }"""
            )
            page.wait_for_function(
                """() => (
                    typeof publisher !== "undefined" &&
                    publisher &&
                    publisher.peer !== window.__whipTestOldPeer &&
                    publisher.peer.connectionState === "connected" &&
                    document.getElementById("phoneCameraState").textContent ===
                    "正在发送"
                )""",
                timeout=int(args.browser_timeout * 1000),
            )
            page.evaluate("delete window.__whipTestOldPeer")
            if page.locator("#desiredSource").inner_text() != "OV5640":
                raise TestFailure(
                    "peer recovery changed the explicit OV5640 selection"
                )

            page.evaluate("window.dispatchEvent(new Event('pagehide'))")
            page.wait_for_function(
                """() => (
                    document.getElementById("phoneCameraState").textContent ===
                    "等待恢复"
                )""",
                timeout=int(args.browser_timeout * 1000),
            )
            page.wait_for_timeout(300)
            page.evaluate("window.dispatchEvent(new Event('pageshow'))")
            page.wait_for_function(
                """() => (
                    document.getElementById("phoneCameraState").textContent ===
                    "正在发送"
                )""",
                timeout=int(args.browser_timeout * 1000),
            )
            if page.locator("#desiredSource").inner_text() != "OV5640":
                raise TestFailure(
                    "page recovery changed the explicit OV5640 selection"
                )

            decoded_size = decode_rtsp_with_retry(
                args.ffmpeg_command,
                rtsp_url,
                args.decode_frames,
                args.decode_timeout,
            )
            page.evaluate("window.stopPhoneCamera()")
            page.evaluate("window.dispatchEvent(new Event('pageshow'))")
            page.wait_for_timeout(500)
            if page.locator("#phoneCameraState").inner_text() != "未启动":
                raise TestFailure("user Stop did not clear camera recovery intent")
            return decoded_size
        except PlaywrightTimeoutError as exc:
            state = page.locator("#phoneCameraState").inner_text()
            console = "\n".join(console_messages[-30:])
            raise TestFailure(
                f"browser did not reach the publishing state; state={state!r}\n"
                + console
            ) from exc
        finally:
            context.close()
            browser.close()


def run_test(args: argparse.Namespace) -> None:
    ports = Ports(
        web=args.web_port,
        whip=args.whip_port,
        rtsp=args.rtsp_port,
        webrtc_udp=args.webrtc_udp_port,
    )
    for port in (ports.web, ports.whip, ports.rtsp):
        require_tcp_port_free(port)
    require_udp_port_free(ports.webrtc_udp)

    commands = {
        "MediaMTX": args.mediamtx_command,
        "GStreamer": args.gst_launch_command,
        "ffmpeg": args.ffmpeg_command,
        "OpenSSL": args.openssl_command,
        "web server": args.server_command,
    }
    for label, command in commands.items():
        if not command:
            raise TestFailure(f"{label} command is empty")

    work_root = Path(args.work_root).resolve() if args.work_root else None
    with tempfile.TemporaryDirectory(
        prefix="pplcnet-whip-test-",
        dir=work_root,
    ) as temp_name:
        temp = Path(temp_name)
        y4m_path = temp / "camera.y4m"
        certificate = temp / "server.crt"
        private_key = temp / "server.key"
        control_socket = temp / "control.sock"
        mediamtx_config = temp / "mediamtx.yml"
        mediamtx_log = temp / "mediamtx.log"
        web_log = temp / "web.log"

        print("[1/5] generating 1280x720@15 synthetic camera Y4M")
        run_checked(
            "GStreamer videotestsrc generation",
            build_gst_y4m_command(
                args.gst_launch_command,
                y4m_path,
                args.source_frames,
            ),
            args.tool_timeout,
            env=os.environ.copy(),
        )
        if not y4m_path.exists() or y4m_path.stat().st_size < 1024:
            raise TestFailure("GStreamer did not produce a valid Y4M fixture")
        y4m_header = y4m_path.open("rb").readline()
        if not all(token in y4m_header for token in (b"W1280", b"H720", b"F15:1")):
            raise TestFailure(
                "GStreamer Y4M fixture is not exactly 1280x720 at 15fps"
            )

        print("[2/5] generating temporary HTTPS certificate")
        make_certificate(
            args.openssl_command,
            certificate,
            private_key,
            args.tool_timeout,
        )
        mediamtx_config.write_text(
            render_mediamtx_config(ports),
            encoding="utf-8",
        )

        control = FakeControlServer(control_socket)
        control_thread = threading.Thread(
            target=control.serve_forever,
            daemon=True,
        )
        control_thread.start()
        mediamtx = ManagedProcess(
            "MediaMTX",
            [*args.mediamtx_command, str(mediamtx_config)],
            mediamtx_log,
        )
        web_env = os.environ.copy()
        web_env["PYTHONDONTWRITEBYTECODE"] = "1"
        web = ManagedProcess(
            "HTTPS web control",
            [
                *args.server_command,
                str(ROOT / "server.py"),
                "--host",
                "127.0.0.1",
                "--port",
                str(ports.web),
                "--cert-file",
                str(certificate),
                "--key-file",
                str(private_key),
                "--control-socket",
                str(control_socket),
                "--mediamtx-host",
                "127.0.0.1",
                "--mediamtx-port",
                str(ports.whip),
                "--log-level",
                "WARNING",
            ],
            web_log,
            env=web_env,
        )
        try:
            print("[3/5] starting exact MediaMTX and HTTPS WHIP proxy")
            mediamtx.start()
            wait_for_tcp(ports.whip, mediamtx, args.startup_timeout)
            wait_for_tcp(ports.rtsp, mediamtx, args.startup_timeout)
            if f"v{MEDIAMTX_VERSION}" not in mediamtx.tail():
                raise TestFailure(
                    f"MediaMTX is not the required v{MEDIAMTX_VERSION}\n"
                    + mediamtx.tail()
                )
            web.start()
            wait_for_tcp(ports.web, web, args.startup_timeout)

            print("[4/5] publishing fake camera through browser WHIP/WebRTC")
            decoded_width, decoded_height = run_browser_and_decode(
                args,
                y4m_path,
                f"https://127.0.0.1:{ports.web}/",
                f"rtsp://127.0.0.1:{ports.rtsp}/phone",
            )
            print(
                f"[5/5] [PASS] browser WHIP -> MediaMTX -> H.264 RTSP "
                f"decoded {args.decode_frames} frames at "
                f"{decoded_width}x{decoded_height} (16:9); "
                "board source performs final 1280x720 normalization"
            )
        except Exception:
            print("--- MediaMTX log tail ---", file=sys.stderr)
            print(mediamtx.tail(), file=sys.stderr)
            print("--- web server log tail ---", file=sys.stderr)
            print(web.tail(), file=sys.stderr)
            raise
        finally:
            web.stop()
            mediamtx.stop()
            control.shutdown()
            control.server_close()
            control_thread.join(timeout=2)
            control_socket.unlink(missing_ok=True)


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Publish GStreamer videotestsrc through Chromium WHIP/WebRTC, "
            "then decode and verify MediaMTX RTSP output"
        )
    )
    parser.add_argument(
        "--mediamtx-command",
        default=os.environ.get(
            "MEDIAMTX_COMMAND",
            str(ROOT / "vendor" / "mediamtx-v1.19.2" / "bin" / "mediamtx"),
        ),
        help="command prefix for exact MediaMTX v1.19.2; wrappers are allowed",
    )
    parser.add_argument(
        "--gst-launch-command",
        default=os.environ.get("GST_LAUNCH_COMMAND", "gst-launch-1.0"),
        help="command prefix for gst-launch-1.0; wrappers and env are allowed",
    )
    parser.add_argument(
        "--ffmpeg-command",
        default=os.environ.get("FFMPEG_COMMAND", "ffmpeg"),
        help="command prefix for ffmpeg",
    )
    parser.add_argument(
        "--openssl-command",
        default=os.environ.get("OPENSSL_COMMAND", "openssl"),
        help="command prefix for openssl",
    )
    parser.add_argument(
        "--server-command",
        default=os.environ.get("WEB_SERVER_COMMAND", sys.executable),
        help="command prefix for Python server.py",
    )
    parser.add_argument(
        "--chromium-executable",
        default=os.environ.get("CHROMIUM_EXECUTABLE", "/usr/bin/chromium"),
    )
    parser.add_argument("--web-port", type=int, default=8443)
    parser.add_argument("--whip-port", type=int, default=8889)
    parser.add_argument("--rtsp-port", type=int, default=8554)
    parser.add_argument("--webrtc-udp-port", type=int, default=8189)
    parser.add_argument("--source-frames", type=int, default=60)
    parser.add_argument("--decode-frames", type=int, default=30)
    parser.add_argument("--startup-timeout", type=float, default=15.0)
    parser.add_argument("--browser-timeout", type=float, default=20.0)
    parser.add_argument("--decode-timeout", type=float, default=20.0)
    parser.add_argument("--tool-timeout", type=float, default=60.0)
    parser.add_argument("--work-root")
    parser.add_argument("--headful", action="store_true")
    args = parser.parse_args(argv)

    args.mediamtx_command = parse_command(
        args.mediamtx_command,
        "MediaMTX",
    )
    args.gst_launch_command = parse_command(
        args.gst_launch_command,
        "GStreamer",
    )
    args.ffmpeg_command = parse_command(args.ffmpeg_command, "ffmpeg")
    args.openssl_command = parse_command(args.openssl_command, "OpenSSL")
    args.server_command = parse_command(args.server_command, "web server")
    if len({args.web_port, args.whip_port, args.rtsp_port}) != 3:
        parser.error("web, WHIP, and RTSP TCP ports must be distinct")
    for port in (
        args.web_port,
        args.whip_port,
        args.rtsp_port,
        args.webrtc_udp_port,
    ):
        if port < 1 or port > 65535:
            parser.error("ports must be between 1 and 65535")
    if args.source_frames < 30 or args.decode_frames < 1:
        parser.error("--source-frames must be >=30 and --decode-frames must be positive")
    for timeout in (
        args.startup_timeout,
        args.browser_timeout,
        args.decode_timeout,
        args.tool_timeout,
    ):
        if timeout <= 0:
            parser.error("timeouts must be positive")
    chromium = Path(args.chromium_executable)
    if not chromium.is_file():
        parser.error(f"Chromium executable not found: {chromium}")
    return args


def main(argv: list[str] | None = None) -> int:
    try:
        run_test(parse_args(argv))
    except TestFailure as exc:
        print(f"[FAIL] {exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("[FAIL] interrupted", file=sys.stderr)
        return 130
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
