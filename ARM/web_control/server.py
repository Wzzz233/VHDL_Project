#!/usr/bin/env python3
"""HTTPS control panel and protocol proxy for pplcnet_bgp_live.

The live process remains the owner of capture, inference and preview encoding.
This server serves the browser application, forwards JSON RPC requests over a
Unix socket, and keeps MediaMTX WHIP signaling on the same HTTPS origin.
"""

from __future__ import annotations

import argparse
import dataclasses
import http.client
import json
import logging
import socket
import ssl
import threading
import time
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Mapping
from urllib.parse import urlsplit, urlunsplit

from driver_manager import DriverManager, DriverManagerError
from image_inference import (
    ImageInferenceBusy,
    ImageInferenceConfig,
    ImageInferenceError,
    ImageInferenceRunner,
)


ROOT = Path(__file__).resolve().parent
STATIC_ROOT = ROOT / "static"
DEFAULT_CONTROL_SOCKET = Path("/run/pplcnet-bgp-live/control.sock")
PUBLIC_WHIP_PATH = "/whip/phone"
MAX_JSON_BODY = 64 * 1024
MAX_WHIP_BODY = 1024 * 1024
MAX_RPC_HEADER = 1024 * 1024
MAX_JPEG_SIZE = 4 * 1024 * 1024
MAX_IMAGE_BODY = 15 * 1024 * 1024

LOG = logging.getLogger("bgp-web-control")


class ControlUnavailable(RuntimeError):
    """The live process Unix socket cannot be reached."""


class ControlProtocolError(RuntimeError):
    """The live process returned a malformed response."""


@dataclasses.dataclass(frozen=True)
class AppConfig:
    control_socket: Path = DEFAULT_CONTROL_SOCKET
    control_timeout: float = 2.0
    mediamtx_host: str = "127.0.0.1"
    mediamtx_port: int = 8889
    mediamtx_whip_path: str = "/phone/whip"
    mediamtx_timeout: float = 5.0
    frame_max_fps: float = 5.0
    static_root: Path = STATIC_ROOT
    image_runner: Any | None = None
    driver_manager: Any | None = None


class ControlClient:
    """One-request-per-connection client for the live process JSON RPC."""

    def __init__(self, path: Path, timeout: float) -> None:
        self.path = path
        self.timeout = timeout

    @staticmethod
    def _encode_request(payload: Mapping[str, Any]) -> bytes:
        encoded = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        return encoded + b"\n"

    @staticmethod
    def _read_header(stream: Any) -> dict[str, Any]:
        line = stream.readline(MAX_RPC_HEADER + 1)
        if not line:
            raise ControlProtocolError("empty response from live process")
        if len(line) > MAX_RPC_HEADER or not line.endswith(b"\n"):
            raise ControlProtocolError("live process response header is too large")
        try:
            value = json.loads(line.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ControlProtocolError("live process returned invalid JSON") from exc
        if not isinstance(value, dict):
            raise ControlProtocolError("live process JSON response must be an object")
        return value

    @staticmethod
    def _read_exact(stream: Any, length: int) -> bytes:
        chunks: list[bytes] = []
        remaining = length
        while remaining:
            chunk = stream.read(remaining)
            if not chunk:
                raise ControlProtocolError("truncated JPEG response from live process")
            chunks.append(chunk)
            remaining -= len(chunk)
        return b"".join(chunks)

    def _connect(self) -> socket.socket:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.settimeout(self.timeout)
        try:
            sock.connect(str(self.path))
        except (FileNotFoundError, ConnectionRefusedError, socket.timeout, OSError) as exc:
            sock.close()
            raise ControlUnavailable(f"live process socket unavailable: {self.path}") from exc
        return sock

    def request_json(self, payload: Mapping[str, Any]) -> dict[str, Any]:
        try:
            with self._connect() as sock:
                sock.sendall(self._encode_request(payload))
                with sock.makefile("rb") as stream:
                    return self._read_header(stream)
        except ControlProtocolError:
            raise
        except (socket.timeout, OSError) as exc:
            raise ControlUnavailable("live process did not complete the JSON response") from exc

    def request_frame(self) -> tuple[str, bytes, int]:
        try:
            with self._connect() as sock:
                sock.sendall(self._encode_request({"op": "frame"}))
                with sock.makefile("rb") as stream:
                    header = self._read_header(stream)
                    if header.get("ok") is False:
                        message = str(header.get("error") or header.get("message") or "frame unavailable")
                        raise ControlUnavailable(message)
                    content_type = str(header.get("content_type", ""))
                    length = header.get("content_length")
                    generation = header.get("source_generation")
                    if content_type.split(";", 1)[0].strip().lower() != "image/jpeg":
                        raise ControlProtocolError("live process frame is not image/jpeg")
                    if not isinstance(length, int) or isinstance(length, bool) or length <= 0 or length > MAX_JPEG_SIZE:
                        raise ControlProtocolError("invalid JPEG content_length from live process")
                    if (
                        not isinstance(generation, int)
                        or isinstance(generation, bool)
                        or generation <= 0
                    ):
                        raise ControlProtocolError(
                            "invalid source_generation from live process"
                        )
                    return "image/jpeg", self._read_exact(stream, length), generation
        except (ControlUnavailable, ControlProtocolError):
            raise
        except (socket.timeout, OSError) as exc:
            raise ControlUnavailable("live process did not complete the JPEG response") from exc


class AppState:
    def __init__(self, config: AppConfig) -> None:
        self.config = config
        self.control = ControlClient(config.control_socket, config.control_timeout)
        self.image_runner = config.image_runner
        self.driver_manager = config.driver_manager
        self._frame_lock = threading.Lock()
        self._frame_timestamp = 0.0
        self._frame_content_type = "image/jpeg"
        self._frame_data = b""
        self._frame_generation: int | None = None
        self._status_generation: int | None = None

    def _clear_frame_locked(self) -> None:
        self._frame_timestamp = 0.0
        self._frame_content_type = "image/jpeg"
        self._frame_data = b""
        self._frame_generation = None

    def note_status_generation(self, response: Mapping[str, Any]) -> bool:
        status = response.get("status")
        generation = response.get("source_generation")
        if generation is None and isinstance(status, Mapping):
            generation = status.get("source_generation")
        if (
            not isinstance(generation, int)
            or isinstance(generation, bool)
            or generation <= 0
        ):
            return False
        with self._frame_lock:
            changed = (
                self._status_generation is not None
                and generation != self._status_generation
            )
            self._status_generation = generation
            if self._frame_generation != generation:
                self._clear_frame_locked()
            return changed

    def frame(self) -> tuple[str, bytes, int]:
        """Limit upstream preview reads to the configured maximum frame rate."""
        with self._frame_lock:
            now = time.monotonic()
            minimum_period = 1.0 / self.config.frame_max_fps
            if self._frame_data and now - self._frame_timestamp < minimum_period:
                assert self._frame_generation is not None
                return (
                    self._frame_content_type,
                    self._frame_data,
                    self._frame_generation,
                )
            content_type, data, generation = self.control.request_frame()
            if (
                self._status_generation is None
                or generation == self._status_generation
            ):
                self._frame_content_type = content_type
                self._frame_data = data
                self._frame_generation = generation
                self._frame_timestamp = time.monotonic()
            else:
                self._clear_frame_locked()
            return content_type, data, generation


STATIC_FILES = {
    "/": ("index.html", "text/html; charset=utf-8"),
    "/index.html": ("index.html", "text/html; charset=utf-8"),
    "/app.js": ("app.js", "application/javascript; charset=utf-8"),
    "/style.css": ("style.css", "text/css; charset=utf-8"),
}


class ControlHandler(BaseHTTPRequestHandler):
    state: AppState
    protocol_version = "HTTP/1.1"
    server_version = "BGPWebControl/1"

    def log_message(self, fmt: str, *args: object) -> None:
        LOG.info("%s - %s", self.address_string(), fmt % args)

    def _security_headers(self) -> None:
        self.send_header("X-Content-Type-Options", "nosniff")
        self.send_header("X-Frame-Options", "DENY")
        self.send_header("Referrer-Policy", "no-referrer")
        self.send_header("Permissions-Policy", "camera=(self), microphone=()")
        self.send_header(
            "Content-Security-Policy",
            "default-src 'self'; script-src 'self'; style-src 'self'; "
            "img-src 'self' blob:; media-src 'self' blob:; connect-src 'self'; "
            "object-src 'none'; base-uri 'none'; frame-ancestors 'none'",
        )
        self.send_header("Strict-Transport-Security", "max-age=31536000")

    def _send_bytes(
        self,
        status: int,
        data: bytes,
        content_type: str,
        headers: list[tuple[str, str]] | None = None,
    ) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Cache-Control", "no-store")
        if headers:
            for name, value in headers:
                self.send_header(name, value)
        self._security_headers()
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        if self.command != "HEAD" and data:
            self.wfile.write(data)

    def _send_json(self, status: int, value: Any) -> None:
        data = json.dumps(value, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
        self._send_bytes(status, data, "application/json; charset=utf-8")

    def _send_problem(self, status: int, code: str, message: str) -> None:
        self._send_json(status, {"ok": False, "error": {"code": code, "message": message}})

    def _read_body(self, limit: int) -> bytes:
        raw_length = self.headers.get("Content-Length")
        if raw_length is None:
            raise ValueError("Content-Length is required")
        try:
            length = int(raw_length)
        except ValueError as exc:
            raise ValueError("invalid Content-Length") from exc
        if length < 0 or length > limit:
            raise OverflowError("request body is too large")
        data = self.rfile.read(length)
        if len(data) != length:
            raise ValueError("request body was truncated")
        return data

    def _read_json_object(self) -> dict[str, Any]:
        media_type = self.headers.get("Content-Type", "").split(";", 1)[0].strip().lower()
        if media_type != "application/json":
            raise TypeError("Content-Type must be application/json")
        data = self._read_body(MAX_JSON_BODY)
        try:
            value = json.loads(data.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise ValueError("request body must be valid JSON") from exc
        if not isinstance(value, dict):
            raise ValueError("JSON body must be an object")
        return value

    def _proxy_control_json(self, payload: Mapping[str, Any]) -> None:
        try:
            response = self.state.control.request_json(payload)
        except ControlUnavailable as exc:
            self._send_problem(HTTPStatus.SERVICE_UNAVAILABLE, "control_unavailable", str(exc))
            return
        except ControlProtocolError as exc:
            self._send_problem(HTTPStatus.BAD_GATEWAY, "control_protocol", str(exc))
            return
        if payload.get("op") == "status" and response.get("ok") is not False:
            self.state.note_status_generation(response)
        status = HTTPStatus.OK if response.get("ok") is not False else HTTPStatus.CONFLICT
        self._send_json(status, response)

    def _serve_static(self, path: str) -> bool:
        entry = STATIC_FILES.get(path)
        if entry is None:
            return False
        filename, content_type = entry
        file_path = self.state.config.static_root / filename
        try:
            data = file_path.read_bytes()
        except FileNotFoundError:
            self._send_problem(HTTPStatus.NOT_FOUND, "not_found", "static file not found")
            return True
        self._send_bytes(HTTPStatus.OK, data, content_type)
        return True

    @staticmethod
    def _is_whip_session_path(path: str) -> bool:
        return path.startswith(PUBLIC_WHIP_PATH + "/") and len(path) > len(PUBLIC_WHIP_PATH) + 1

    def _require_same_origin(self) -> bool:
        origin = self.headers.get("Origin")
        if not origin:
            return True
        parts = urlsplit(origin)
        expected_scheme = "https" if isinstance(self.connection, ssl.SSLSocket) else "http"
        expected_host = self.headers.get("Host", "").lower()
        if parts.scheme != expected_scheme or parts.netloc.lower() != expected_host:
            self._send_problem(HTTPStatus.FORBIDDEN, "cross_origin", "state-changing requests must be same-origin")
            return False
        return True

    def _whip_upstream_path(self, request_path: str, query: str) -> str:
        suffix = request_path[len(PUBLIC_WHIP_PATH) :]
        upstream = self.state.config.mediamtx_whip_path.rstrip("/") + suffix
        return upstream + (("?" + query) if query else "")

    def _rewrite_whip_location(self, location: str) -> str:
        parts = urlsplit(location)
        upstream_base = self.state.config.mediamtx_whip_path.rstrip("/")
        if parts.scheme or parts.netloc or location.startswith("/"):
            upstream_path = parts.path
        else:
            upstream_path = upstream_base + "/" + parts.path.lstrip("./")

        if upstream_path == upstream_base:
            public_path = PUBLIC_WHIP_PATH
        elif upstream_path.startswith(upstream_base + "/"):
            public_path = PUBLIC_WHIP_PATH + upstream_path[len(upstream_base) :]
        else:
            public_path = PUBLIC_WHIP_PATH + "/" + upstream_path.rsplit("/", 1)[-1]
        return urlunsplit(("", "", public_path, parts.query, parts.fragment))

    def _proxy_whip(self) -> None:
        parsed = urlsplit(self.path)
        expected_media_type = {
            "POST": "application/sdp",
            "PATCH": "application/trickle-ice-sdpfrag",
        }.get(self.command)
        if expected_media_type:
            media_type = self.headers.get("Content-Type", "").split(";", 1)[0].strip().lower()
            if media_type != expected_media_type:
                self._send_problem(
                    HTTPStatus.UNSUPPORTED_MEDIA_TYPE,
                    "content_type",
                    f"Content-Type must be {expected_media_type}",
                )
                return
        if self.command == "DELETE" and self.headers.get("Content-Length") is None:
            body = b""
        else:
            try:
                body = self._read_body(MAX_WHIP_BODY)
            except ValueError as exc:
                self._send_problem(HTTPStatus.LENGTH_REQUIRED, "invalid_length", str(exc))
                return
            except OverflowError as exc:
                self._send_problem(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, "body_too_large", str(exc))
                return
        if self.command == "DELETE" and body:
            self._send_problem(HTTPStatus.BAD_REQUEST, "unexpected_body", "WHIP DELETE takes no body")
            return

        upstream_headers: dict[str, str] = {}
        for name in ("Content-Type", "Accept", "If-Match"):
            if self.headers.get(name):
                upstream_headers[name] = self.headers[name]
        upstream_headers["Content-Length"] = str(len(body))

        conn = http.client.HTTPConnection(
            self.state.config.mediamtx_host,
            self.state.config.mediamtx_port,
            timeout=self.state.config.mediamtx_timeout,
        )
        try:
            conn.request(
                self.command,
                self._whip_upstream_path(parsed.path, parsed.query),
                body=body,
                headers=upstream_headers,
            )
            response = conn.getresponse()
            response_body = response.read(MAX_WHIP_BODY + 1)
            if len(response_body) > MAX_WHIP_BODY:
                raise http.client.HTTPException("MediaMTX WHIP response is too large")
            response_headers: list[tuple[str, str]] = []
            for name, value in response.getheaders():
                lower = name.lower()
                if lower == "location":
                    response_headers.append(("Location", self._rewrite_whip_location(value)))
                elif lower in ("etag", "link", "accept-patch", "accept-post"):
                    response_headers.append((name, value))
            content_type = response.getheader("Content-Type", "application/octet-stream")
            self._send_bytes(response.status, response_body, content_type, response_headers)
        except (ConnectionError, TimeoutError, socket.timeout, OSError, http.client.HTTPException) as exc:
            self._send_problem(HTTPStatus.BAD_GATEWAY, "whip_unavailable", "MediaMTX WHIP signaling is unavailable")
            LOG.warning("MediaMTX WHIP proxy failed: %s", exc)
        finally:
            conn.close()

    def do_HEAD(self) -> None:
        parsed = urlsplit(self.path)
        if not self._serve_static(parsed.path):
            self._send_problem(HTTPStatus.NOT_FOUND, "not_found", "resource not found")

    def do_GET(self) -> None:
        parsed = urlsplit(self.path)
        if self._serve_static(parsed.path):
            return
        if parsed.path == "/api/v1/status":
            self._proxy_control_json({"op": "status"})
        elif parsed.path == "/api/v1/results":
            self._proxy_control_json({"op": "results"})
        elif parsed.path == "/api/v1/frame.jpg":
            try:
                content_type, data, generation = self.state.frame()
            except ControlUnavailable as exc:
                self._send_problem(HTTPStatus.SERVICE_UNAVAILABLE, "frame_unavailable", str(exc))
            except ControlProtocolError as exc:
                self._send_problem(HTTPStatus.BAD_GATEWAY, "control_protocol", str(exc))
            else:
                self._send_bytes(
                    HTTPStatus.OK,
                    data,
                    content_type,
                    [("X-Source-Generation", str(generation))],
                )
        elif parsed.path == "/api/v1/mode":
            if self.state.driver_manager is None:
                self._send_problem(HTTPStatus.SERVICE_UNAVAILABLE, "driver_manager_unavailable", "driver manager is not configured")
                return
            try:
                self._send_json(HTTPStatus.OK, self.state.driver_manager.status())
            except DriverManagerError as exc:
                self._send_problem(HTTPStatus.CONFLICT, "driver_mode_failed", str(exc))
        else:
            self._send_problem(HTTPStatus.NOT_FOUND, "not_found", "resource not found")

    def do_PUT(self) -> None:
        parsed = urlsplit(self.path)
        if parsed.path == "/api/v1/mode":
            if not self._require_same_origin():
                return
            try:
                value = self._read_json_object()
            except TypeError as exc:
                self._send_problem(HTTPStatus.UNSUPPORTED_MEDIA_TYPE, "content_type", str(exc))
                return
            except OverflowError as exc:
                self._send_problem(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, "body_too_large", str(exc))
                return
            except ValueError as exc:
                self._send_problem(HTTPStatus.BAD_REQUEST, "invalid_json", str(exc))
                return
            if set(value) != {"mode"} or value.get("mode") not in ("plate", "pedestrian"):
                self._send_problem(HTTPStatus.BAD_REQUEST, "invalid_mode", "mode must be plate or pedestrian")
                return
            if self.state.driver_manager is None:
                self._send_problem(HTTPStatus.SERVICE_UNAVAILABLE, "driver_manager_unavailable", "driver manager is not configured")
                return
            try:
                response = self.state.driver_manager.set_mode(value["mode"])
            except DriverManagerError as exc:
                self._send_problem(HTTPStatus.CONFLICT, "driver_mode_failed", str(exc))
            else:
                self._send_json(HTTPStatus.OK, response)
            return
        if parsed.path != "/api/v1/source":
            self._send_problem(HTTPStatus.NOT_FOUND, "not_found", "resource not found")
            return
        if not self._require_same_origin():
            return
        try:
            value = self._read_json_object()
        except TypeError as exc:
            self._send_problem(HTTPStatus.UNSUPPORTED_MEDIA_TYPE, "content_type", str(exc))
            return
        except OverflowError as exc:
            self._send_problem(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, "body_too_large", str(exc))
            return
        except ValueError as exc:
            self._send_problem(HTTPStatus.BAD_REQUEST, "invalid_json", str(exc))
            return
        if set(value) != {"source"}:
            self._send_problem(HTTPStatus.BAD_REQUEST, "invalid_source", "JSON body must contain only source")
            return
        source = value["source"]
        if source not in ("fpga", "phone"):
            self._send_problem(HTTPStatus.BAD_REQUEST, "invalid_source", "source must be fpga or phone")
            return
        self._proxy_control_json({"op": "source", "source": source})

    def do_POST(self) -> None:
        parsed = urlsplit(self.path)
        pipeline_paths = {
            "/api/v1/pipeline/pause",
            "/api/v1/pipeline/resume",
            "/api/v1/pipeline/restart",
        }
        if parsed.path == "/api/v1/image-inference":
            if not self._require_same_origin():
                return
            mode = self.headers.get("X-Inference-Mode", "").strip().lower()
            media_type = self.headers.get("Content-Type", "").split(";", 1)[0].strip().lower()
            if mode not in ("plate", "pedestrian"):
                self._send_problem(HTTPStatus.BAD_REQUEST, "invalid_mode", "mode must be plate or pedestrian")
                return
            if media_type not in ("image/jpeg", "image/png"):
                self._send_problem(HTTPStatus.UNSUPPORTED_MEDIA_TYPE, "content_type", "only JPEG and PNG images are supported")
                return
            try:
                image = self._read_body(MAX_IMAGE_BODY)
            except ValueError as exc:
                self._send_problem(HTTPStatus.LENGTH_REQUIRED, "invalid_length", str(exc))
                return
            except OverflowError as exc:
                self._send_problem(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, "body_too_large", str(exc))
                return
            if not image:
                self._send_problem(HTTPStatus.BAD_REQUEST, "empty_image", "image is empty")
                return
            if self.state.image_runner is None:
                self._send_problem(HTTPStatus.SERVICE_UNAVAILABLE, "image_inference_unavailable", "image inference is not configured")
                return
            try:
                if self.state.driver_manager is None:
                    raise DriverManagerError("driver manager is not configured")
                with self.state.driver_manager.image_session(mode):
                    result = self.state.image_runner.run(mode, image, media_type)
            except ImageInferenceBusy as exc:
                self._send_problem(HTTPStatus.CONFLICT, "image_inference_busy", str(exc))
            except DriverManagerError as exc:
                self._send_problem(HTTPStatus.CONFLICT, "driver_mode_failed", str(exc))
            except ImageInferenceError as exc:
                LOG.warning("image inference failed: %s", exc)
                self._send_problem(HTTPStatus.BAD_GATEWAY, "image_inference_failed", str(exc))
            except Exception:
                LOG.exception("unexpected image inference failure")
                self._send_problem(HTTPStatus.INTERNAL_SERVER_ERROR, "image_inference_failed", "unexpected image inference failure")
            else:
                self._send_json(HTTPStatus.OK, result)
        elif parsed.path in pipeline_paths:
            if not self._require_same_origin():
                return
            action = parsed.path.rsplit("/", 1)[-1]
            try:
                value = self._read_json_object()
            except TypeError as exc:
                self._send_problem(HTTPStatus.UNSUPPORTED_MEDIA_TYPE, "content_type", str(exc))
                return
            except OverflowError as exc:
                self._send_problem(HTTPStatus.REQUEST_ENTITY_TOO_LARGE, "body_too_large", str(exc))
                return
            except ValueError as exc:
                self._send_problem(HTTPStatus.BAD_REQUEST, "invalid_json", str(exc))
                return
            if value:
                self._send_problem(HTTPStatus.BAD_REQUEST, "unexpected_body", "pipeline action takes no request fields")
                return
            self._proxy_control_json({"op": "pipeline", "action": action})
        elif parsed.path == PUBLIC_WHIP_PATH:
            if not self._require_same_origin():
                return
            self._proxy_whip()
        else:
            self._send_problem(HTTPStatus.NOT_FOUND, "not_found", "resource not found")

    def do_PATCH(self) -> None:
        parsed = urlsplit(self.path)
        if self._is_whip_session_path(parsed.path):
            if not self._require_same_origin():
                return
            self._proxy_whip()
        else:
            self._send_problem(HTTPStatus.NOT_FOUND, "not_found", "resource not found")

    def do_DELETE(self) -> None:
        parsed = urlsplit(self.path)
        if self._is_whip_session_path(parsed.path):
            if not self._require_same_origin():
                return
            self._proxy_whip()
        else:
            self._send_problem(HTTPStatus.NOT_FOUND, "not_found", "resource not found")

    def do_OPTIONS(self) -> None:
        parsed = urlsplit(self.path)
        if parsed.path == PUBLIC_WHIP_PATH:
            self._send_bytes(
                HTTPStatus.NO_CONTENT,
                b"",
                "text/plain; charset=utf-8",
                [
                    ("Allow", "OPTIONS, POST, PATCH, DELETE"),
                    ("Accept-Post", "application/sdp"),
                    ("Accept-Patch", "application/trickle-ice-sdpfrag"),
                ],
            )
        else:
            self._send_problem(HTTPStatus.NOT_FOUND, "not_found", "resource not found")


class ControlHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True
    request_queue_size = 32


class TLSControlHTTPServer(ControlHTTPServer):
    """Perform TLS handshakes inside worker threads with bounded reads."""

    def __init__(
        self,
        address: tuple[str, int],
        handler: type[ControlHandler],
        context: ssl.SSLContext,
        handshake_timeout: float,
        client_timeout: float,
    ) -> None:
        self.tls_context = context
        self.handshake_timeout = handshake_timeout
        self.client_timeout = client_timeout
        super().__init__(address, handler)

    def process_request_thread(self, request: socket.socket, client_address: tuple[Any, ...]) -> None:
        connection: socket.socket | ssl.SSLSocket = request
        try:
            request.settimeout(self.handshake_timeout)
            connection = self.tls_context.wrap_socket(request, server_side=True)
            connection.settimeout(self.client_timeout)
            self.finish_request(connection, client_address)
        except (socket.timeout, ssl.SSLError):
            pass
        except Exception:
            self.handle_error(connection, client_address)
        finally:
            self.shutdown_request(connection)


def make_handler(state: AppState) -> type[ControlHandler]:
    class BoundControlHandler(ControlHandler):
        pass

    BoundControlHandler.state = state
    return BoundControlHandler


def create_server(address: tuple[str, int], state: AppState) -> ControlHTTPServer:
    return ControlHTTPServer(address, make_handler(state))


def create_tls_server(
    address: tuple[str, int],
    state: AppState,
    context: ssl.SSLContext,
    handshake_timeout: float,
    client_timeout: float,
) -> TLSControlHTTPServer:
    return TLSControlHTTPServer(
        address,
        make_handler(state),
        context,
        handshake_timeout,
        client_timeout,
    )


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="HTTPS control panel for pplcnet_bgp_live")
    parser.add_argument("--host", required=True, help="fixed LAN address to bind; wildcard addresses are rejected")
    parser.add_argument("--port", type=int, default=8443)
    parser.add_argument("--cert-file", type=Path, default=ROOT / "tls" / "generated" / "server.crt")
    parser.add_argument("--key-file", type=Path, default=ROOT / "tls" / "generated" / "server.key")
    parser.add_argument("--control-socket", type=Path, default=DEFAULT_CONTROL_SOCKET)
    parser.add_argument("--control-timeout", type=float, default=2.0)
    parser.add_argument("--mediamtx-host", default="127.0.0.1")
    parser.add_argument("--mediamtx-port", type=int, default=8889)
    parser.add_argument("--mediamtx-whip-path", default="/phone/whip")
    parser.add_argument("--mediamtx-timeout", type=float, default=5.0)
    parser.add_argument("--frame-max-fps", type=float, default=5.0)
    parser.add_argument("--arm-root", type=Path, default=Path("/home/linaro/ARM"))
    parser.add_argument("--model-root", type=Path, default=Path("/userdata/model"))
    parser.add_argument("--plate-image-driver", type=Path, default=Path("/home/linaro/ARM/pplcnet_bgp_live"))
    parser.add_argument("--pedestrian-image-driver", type=Path, default=Path("/home/linaro/ARM/cplus-rk3568-driver"))
    parser.add_argument("--image-inference-timeout", type=float, default=45.0)
    parser.add_argument("--driver-runtime-dir", type=Path, default=Path("/run/pplcnet-board"))
    parser.add_argument("--driver-log-dir", type=Path, default=Path("/var/log/pplcnet-board"))
    parser.add_argument("--plate-live-script", type=Path, default=ROOT / "run_plate_live.sh")
    parser.add_argument("--tls-handshake-timeout", type=float, default=5.0)
    parser.add_argument("--client-timeout", type=float, default=10.0)
    parser.add_argument("--log-level", choices=("DEBUG", "INFO", "WARNING", "ERROR"), default="INFO")
    args = parser.parse_args(argv)
    if args.host in ("", "0.0.0.0", "::", "[::]"):
        parser.error("--host must be a specific LAN address, not a wildcard")
    if not 1 <= args.port <= 65535 or not 1 <= args.mediamtx_port <= 65535:
        parser.error("ports must be between 1 and 65535")
    if (
        args.control_timeout <= 0
        or args.mediamtx_timeout <= 0
        or args.tls_handshake_timeout <= 0
        or args.client_timeout <= 0
        or args.image_inference_timeout <= 0
    ):
        parser.error("timeouts must be positive")
    if args.frame_max_fps <= 0 or args.frame_max_fps > 5:
        parser.error("--frame-max-fps must be greater than zero and no more than 5")
    if not args.mediamtx_whip_path.startswith("/"):
        parser.error("--mediamtx-whip-path must begin with /")
    return args


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    logging.basicConfig(level=getattr(logging, args.log_level), format="%(asctime)s %(levelname)s %(message)s")
    image_runner = ImageInferenceRunner(
        ImageInferenceConfig(
            arm_root=args.arm_root,
            model_root=args.model_root,
            plate_driver=args.plate_image_driver,
            pedestrian_driver=args.pedestrian_image_driver,
            timeout=args.image_inference_timeout,
        )
    )
    driver_manager = DriverManager(
        plate_script=args.plate_live_script,
        runtime_dir=args.driver_runtime_dir,
        log_dir=args.driver_log_dir,
        model_dir=args.model_root,
    )
    config = AppConfig(
        control_socket=args.control_socket,
        control_timeout=args.control_timeout,
        mediamtx_host=args.mediamtx_host,
        mediamtx_port=args.mediamtx_port,
        mediamtx_whip_path=args.mediamtx_whip_path,
        mediamtx_timeout=args.mediamtx_timeout,
        frame_max_fps=args.frame_max_fps,
        image_runner=image_runner,
        driver_manager=driver_manager,
    )
    state = AppState(config)

    context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
    context.minimum_version = ssl.TLSVersion.TLSv1_2
    try:
        context.load_cert_chain(args.cert_file, args.key_file)
    except OSError as exc:
        raise SystemExit(f"cannot load HTTPS certificate/key: {exc}") from exc
    server = create_tls_server(
        (args.host, args.port),
        state,
        context,
        args.tls_handshake_timeout,
        args.client_timeout,
    )

    LOG.info("web control listening on https://%s:%d", args.host, args.port)
    LOG.info("live process socket: %s", args.control_socket)
    try:
        server.serve_forever(poll_interval=0.25)
    except KeyboardInterrupt:
        LOG.info("shutdown requested")
    finally:
        server.server_close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
