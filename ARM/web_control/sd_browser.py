#!/usr/bin/env python3
"""SD card file browser for the board web control.

Lists photo and video files under a configurable SD card root. Every
client-supplied sub-path is confined to that root to prevent directory
traversal, so the browser surface is safe to expose over HTTPS.
"""

from __future__ import annotations

import dataclasses
from pathlib import Path
from typing import Any


PHOTO_EXTENSIONS = {".jpg", ".jpeg", ".png"}
VIDEO_EXTENSIONS = {
    ".mp4",
    ".mov",
    ".avi",
    ".mkv",
    ".m4v",
    ".h264",
    ".264",
    ".ts",
    ".webm",
}

DEFAULT_MAX_ENTRIES = 2000


class SdBrowserError(RuntimeError):
    pass


class SdPathOutsideRoot(SdBrowserError):
    pass


@dataclasses.dataclass(frozen=True)
class SdBrowseConfig:
    sd_root: Path = Path("/mnt/sdcard")
    max_entries: int = DEFAULT_MAX_ENTRIES


def file_kind(path: Path) -> str:
    suffix = path.suffix.lower()
    if suffix in PHOTO_EXTENSIONS:
        return "photo"
    if suffix in VIDEO_EXTENSIONS:
        return "video"
    return "other"


def _relative_subpath(root_resolved: Path, path: Path) -> str:
    if path == root_resolved:
        return ""
    try:
        return path.relative_to(root_resolved).as_posix()
    except ValueError as exc:
        raise SdPathOutsideRoot("路径超出 SD 卡根目录") from exc


def _resolve_safe(root_resolved: Path, subpath: str) -> Path:
    cleaned = (subpath or "").strip().lstrip("/\\")
    candidate = root_resolved / cleaned if cleaned else root_resolved
    try:
        resolved = candidate.resolve(strict=False)
    except OSError as exc:
        raise SdBrowserError(f"无法解析路径: {subpath}") from exc
    try:
        resolved.relative_to(root_resolved)
    except ValueError as exc:
        raise SdPathOutsideRoot("路径超出 SD 卡根目录") from exc
    return resolved


def _sort_key(path: Path) -> tuple[int, str]:
    try:
        is_dir = path.is_dir()
    except OSError:
        is_dir = False
    return (0 if is_dir else 1, path.name.lower())


def _root_resolved(sd_root: Path) -> Path:
    try:
        return sd_root.resolve(strict=False)
    except OSError as exc:
        raise SdBrowserError(f"无法解析 SD 卡根目录: {sd_root}") from exc


def list_files(
    sd_root: Path,
    subpath: str = "",
    max_entries: int = DEFAULT_MAX_ENTRIES,
) -> dict[str, Any]:
    root_resolved = _root_resolved(sd_root)
    if not root_resolved.exists():
        raise SdBrowserError(f"SD 卡未挂载或目录不存在: {sd_root}")
    if not root_resolved.is_dir():
        raise SdBrowserError(f"SD 卡根不是目录: {sd_root}")
    target = _resolve_safe(root_resolved, subpath)
    if not target.exists():
        raise SdBrowserError(f"路径不存在: {subpath}")
    if not target.is_dir():
        raise SdBrowserError(f"路径不是目录: {subpath}")

    entries: list[dict[str, Any]] = []
    if target != root_resolved:
        entries.append(
            {
                "name": "..",
                "path": _relative_subpath(root_resolved, target.parent),
                "type": "parent",
                "size": 0,
                "mtime": 0,
            }
        )

    count = 0
    truncated = False
    for child in sorted(target.iterdir(), key=_sort_key):
        if child.name.startswith("."):
            continue
        try:
            stat = child.stat()
        except OSError:
            continue
        is_dir = child.is_dir()
        kind = "dir" if is_dir else file_kind(child)
        entries.append(
            {
                "name": child.name,
                "path": _relative_subpath(root_resolved, child),
                "type": kind,
                "size": stat.st_size if not is_dir else 0,
                "mtime": int(stat.st_mtime),
            }
        )
        count += 1
        if count >= max_entries:
            truncated = True
            break

    return {
        "ok": True,
        "root": str(sd_root),
        "path": _relative_subpath(root_resolved, target),
        "truncated": truncated,
        "entries": entries,
    }


def resolve_file(sd_root: Path, subpath: str) -> Path:
    """Resolve a client-supplied subpath to an absolute file path under sd_root."""
    root_resolved = _root_resolved(sd_root)
    if not root_resolved.exists():
        raise SdBrowserError(f"SD 卡未挂载或目录不存在: {sd_root}")
    target = _resolve_safe(root_resolved, subpath)
    if not target.is_file():
        raise SdBrowserError(f"不是文件: {subpath}")
    return target
