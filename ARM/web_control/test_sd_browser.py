#!/usr/bin/env python3
"""Tests for the SD card file browser."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

from sd_browser import (
    SdBrowserError,
    SdPathOutsideRoot,
    file_kind,
    list_files,
    resolve_file,
)


class SdBrowserTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory(prefix="sd-browser-test-")
        self.root = Path(self.tmp.name)
        (self.root / "photo.jpg").write_bytes(b"jpg")
        (self.root / "clip.MP4").write_bytes(b"mp4")
        (self.root / "notes.txt").write_bytes(b"txt")
        (self.root / ".hidden").write_bytes(b"h")
        sub = self.root / "sub"
        sub.mkdir()
        (sub / "a.png").write_bytes(b"png")
        (sub / "b.mov").write_bytes(b"mov")

    def tearDown(self) -> None:
        self.tmp.cleanup()

    def test_file_kind(self) -> None:
        self.assertEqual(file_kind(Path("a.jpg")), "photo")
        self.assertEqual(file_kind(Path("a.JPEG")), "photo")
        self.assertEqual(file_kind(Path("a.png")), "photo")
        self.assertEqual(file_kind(Path("a.mp4")), "video")
        self.assertEqual(file_kind(Path("a.mov")), "video")
        self.assertEqual(file_kind(Path("a.txt")), "other")

    def test_list_root_skips_hidden_and_classifies(self) -> None:
        listing = list_files(self.root, "")
        names = {entry["name"]: entry["type"] for entry in listing["entries"]}
        self.assertEqual(names["photo.jpg"], "photo")
        self.assertEqual(names["clip.MP4"], "video")
        self.assertEqual(names["sub"], "dir")
        self.assertNotIn(".hidden", names)

    def test_list_subdir_has_parent_entry(self) -> None:
        listing = list_files(self.root, "sub")
        entries = listing["entries"]
        self.assertEqual(entries[0]["type"], "parent")
        names = {entry["name"]: entry["type"] for entry in entries}
        self.assertEqual(names["a.png"], "photo")
        self.assertEqual(names["b.mov"], "video")

    def test_traversal_blocked(self) -> None:
        for bad in ("../etc", "sub/../.."):
            with self.assertRaises(SdPathOutsideRoot, msg=bad):
                list_files(self.root, bad)

    def test_nonexistent_path_is_browser_error_not_traversal(self) -> None:
        # /etc/passwd is treated as root/etc/passwd, which does not exist.
        with self.assertRaises(SdBrowserError):
            list_files(self.root, "etc/passwd")

    def test_resolve_file_ok(self) -> None:
        resolved = resolve_file(self.root, "sub/a.png")
        self.assertTrue(resolved.is_file())
        self.assertEqual(resolved.name, "a.png")

    def test_resolve_file_rejects_directory(self) -> None:
        with self.assertRaises(SdBrowserError):
            resolve_file(self.root, "sub")

    def test_resolve_file_blocks_traversal(self) -> None:
        with self.assertRaises(SdPathOutsideRoot):
            resolve_file(self.root, "../etc/passwd")

    def test_truncation(self) -> None:
        for i in range(5):
            (self.root / f"f{i}.jpg").write_bytes(b"x")
        listing = list_files(self.root, "", max_entries=3)
        self.assertTrue(listing["truncated"])
        self.assertEqual(len(listing["entries"]), 3)

    def test_missing_root(self) -> None:
        with self.assertRaises(SdBrowserError):
            list_files(Path("/nonexistent-sd-root-xyz"), "")


if __name__ == "__main__":
    unittest.main(verbosity=2)
