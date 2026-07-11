#!/usr/bin/env python3
"""Focused hardware-free checks for the true WHIP ingress test harness."""

from __future__ import annotations

import tempfile
import unittest
from pathlib import Path

import test_whip_videotestsrc as whip_test


class WhipVideoTestHarnessTest(unittest.TestCase):
    def test_mediamtx_config_is_webrtc_only_ingress(self) -> None:
        config = whip_test.render_mediamtx_config(
            whip_test.Ports(web=9443, whip=9889, rtsp=9554, webrtc_udp=9189)
        )
        self.assertIn("rtmp: no", config)
        self.assertIn("hls: no", config)
        self.assertIn("srt: no", config)
        self.assertIn("moq: no", config)
        self.assertIn("webrtcAddress: 127.0.0.1:9889", config)
        self.assertIn("webrtcLocalUDPAddress: 127.0.0.1:9189", config)
        self.assertIn("rtspAddress: 127.0.0.1:9554", config)
        self.assertIn("phone:\n    source: publisher", config)

    def test_y4m_command_has_exact_camera_contract(self) -> None:
        with tempfile.TemporaryDirectory() as temp:
            output = Path(temp) / "camera.y4m"
            command = whip_test.build_gst_y4m_command(
                ["qemu-aarch64", "/sdk/gst-launch-1.0"],
                output,
                60,
            )
        self.assertEqual(command[:2], ["qemu-aarch64", "/sdk/gst-launch-1.0"])
        self.assertIn("num-buffers=60", command)
        self.assertIn(
            "video/x-raw,format=I420,width=1280,height=720,framerate=15/1",
            command,
        )
        self.assertIn("y4menc", command)
        self.assertIn(f"location={output}", command)

    def test_browser_uses_fake_y4m_camera(self) -> None:
        args = whip_test.build_chromium_args(Path("/tmp/camera.y4m"))
        self.assertIn("--use-fake-ui-for-media-stream", args)
        self.assertIn("--use-fake-device-for-media-stream", args)
        self.assertIn(
            "--use-file-for-fake-video-capture=/tmp/camera.y4m",
            args,
        )

    def test_ffmpeg_assertion_requires_h264_resolution_and_frames(self) -> None:
        stderr = (
            "Stream #0:0: Video: h264 (Constrained Baseline), "
            "yuv420p, 1280x720, 15 fps"
        )
        stdout = "frame=1\nprogress=continue\nframe=30\nprogress=end\n"
        whip_test.assert_ffmpeg_output(stdout, stderr)
        adapted = stderr.replace("1280x720", "480x270")
        self.assertEqual(whip_test.assert_ffmpeg_output(stdout, adapted), (480, 270))
        with self.assertRaisesRegex(whip_test.TestFailure, "not H.264"):
            whip_test.assert_ffmpeg_output(
                stdout,
                stderr.replace("h264", "vp8"),
            )
        with self.assertRaisesRegex(whip_test.TestFailure, "decoded 29"):
            whip_test.assert_ffmpeg_output(
                "frame=29\nprogress=end\n",
                stderr,
            )
        with self.assertRaisesRegex(whip_test.TestFailure, "not 16:9"):
            whip_test.assert_ffmpeg_output(
                stdout,
                stderr.replace("1280x720", "640x480"),
            )

    def test_ffmpeg_command_reads_fixed_phone_rtsp_path(self) -> None:
        command = whip_test.build_ffmpeg_command(
            ["ffmpeg"],
            "rtsp://127.0.0.1:8554/phone",
            30,
        )
        self.assertIn("rtsp://127.0.0.1:8554/phone", command)
        self.assertEqual(command[command.index("-frames:v") + 1], "30")
        self.assertEqual(command[command.index("-rtsp_transport") + 1], "tcp")
        self.assertEqual(command[command.index("-timeout") + 1], "3000000")


if __name__ == "__main__":
    unittest.main(verbosity=2)
