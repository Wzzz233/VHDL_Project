#!/usr/bin/env python3
"""Tests for the offline video inference runner.

These exercise the per-frame aggregation, frame sampling, batch inference,
truncation, and skip-on-error logic without touching gstreamer or the board
driver: _extract_frames is replaced with a stub that writes valid BGRx files,
and _run_pedestrian_batch is replaced with a stub that returns scripted results.
"""

from __future__ import annotations

import io
import tempfile
import types
import unittest
from pathlib import Path
from unittest import mock

from image_inference import (
    ImageInferenceConfig,
    ImageInferenceError,
    ImageInferenceRunner,
    VideoInferenceConfig,
    VideoInferenceRunner,
)

EXPECTED_BGRX_SIZE = 1280 * 720 * 4


class _VideoRunner(VideoInferenceRunner):
    def __init__(self, image_runner, config, frame_count):
        super().__init__(image_runner, config)
        self.frame_count = frame_count

    def _extract_frames(self, video_path, sample_fps, frames_dir, progress_callback):
        if progress_callback is not None:
            progress_callback(0, 0, "extracting")
        frames = []
        for index in range(self.frame_count):
            frame_path = frames_dir / f"frame{index:05d}.bgrx"
            frame_path.write_bytes(bytes([index % 256]) + b"\0" * (EXPECTED_BGRX_SIZE - 1))
            frames.append(frame_path)
        return frames, False


class VideoInferenceTest(unittest.TestCase):
    def setUp(self) -> None:
        self.tmp = tempfile.TemporaryDirectory(prefix="video-inference-test-")
        self.work = Path(self.tmp.name)
        self.video_path = self.work / "clip.mp4"
        self.video_path.write_bytes(b"fakevideo")
        self.image_runner = ImageInferenceRunner(ImageInferenceConfig())

    def tearDown(self) -> None:
        self.tmp.cleanup()

    def _wire_results(self, results_map, seen_source_indices=None) -> None:
        def run(self, frames_dir, output_dir, work):
            frame_paths = sorted(frames_dir.glob("frame*.bgrx"))
            if seen_source_indices is not None:
                for frame_path in frame_paths:
                    with frame_path.open("rb") as stream:
                        seen_source_indices.append(stream.read(1)[0])
            ordered = []
            for index, _frame_path in enumerate(frame_paths):
                item = results_map.get(index, [])
                if isinstance(item, Exception):
                    raise item
                ordered.append(
                    {"result": {"frame": index, "targets": item}, "mask_jpeg": b"\xff\xd8fake-jpeg"}
                )
            return ordered

        self.image_runner._run_pedestrian_batch = types.MethodType(run, self.image_runner)

    def _runner(self, frame_count, max_frames=600, max_keyframes=24) -> _VideoRunner:
        return _VideoRunner(
            self.image_runner,
            VideoInferenceConfig(max_frames=max_frames, max_keyframes=max_keyframes),
            frame_count,
        )

    def test_half_fps_samples_every_other_frame(self) -> None:
        seen_source_indices = []
        self._wire_results({0: [], 1: []}, seen_source_indices)
        runner = self._runner(frame_count=4)
        result = runner.run_video(self.video_path, sample_fps=0.5)
        # 4 frames at 1fps, step 2 -> sampled frames 0 and 2
        self.assertEqual(result["summary"]["total_frames"], 2)
        self.assertFalse(result["summary"]["truncated"])
        self.assertEqual(seen_source_indices, [0, 2])
        self.assertEqual(result["summary"]["full_analysis_frames"], 0)
        self.assertEqual(result["summary"]["fast_path_frames"], 2)

    def test_events_and_frames_recorded(self) -> None:
        targets = {
            0: [{"type": "pedestrian", "score": 0.9, "decision": "suspected_crossing_road_outside_zebra",
                 "reason": "road_dominant_without_zebra", "box": [1, 2, 3, 4],
                 "ground": {"road": 0.7, "sidewalk": 0.1, "zebra": 0.0}}],
            1: [{"type": "pedestrian", "score": 0.8, "decision": "not_suspected_crossing_road",
                 "reason": "sidewalk_suppressed", "box": [5, 6, 7, 8],
                 "ground": {}}],
        }
        self._wire_results(targets)
        runner = self._runner(frame_count=2)
        result = runner.run_video(self.video_path, sample_fps=1.0)
        self.assertEqual(result["summary"]["violation_count"], 1)
        self.assertEqual(result["summary"]["full_analysis_frames"], 2)
        self.assertEqual(result["summary"]["fast_path_frames"], 0)
        self.assertEqual(result["summary"]["violation_frames"], 1)
        # All frames are stored now (not just violation keyframes).
        self.assertEqual(len(result["frames"]), 2)
        self.assertTrue(result["frames"][0]["violation"])
        self.assertFalse(result["frames"][1]["violation"])
        self.assertEqual(result["events"][0]["frame_index"], 0)
        self.assertEqual(result["events"][0]["time_sec"], 0.0)

    def test_max_frames_truncates(self) -> None:
        self._wire_results({i: [] for i in range(5)})
        runner = self._runner(frame_count=5, max_frames=3)
        result = runner.run_video(self.video_path, sample_fps=1.0)
        self.assertEqual(result["summary"]["total_frames"], 3)
        self.assertTrue(result["summary"]["truncated"])

    def test_skipped_frame_when_batch_missing_output(self) -> None:
        # A batch entry with mask_jpeg=None counts as skipped.
        def run(self, frames_dir, output_dir, work):
            return [
                {"result": {"frame": 0, "targets": []}, "mask_jpeg": None},
                {"result": {"frame": 1, "targets": []}, "mask_jpeg": b"\xff\xd8"},
            ]
        self.image_runner._run_pedestrian_batch = types.MethodType(run, self.image_runner)
        runner = self._runner(frame_count=2)
        result = runner.run_video(self.video_path, sample_fps=1.0)
        self.assertEqual(result["summary"]["skipped_frames"], 1)
        self.assertEqual(result["summary"]["total_frames"], 2)
        self.assertEqual(result["summary"]["violation_count"], 0)

    def test_all_violation_frames_stored(self) -> None:
        targets = {
            i: [{"type": "pedestrian", "score": 0.9, "decision": "suspected_crossing_road_outside_zebra",
                 "reason": "road_dominant_without_zebra", "box": [1, 2, 3, 4],
                 "ground": {}}]
            for i in range(4)
        }
        self._wire_results(targets)
        runner = self._runner(frame_count=4, max_keyframes=2)
        result = runner.run_video(self.video_path, sample_fps=1.0)
        self.assertEqual(result["summary"]["violation_count"], 4)
        # No keyframe cap anymore: all frames stored.
        self.assertEqual(len(result["frames"]), 4)

    def test_first_frame_pipeline_uses_compatible_multifilesink(self) -> None:
        runner = self._runner(frame_count=1)
        frames_dir = self.work / "first"
        command = runner._first_frame_pipeline(self.video_path, frames_dir)
        self.assertIn("multifilesink", command)
        self.assertIn(str(frames_dir / "frame%05d.bgrx"), command[-1])
        self.assertNotIn("identity", command)
        self.assertFalse(any("eos-after" in argument for argument in command))

    def test_fixed_video_stream_runs_to_eof_on_hdmi(self) -> None:
        runner = self._runner(frame_count=1)
        gst_command, driver_command = runner._fixed_stream_commands(
            self.video_path, self.work / "fixed-mask.bin"
        )

        self.assertNotIn("videorate", gst_command)
        self.assertFalse(any("framerate=" in argument for argument in gst_command))
        self.assertNotIn("videoconvert", gst_command)
        self.assertNotIn("videoscale", gst_command)
        decode_index = gst_command.index("decodebin")
        self.assertEqual(gst_command[decode_index + 2], "capsfilter")
        self.assertEqual(gst_command[decode_index + 3], "caps=video/x-raw,format=NV12")
        identity_index = gst_command.index("identity")
        self.assertGreater(identity_index, decode_index)
        self.assertEqual(gst_command[identity_index + 1], "sync=true")
        self.assertLess(identity_index, gst_command.index("queue"))
        self.assertEqual(gst_command.count("leaky=downstream"), 1)
        self.assertEqual(gst_command.count("max-size-buffers=1"), 1)
        self.assertIn("sync=false", gst_command)
        self.assertIn("--input-nv12-stream", driver_command)
        self.assertIn("--src-width", driver_command)
        self.assertIn("--src-height", driver_command)
        self.assertIn("--fixed-mask", driver_command)
        frames_index = driver_command.index("--frames")
        self.assertEqual(driver_command[frames_index + 1], "0")
        display_index = driver_command.index("--display")
        self.assertEqual(driver_command[display_index + 1], "1")
        mask_index = driver_command.index("--display-mask")
        self.assertEqual(driver_command[mask_index + 1], "1")

    def test_first_frame_extraction_keeps_complete_frame_and_stops_decoder(self) -> None:
        runner = self._runner(frame_count=1)

        class FakeProcess:
            def __init__(self) -> None:
                self.running = True
                self.terminated = False
                self.stderr = io.BytesIO()

            def poll(self):
                return None if self.running else -15

            def terminate(self) -> None:
                self.running = False
                self.terminated = True

            def wait(self, timeout=None):
                del timeout
                self.running = False
                return -15

            def kill(self) -> None:
                self.running = False

        process = FakeProcess()

        def start_decoder(command, **kwargs):
            del kwargs
            location = command[-1].removeprefix("location=")
            Path(location.replace("%05d", "00000")).write_bytes(
                b"\0" * EXPECTED_BGRX_SIZE
            )
            return process

        extract_work = self.work / "extract"
        extract_work.mkdir()
        with mock.patch("image_inference.subprocess.Popen", side_effect=start_decoder):
            first_frame = runner._extract_first_frame(self.video_path, extract_work)

        self.assertEqual(first_frame, extract_work / "first-frame.bgrx")
        self.assertEqual(first_frame.stat().st_size, EXPECTED_BGRX_SIZE)
        self.assertTrue(process.terminated)


if __name__ == "__main__":
    unittest.main(verbosity=2)
