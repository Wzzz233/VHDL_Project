#!/usr/bin/env python3
"""Tests for the offline video inference runner.

These exercise the per-frame aggregation, frame sampling, batch inference,
truncation, and skip-on-error logic without touching gstreamer or the board
driver: _extract_frames is replaced with a stub that writes valid BGRx files,
and _run_pedestrian_batch is replaced with a stub that returns scripted results.
"""

from __future__ import annotations

import tempfile
import types
import unittest
from pathlib import Path

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
            frame_path.write_bytes(b"\0" * EXPECTED_BGRX_SIZE)
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

    def _wire_results(self, results_map) -> None:
        def run(self, frames_dir, output_dir, work):
            ordered = []
            for index in sorted(results_map):
                item = results_map[index]
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
        self._wire_results({0: [], 1: [], 2: [], 3: []})
        runner = self._runner(frame_count=4)
        result = runner.run_video(self.video_path, sample_fps=0.5)
        # 4 frames at 1fps, step 2 -> sampled frames 0 and 2
        self.assertEqual(result["summary"]["total_frames"], 2)
        self.assertFalse(result["summary"]["truncated"])

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


if __name__ == "__main__":
    unittest.main(verbosity=2)
