#!/usr/bin/env python3
"""Tests for the offline video inference runner.

These exercise the per-frame aggregation, frame sampling, keyframe linkage,
truncation, and skip-on-error logic without touching gstreamer or the board
driver: _extract_frames is replaced with a stub that writes valid BGRx files,
and _run_pedestrian is replaced with a stub that returns scripted results.
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
        def run(self, raw_path, work):
            index = int(raw_path.stem.replace("frame", ""))
            item = results_map.get(index, [])
            if isinstance(item, Exception):
                raise item
            return {"frame": index, "targets": item}, b"\xff\xd8fake-jpeg"

        self.image_runner._run_pedestrian = types.MethodType(run, self.image_runner)

    def _runner(self, frame_count, max_frames=600, max_keyframes=24) -> _VideoRunner:
        return _VideoRunner(
            self.image_runner,
            VideoInferenceConfig(max_frames=max_frames, max_keyframes=max_keyframes),
            frame_count,
        )

    def test_half_fps_samples_every_other_frame(self) -> None:
        self._wire_results({})
        runner = self._runner(frame_count=4)
        result = runner.run_video(self.video_path, sample_fps=0.5)
        # 4 frames at 1fps, step 2 -> sampled frames 0 and 2
        self.assertEqual(result["summary"]["total_frames"], 2)
        self.assertFalse(result["summary"]["truncated"])

    def test_events_link_keyframe(self) -> None:
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
        self.assertEqual(len(result["keyframes"]), 1)
        self.assertEqual(result["events"][0]["keyframe_index"], 0)
        self.assertEqual(result["events"][0]["time_sec"], 0.0)

    def test_max_frames_truncates(self) -> None:
        self._wire_results({})
        runner = self._runner(frame_count=5, max_frames=3)
        result = runner.run_video(self.video_path, sample_fps=1.0)
        self.assertEqual(result["summary"]["total_frames"], 3)
        self.assertTrue(result["summary"]["truncated"])

    def test_skipped_frame_does_not_abort(self) -> None:
        targets = {0: ImageInferenceError("driver failed"), 1: []}
        self._wire_results(targets)
        runner = self._runner(frame_count=2)
        result = runner.run_video(self.video_path, sample_fps=1.0)
        self.assertEqual(result["summary"]["skipped_frames"], 1)
        self.assertEqual(result["summary"]["total_frames"], 2)
        self.assertEqual(result["summary"]["violation_count"], 0)

    def test_keyframe_cap(self) -> None:
        # every frame is a violation, but only max_keyframes are stored
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
        self.assertEqual(len(result["keyframes"]), 2)
        # only the first two violation frames carry a keyframe_index
        linked = [event for event in result["events"] if "keyframe_index" in event]
        self.assertEqual(len(linked), 2)


if __name__ == "__main__":
    unittest.main(verbosity=2)
