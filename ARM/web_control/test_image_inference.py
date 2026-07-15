#!/usr/bin/env python3

import tempfile
import unittest
from pathlib import Path
from unittest import mock

from image_inference import (
    ImageInferenceConfig,
    ImageInferenceError,
    ImageInferenceRunner,
    PlateImagePass,
    completed_plate_result,
    image_dimensions,
    map_plate_box,
    merge_plate_results,
    plate_input_index,
    plan_plate_passes,
)


class ImageInferenceTest(unittest.TestCase):
    def test_reads_png_and_jpeg_dimensions_without_image_library(self) -> None:
        png = b"\x89PNG\r\n\x1a\n" + b"\x00" * 8 + (321).to_bytes(4, "big") + (654).to_bytes(4, "big")
        jpeg = (
            b"\xff\xd8"
            + b"\xff\xe0\x00\x04\x00\x00"
            + b"\xff\xc0\x00\x0b\x08"
            + (480).to_bytes(2, "big")
            + (640).to_bytes(2, "big")
            + b"\x01\x01\x11\x00"
        )
        self.assertEqual(image_dimensions(png), (321, 654))
        self.assertEqual(image_dimensions(jpeg), (640, 480))

    def test_extreme_portrait_passes_include_top_and_bottom(self) -> None:
        passes = plan_plate_passes(1080, 3000)
        aspect_passes = [item for item in passes if item.name.startswith("aspect-")]
        self.assertEqual(passes[0].name, "global")
        self.assertLessEqual(len(passes), 9)
        intervals = sorted(
            (item.y, item.y + item.height) for item in aspect_passes
        )
        self.assertEqual(intervals[0][0], 0)
        self.assertEqual(intervals[-1][1], 3000)
        covered_until = 0
        for start, end in intervals:
            self.assertLessEqual(start, covered_until)
            covered_until = max(covered_until, end)
        self.assertEqual(covered_until, 3000)

    def test_narrow_strip_covers_every_row_and_checks_both_ends_first(self) -> None:
        passes = plan_plate_passes(203, 774)
        aspect_passes = [item for item in passes if item.name.startswith("aspect-")]
        self.assertEqual(len(passes), 9)
        self.assertEqual(aspect_passes[0].y, 0)
        self.assertEqual(
            aspect_passes[1].y + aspect_passes[1].height,
            774,
        )
        intervals = sorted(
            (item.y, item.y + item.height) for item in aspect_passes
        )
        covered_until = 0
        for start, end in intervals:
            self.assertLessEqual(start, covered_until)
            covered_until = max(covered_until, end)
        self.assertEqual(covered_until, 774)

    def test_typical_photo_is_bounded_to_five_passes(self) -> None:
        passes = plan_plate_passes(1920, 1080)
        self.assertEqual(len(passes), 5)
        self.assertEqual(passes[0].name, "global")

    def test_global_letterbox_box_maps_back_to_original_image(self) -> None:
        mapped = map_plate_box(
            [280, 0, 1000, 720],
            PlateImagePass("global", 0, 0, 1000, 1000),
            1000,
            1000,
        )
        self.assertEqual(mapped, [0, 0, 1000, 1000])

    def test_merge_keeps_global_result_and_only_adds_new_plate(self) -> None:
        global_pass = PlateImagePass("global", 0, 0, 1280, 720)
        tile_pass = PlateImagePass("aspect-0", 0, 0, 1280, 720)
        merged = merge_plate_results(
            [
                (global_pass, {"sequence": 1, "detections": [
                    {"x1": 100, "y1": 100, "x2": 300, "y2": 200,
                     "text": "OLD", "det_conf": 0.8},
                ]}),
                (tile_pass, {"sequence": 2, "detections": [
                    {"x1": 105, "y1": 105, "x2": 305, "y2": 205,
                     "text": "REPLACEMENT", "det_conf": 0.99},
                    {"x1": 600, "y1": 100, "x2": 800, "y2": 200,
                     "text": "NEW", "det_conf": 0.7},
                ]}),
            ],
            1280,
            720,
        )
        self.assertEqual(
            [detection["text"] for detection in merged["detections"]],
            ["OLD", "NEW"],
        )
        self.assertEqual(merged["detections"][0]["box"], [100, 100, 300, 200])
        self.assertEqual(
            [
                merged["detections"][0]["x1"],
                merged["detections"][0]["y1"],
                merged["detections"][0]["x2"],
                merged["detections"][0]["y2"],
            ],
            [100, 100, 300, 200],
        )

    def test_detail_decode_uses_crop_and_outer_padding(self) -> None:
        runner = ImageInferenceRunner(ImageInferenceConfig())
        image_pass = PlateImagePass(
            "detail-tl", 0, 0, 700, 700, pad_left=84, pad_top=84
        )
        with mock.patch.object(runner, "_run_decode_command") as run_decode:
            runner._decode_plate_pass(
                Path("input.jpg"), Path("output.bgrx"), image_pass, 1000, 1000
            )
        command = run_decode.call_args.args[0]
        self.assertIn("videocrop", command)
        self.assertIn("right=300", command)
        self.assertIn("bottom=300", command)
        self.assertIn("videobox", command)

    def test_failed_local_pass_returns_global_result(self) -> None:
        jpeg = (
            b"\xff\xd8\xff\xc0\x00\x0b\x08"
            + (3000).to_bytes(2, "big")
            + (1080).to_bytes(2, "big")
            + b"\x01\x01\x11\x00"
        )
        runner = ImageInferenceRunner(ImageInferenceConfig())

        def decode_global_only(
            _image_path, _raw_path, image_pass, _width, _height, timeout=None
        ):
            if image_pass.name != "global":
                raise ImageInferenceError("local pass failed")

        with (
            mock.patch.object(
                runner, "_decode_plate_pass", side_effect=decode_global_only
            ),
            mock.patch.object(
                runner,
                "_run_plate_batch",
                return_value={
                    0: {"sequence": 1, "detections": [
                        {"x1": 570, "y1": 100, "x2": 710, "y2": 180,
                         "text": "GLOBAL"},
                    ]},
                },
            ),
        ):
            response = runner.run("plate", jpeg, "image/jpeg")
        self.assertTrue(response["ok"])
        self.assertEqual(response["frame"], {"width": 1080, "height": 3000})
        self.assertEqual(
            response["results"]["detections"][0]["text"], "GLOBAL"
        )
        self.assertEqual(response["results"]["pass_count"], 1)
        self.assertIn("pass_warnings", response["results"])

    def test_plate_result_requires_completed_positive_sequence(self) -> None:
        self.assertFalse(completed_plate_result({}))
        self.assertFalse(completed_plate_result({"sequence": 0}))
        self.assertFalse(completed_plate_result({"sequence": True}))
        self.assertFalse(completed_plate_result({"sequence": "1"}))
        self.assertTrue(completed_plate_result({"sequence": 1, "detections": []}))

    def test_static_input_sequence_maps_to_the_correct_window(self) -> None:
        self.assertEqual(plate_input_index(1, 9, 75), 0)
        self.assertEqual(plate_input_index(75, 9, 75), 0)
        self.assertEqual(plate_input_index(76, 9, 75), 1)
        self.assertEqual(plate_input_index(675, 9, 75), 8)
        self.assertEqual(plate_input_index(676, 9, 75), 0)
        with self.assertRaises(ValueError):
            plate_input_index(0, 9, 75)

    def test_extreme_upload_runs_all_windows_in_one_driver_process(self) -> None:
        jpeg = (
            b"\xff\xd8\xff\xc0\x00\x0b\x08"
            + (774).to_bytes(2, "big")
            + (203).to_bytes(2, "big")
            + b"\x01\x01\x11\x00"
        )
        runner = ImageInferenceRunner(ImageInferenceConfig())
        batch = {
            index: {"sequence": index + 1, "detections": []}
            for index in range(9)
        }
        with (
            mock.patch.object(runner, "_decode_plate_pass"),
            mock.patch.object(
                runner, "_run_plate_batch", return_value=batch
            ) as run_batch,
        ):
            response = runner.run("plate", jpeg, "image/jpeg")
        self.assertEqual(run_batch.call_count, 1)
        self.assertEqual(len(run_batch.call_args.args[0]), 9)
        self.assertEqual(response["results"]["pass_count"], 9)

    def test_plate_upload_uses_five_color_pipeline(self) -> None:
        with tempfile.TemporaryDirectory(prefix="image-command-test-") as directory:
            root = Path(directory)
            runner = ImageInferenceRunner(
                ImageInferenceConfig(
                    arm_root=root,
                    model_root=Path("/userdata/model"),
                    plate_driver=root / "pplcnet_bgp_live",
                )
            )
            command = runner._plate_command(root / "input.bgrx", root / "control.sock")
        joined = " ".join(command)
        self.assertIn("best_fp16_5color_largegreen_scorex256_rk3568.rknn", joined)
        self.assertIn("plate_type_classifier_5color_large_green_v2_rk3568_fp16_opt0.rknn", joined)
        self.assertIn("pplcnet_police_v5_whiteexpand_rk3568_fp16.rknn", joined)
        self.assertIn("pplcnet_black_unified_v1_rk3568_fp16.rknn", joined)
        self.assertIn("black_unified_keys.txt", joined)
        self.assertEqual(
            command[command.index("--plate-type-classifier-min-conf") + 1],
            "0.95",
        )
        self.assertEqual(
            command[command.index("--plate-type-classifier-special-min-conf") + 1],
            "0.70",
        )
        self.assertEqual(command[command.index("--fps") + 1], "30")
        self.assertEqual(command[command.index("--det-score-scale") + 1], "256")
        self.assertEqual(command[command.index("--min-plate-conf") + 1], "0.35")
        self.assertEqual(command[command.index("--plate-nms-iou") + 1], "0.35")
        self.assertEqual(command[command.index("--plate-max-det") + 1], "24")

    def test_plate_upload_accepts_classifier_override(self) -> None:
        classifier = Path("/userdata/model/plate_type_classifier_20260602.rknn")
        runner = ImageInferenceRunner(
            ImageInferenceConfig(
                plate_type_model=classifier,
            )
        )
        command = runner._plate_command(Path("input.bgrx"), Path("control.sock"))
        self.assertEqual(
            command[command.index("--plate-type-classifier-model") + 1],
            str(classifier),
        )

    def test_plate_batch_command_uses_list_source(self) -> None:
        runner = ImageInferenceRunner(ImageInferenceConfig())
        command = runner._plate_command(
            None,
            Path("control.sock"),
            raw_list_path=Path("inputs.txt"),
            input_repeat=75,
        )
        self.assertEqual(
            command[command.index("--input-bgrx-list") + 1], "inputs.txt"
        )
        self.assertEqual(
            command[command.index("--input-bgrx-repeat") + 1], "75"
        )
        self.assertEqual(command[command.index("--frames") + 1], "0")


if __name__ == "__main__":
    unittest.main(verbosity=2)
