#!/usr/bin/env python3

import tempfile
import unittest
from pathlib import Path

from image_inference import ImageInferenceConfig, ImageInferenceRunner, completed_plate_result


class ImageInferenceTest(unittest.TestCase):
    def test_plate_result_requires_completed_positive_sequence(self) -> None:
        self.assertFalse(completed_plate_result({}))
        self.assertFalse(completed_plate_result({"sequence": 0}))
        self.assertFalse(completed_plate_result({"sequence": True}))
        self.assertFalse(completed_plate_result({"sequence": "1"}))
        self.assertTrue(completed_plate_result({"sequence": 1, "detections": []}))

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
        self.assertIn("best_int8_5color_scorex256_rk3568.rknn", joined)
        self.assertIn("plate_type_classifier_5color_large_green_v2_rk3568_fp16_opt0.rknn", joined)
        self.assertIn("pplcnet_police_v5_whiteexpand_rk3568_fp16.rknn", joined)
        self.assertIn("pplcnet_black_unified_v1_rk3568_fp16.rknn", joined)
        self.assertIn("black_unified_keys.txt", joined)
        self.assertEqual(command[command.index("--fps") + 1], "30")
        self.assertEqual(command[command.index("--det-score-scale") + 1], "256")
        self.assertEqual(command[command.index("--min-plate-conf") + 1], "0.35")
        self.assertEqual(command[command.index("--plate-nms-iou") + 1], "0.35")
        self.assertEqual(command[command.index("--plate-max-det") + 1], "9")

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


if __name__ == "__main__":
    unittest.main(verbosity=2)
