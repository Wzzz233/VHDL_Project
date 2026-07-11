#!/usr/bin/env python3

import unittest

from image_inference import completed_plate_result


class ImageInferenceTest(unittest.TestCase):
    def test_plate_result_requires_completed_positive_sequence(self) -> None:
        self.assertFalse(completed_plate_result({}))
        self.assertFalse(completed_plate_result({"sequence": 0}))
        self.assertFalse(completed_plate_result({"sequence": True}))
        self.assertFalse(completed_plate_result({"sequence": "1"}))
        self.assertTrue(completed_plate_result({"sequence": 1, "detections": []}))


if __name__ == "__main__":
    unittest.main(verbosity=2)
