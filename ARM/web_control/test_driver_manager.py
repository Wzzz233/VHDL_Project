#!/usr/bin/env python3

import tempfile
import unittest
from pathlib import Path

from driver_manager import DriverManager


class FakeDriverManager(DriverManager):
    def __init__(self, root: Path) -> None:
        script = root / "plate.sh"
        script.write_text("#!/bin/sh\nexit 0\n", encoding="ascii")
        script.chmod(0o755)
        super().__init__(
            plate_script=script,
            runtime_dir=root / "run",
            log_dir=root / "log",
            model_dir=root / "models",
            startup_grace=0.0,
            stop_timeout=0.0,
        )
        self.running = True
        self.starts = 0
        self.stops = 0

    def _start_plate_locked(self) -> None:
        self.running = True
        self.starts += 1

    def _stop_plate_locked(self) -> None:
        self.running = False
        self.stops += 1

    def _alive(self, pid) -> bool:
        return self.running

    def status(self):
        return {
            "ok": True,
            "mode": self._read_mode(),
            "plate_running": self.running,
            "pedestrian_on_demand": True,
        }


class DriverManagerTest(unittest.TestCase):
    def test_switch_and_exclusive_image_sessions(self) -> None:
        with tempfile.TemporaryDirectory(prefix="driver-manager-test-") as directory:
            manager = FakeDriverManager(Path(directory))
            status = manager.set_mode("pedestrian")
            self.assertEqual(status["mode"], "pedestrian")
            self.assertFalse(status["plate_running"])

            manager.set_mode("plate")
            self.assertTrue(manager.running)
            with manager.image_session("plate"):
                self.assertFalse(manager.running)
            self.assertTrue(manager.running)

            # A one-shot pedestrian inference from the plate live state must
            # restore the plate live pipeline afterwards, otherwise the preview
            # freezes when returning to the camera.
            with manager.image_session("pedestrian"):
                self.assertFalse(manager.running)
            self.assertTrue(manager.running)
            self.assertEqual(manager._read_mode(), "plate")

            # When already in pedestrian mode, a pedestrian one-shot must not
            # start the plate live pipeline.
            manager.set_mode("pedestrian")
            with manager.image_session("pedestrian"):
                self.assertFalse(manager.running)
            self.assertFalse(manager.running)
            self.assertEqual(manager._read_mode(), "pedestrian")


if __name__ == "__main__":
    unittest.main(verbosity=2)
