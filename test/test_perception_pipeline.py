# File: vision/test/test_perception_pipeline.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unit tests for :mod:`vision.perception.pipeline`."""

from __future__ import annotations

import time
import unittest

import numpy as np

from vision.perception.pipeline import PerceptionPipeline


class _DummyStream:
    def __iter__(self):
        for idx in range(3):
            frame = np.full((4, 4, 3), idx, dtype=np.uint8)
            yield frame, float(idx)


class _SingleFrameStream:
    def __iter__(self):
        frame = np.zeros((2, 2, 3), dtype=np.uint8)
        yield frame, 0.0


class _DummyDetector:
    def infer(self, frame):  # pragma: no cover - simple stub
        return [{"bbox": [0, 0, 1, 1], "class_id": 0, "score": 1.0}]


class _FailingDetector:
    def infer(self, frame):
        raise RuntimeError("detector failed")


class _DummyTagDetector:
    def detect(self, gray, K, dist):  # pragma: no cover - simple stub
        return [{"id": 0, "T_cam_tag": np.eye(4)}]


class PerceptionPipelineTestCase(unittest.TestCase):
    def setUp(self) -> None:
        self.K = np.eye(3)
        self.dist = np.zeros(5)

    def tearDown(self) -> None:
        pass

    def test_pipeline_produces_samples(self) -> None:
        pipeline = PerceptionPipeline(
            frame_source=_DummyStream(),
            detector=_DummyDetector(),
            tag_detector=_DummyTagDetector(),
            camera_matrix=self.K,
            dist_coeffs=self.dist,
        )
        try:
            pipeline.start()
            sample = None
            deadline = time.time() + 2.0
            while time.time() < deadline and sample is None:
                sample = pipeline.latest()
                if sample is None:
                    time.sleep(0.05)
            self.assertIsNotNone(sample, "expected perception sample to be produced")
            assert sample is not None  # narrow type for mypy/linters
            self.assertGreaterEqual(sample.seq, 0)
            self.assertTrue(sample.detections)
            self.assertTrue(sample.tags)
        finally:
            pipeline.stop()

    def test_error_handler_invoked(self) -> None:
        errors: list[str] = []

        def on_error(exc: Exception) -> None:
            errors.append(str(exc))

        pipeline = PerceptionPipeline(
            frame_source=_SingleFrameStream(),
            detector=_FailingDetector(),
            tag_detector=_DummyTagDetector(),
            camera_matrix=self.K,
            dist_coeffs=self.dist,
            error_handler=on_error,
        )
        try:
            pipeline.start()
            time.sleep(0.1)
        finally:
            pipeline.stop()
        self.assertTrue(errors, "error handler should capture detector failure")


if __name__ == "__main__":  # pragma: no cover
    unittest.main()
