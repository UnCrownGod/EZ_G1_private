# File: vision/perception/pipeline.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Reusable perception pipeline utilities.

This module encapsulates the long-running camera/inference loop so higher level
consumers (ROS nodes, CLI tools) can subscribe to the latest detections without
managing threads themselves.

The pipeline pulls frames from a generic iterator that yields ``(frame, ts)``
tuples, runs YOLO detection and AprilTag pose estimation, and stores the most
recent result. The loop can be stopped gracefully, making it suitable for
integration with launch/shutdown hooks.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass
from typing import Callable, Iterable, Iterator, List, Optional, Tuple

import numpy as np

try:  # OpenCV is optional for unit tests
    import cv2
except Exception:  # pragma: no cover
    cv2 = None  # type: ignore


FrameType = np.ndarray
FrameIterator = Iterator[Tuple[FrameType, float]]
DetectionList = List[dict]
TagList = List[dict]
GrayFn = Callable[[FrameType], np.ndarray]


@dataclass
class PerceptionSample:
    """Container for the latest perception result."""

    seq: int
    frame_shape: Tuple[int, int, int]
    detections: DetectionList
    tags: TagList
    timestamp: float


class PerceptionPipeline:
    """Background perception worker with thread-safe latest cache."""

    def __init__(
        self,
        frame_source: Iterable[Tuple[FrameType, float]] | FrameIterator,
        detector: object,
        tag_detector: object,
        camera_matrix: np.ndarray,
        dist_coeffs: np.ndarray,
        gray_fn: Optional[GrayFn] = None,
        error_handler: Optional[Callable[[Exception], None]] = None,
    ) -> None:
        self._frame_source = frame_source
        self._detector = detector
        self._tag_detector = tag_detector
        self._K = camera_matrix
        self._dist = dist_coeffs
        self._gray_fn = gray_fn or _default_gray
        self._error_handler = error_handler

        self._lock = threading.Lock()
        self._latest: Optional[PerceptionSample] = None
        self._seq = 0
        self._stop = threading.Event()
        self._thread: Optional[threading.Thread] = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------
    def start(self) -> None:
        if self._thread and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="perception-pipeline", daemon=True)
        self._thread.start()

    def stop(self, timeout: float = 1.0) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=timeout)
            self._thread = None

    def latest(self) -> Optional[PerceptionSample]:
        with self._lock:
            return self._latest

    # ------------------------------------------------------------------
    # Internal loop
    # ------------------------------------------------------------------
    def _run(self) -> None:
        iterator = iter(self._frame_source)
        while not self._stop.is_set():
            try:
                frame, ts = next(iterator)
            except StopIteration:
                break
            except Exception as exc:  # pragma: no cover - guarded because hardware loops can raise
                _notify(self._error_handler, exc)
                time.sleep(0.1)
                continue

            if frame is None:
                time.sleep(0.01)
                continue

            try:
                detections = _invoke_detector(self._detector, frame)
                gray = self._gray_fn(frame)
                tags = _invoke_tag_detector(self._tag_detector, gray, self._K, self._dist)
            except Exception as exc:
                _notify(self._error_handler, exc)
                time.sleep(0.05)
                continue

            sample = PerceptionSample(
                seq=self._seq,
                frame_shape=tuple(frame.shape),
                detections=detections,
                tags=tags,
                timestamp=float(ts),
            )
            with self._lock:
                self._seq += 1
                self._latest = sample


# ----------------------------------------------------------------------
# Helpers
# ----------------------------------------------------------------------

def _default_gray(frame: FrameType) -> np.ndarray:
    if cv2 is not None:
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    # Fallback: simple average to mocked grayscale (frame assumed uint8)
    return np.mean(frame, axis=2).astype(frame.dtype)


def _invoke_detector(detector: object, frame: FrameType) -> DetectionList:
    infer = getattr(detector, "infer", None)
    if callable(infer):
        return list(infer(frame))
    raise AttributeError("detector must provide an infer(frame) method")


def _invoke_tag_detector(tag_detector: object, gray: np.ndarray, K: np.ndarray, dist: np.ndarray) -> TagList:
    detect = getattr(tag_detector, "detect", None)
    if callable(detect):
        return list(detect(gray, K, dist))
    raise AttributeError("tag_detector must provide detect(gray, K, dist)")


def _notify(handler: Optional[Callable[[Exception], None]], exc: Exception) -> None:
    if handler is None:
        return
    try:
        handler(exc)
    except Exception:
        pass


__all__ = ["PerceptionPipeline", "PerceptionSample"]
