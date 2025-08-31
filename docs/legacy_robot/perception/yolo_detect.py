#!/usr/bin/env python3
# robot/perception/yolo_detect.py
"""
YOLO detector wrapper using Ultralytics.
Ultralytics YOLO 推理封装（可选，用于 ROI 过滤）。

Requires:
    pip install ultralytics
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Tuple

import numpy as np
from ultralytics import YOLO


@dataclass
class Detection:
    """Single detection record.

    单个检测结果。

    Args:
        cls_name (str): Class name.
        conf (float): Confidence score.
        xyxy (Tuple[float, float, float, float]): Box in xyxy.

    Raises:
        ValueError: If fields are invalid.
    """
    cls_name: str
    conf: float
    xyxy: Tuple[float, float, float, float]

    def __post_init__(self) -> None:
        if not (0.0 <= self.conf <= 1.0):
            raise ValueError("conf must be in [0, 1]")


class YoloDetector:
    """Ultralytics YOLO detector.

    Ultralytics YOLO 检测器。

    Raises:
        RuntimeError: If model fails to load.
    """

    def __init__(self, weights: str, class_filter: Optional[List[str]] = None) -> None:
        """Initialize model.

        初始化模型。

        Args:
            weights (str): Path to YOLO weights (e.g., best.pt).
            class_filter (Optional[List[str]]): Keep only these classes.

        Returns:
            None
        """
        self.model = YOLO(weights)
        self.class_filter = set(class_filter) if class_filter else None

    def infer(self, bgr: np.ndarray, conf: float = 0.25) -> List[Detection]:
        """Run detection.

        执行检测。

        Args:
            bgr (np.ndarray): BGR image (H,W,3).
            conf (float): Confidence threshold.

        Returns:
            List[Detection]: Sorted by confidence desc.

        Raises:
            ValueError: If image is empty.
        """
        if bgr is None or bgr.size == 0:
            raise ValueError("empty image")

        res = self.model.predict(source=bgr, verbose=False, conf=conf)[0]
        dets: List[Detection] = []
        for b in res.boxes:
            cls_idx = int(b.cls.item())
            cls_name = res.names.get(cls_idx, str(cls_idx))
            if self.class_filter and cls_name not in self.class_filter:
                continue
            x1, y1, x2, y2 = map(float, b.xyxy[0].tolist())
            dets.append(Detection(cls_name=cls_name, conf=float(b.conf.item()), xyxy=(x1, y1, x2, y2)))
        dets.sort(key=lambda d: d.conf, reverse=True)
        return dets
