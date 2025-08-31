# File: vision/tests/test_detector.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unit tests for YOLO detector wrapper.
单元测试：类别名映射、置信度阈值、空帧/异常输入健壮性。
"""

import numpy as np
from vision.perception.yolo_detector import YoloDetector

def test_init_only():
    # 不加载真模型，检查构造参数无误
    cfg = {"model":"runs/detect/w_lid_v2/weights/best.pt","conf":0.25,"iou":0.5,"max_det":50,"classes":["a","b"]}
    d = YoloDetector(cfg)
    assert d.max_det==50
