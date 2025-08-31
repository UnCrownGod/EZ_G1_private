# File: vision/tests/test_transforms.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unit tests for projection/transforms.
单元测试：射线投影、地面求交、SE3 组合/逆的数值稳定性与边界条件。
"""

import numpy as np
from vision.fusion.tf_tree import invert, compose

def test_invert_compose():
    T = np.eye(4); T[:3,3] = [1,2,3]
    Ti = invert(T)
    I = compose(T, Ti)
    assert np.allclose(I, np.eye(4), atol=1e-6)
