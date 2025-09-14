# File: vision/fusion/tf_tree.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Lightweight TF tree utilities (numpy 4x4 SE3).
坐标系管理：构造/组合/逆变换/应用点云与射线。

Provides:
    se3(R,t), inv(T), compose(A,B), transform_points(T, XYZ)

Usage:
    T_world_base = compose(T_world_cam, inv(T_base_cam))
"""

from __future__ import annotations
import numpy as np

def invert(T: np.ndarray) -> np.ndarray:
    R = T[:3,:3]; t = T[:3,3]
    Ti = np.eye(4)
    Ti[:3,:3] = R.T
    Ti[:3,3]  = -R.T @ t
    return Ti

def compose(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    return A @ B
 