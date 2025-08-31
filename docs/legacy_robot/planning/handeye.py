#!/usr/bin/env python3
# robot/planning/handeye.py
"""
Calibration & transform helpers.
标定与坐标变换工具。
"""

from __future__ import annotations

from typing import Dict
import numpy as np
import yaml


def load_yaml(path: str) -> Dict:
    """Load YAML file.

    读取 YAML。

    Args:
        path (str): YAML file path.

    Returns:
        Dict: Parsed YAML dict.

    Raises:
        FileNotFoundError: If file missing.
        yaml.YAMLError: If YAML invalid.
    """
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def vec_to_T(t_xyz: list[float]) -> np.ndarray:
    """Make 4x4 from translation vector.

    由平移向量构造位姿（单位矩阵的平移部分）。

    Args:
        t_xyz (list[float]): [tx, ty, tz] in meters.

    Returns:
        np.ndarray: 4x4 transform.

    Raises:
        ValueError: If length != 3.
    """
    if len(t_xyz) != 3:
        raise ValueError("t_xyz must be length 3")
    T = np.eye(4)
    T[:3, 3] = np.array(t_xyz, dtype=float).reshape(3)
    return T


def rpy_to_R(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """RPY (XYZ) to rotation matrix.

    RPY（XYZ 顺序）转旋转矩阵。

    Args:
        roll (float): Roll (rad).
        pitch (float): Pitch (rad).
        yaw (float): Yaw (rad).

    Returns:
        np.ndarray: 3x3 rotation.

    Raises:
        None
    """
    rx, ry, rz = roll, pitch, yaw
    Rx = np.array([[1, 0, 0], [0, np.cos(rx), -np.sin(rx)], [0, np.sin(rx), np.cos(rx)]])
    Ry = np.array([[np.cos(ry), 0, np.sin(ry)], [0, 1, 0], [-np.sin(ry), 0, np.cos(ry)]])
    Rz = np.array([[np.cos(rz), -np.sin(rz), 0], [np.sin(rz), np.cos(rz), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def RT_to_T(R: np.ndarray, t: np.ndarray) -> np.ndarray:
    """Assemble 4x4 from R,t.

    由 R,t 组装 4x4。

    Args:
        R (np.ndarray): 3x3 rotation.
        t (np.ndarray): 3 translation.

    Returns:
        np.ndarray: 4x4 transform.

    Raises:
        ValueError: If shapes invalid.
    """
    if R.shape != (3, 3) or t.size != 3:
        raise ValueError("R must be 3x3 and t length 3")
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t.reshape(3)
    return T


def compose(A: np.ndarray, B: np.ndarray) -> np.ndarray:
    """Compose transforms: A @ B.

    组合位姿：A 后乘 B。

    Args:
        A (np.ndarray): 4x4.
        B (np.ndarray): 4x4.

    Returns:
        np.ndarray: A @ B.

    Raises:
        ValueError: If shapes invalid.
    """
    if A.shape != (4, 4) or B.shape != (4, 4):
        raise ValueError("both must be 4x4")
    return A @ B
