#!/usr/bin/env python3
# robot/planning/grasp_planner.py
"""
Grasp pose synthesis from rack/tag pose.
基于板架/Tag 位姿合成抓取位姿。
"""

from __future__ import annotations

from typing import Tuple
import numpy as np

from .handeye import compose, rpy_to_R, RT_to_T


def make_grasp_from_tag(
    T_base_tag: np.ndarray,
    approach_offset_m: float = 0.10,
    grasp_offset_m: float = 0.02,
    rpy_tool_rad: Tuple[float, float, float] = (0.0, 0.0, 0.0),
) -> tuple[np.ndarray, np.ndarray]:
    """Create pre-grasp and grasp poses from tag pose.

    基于 Tag 位姿生成预抓与抓取位姿（简化假设：沿 Tag -Z 方向接近）。

    Args:
        T_base_tag (np.ndarray): 4x4 transform of tag in base frame.
        approach_offset_m (float): Pre-grasp distance along approach axis.
        grasp_offset_m (float): Final inset along approach axis.
        rpy_tool_rad (Tuple[float, float, float]): Tool orientation override (base frame RPY).

    Returns:
        tuple[np.ndarray, np.ndarray]: (T_base_pregrasp, T_base_grasp)

    Raises:
        ValueError: If input shapes invalid.
    """
    if T_base_tag.shape != (4, 4):
        raise ValueError("T_base_tag must be 4x4")

    # Assumption: Tag's +Z is camera-facing; approach along Tag -Z.
    # We align tool Z with Tag -Z and apply optional RPY tool tweak.
    R_tag = T_base_tag[:3, :3]
    t_tag = T_base_tag[:3, 3]

    # Approach direction in base: -tag_Z
    z_tag = R_tag[:, 2]  # tag +Z
    approach_dir = -z_tag / (np.linalg.norm(z_tag) + 1e-9)

    # Tool orientation
    R_tool = rpy_to_R(*rpy_tool_rad)
    T_tool = RT_to_T(R_tool, np.zeros(3))

    # Pre-grasp and grasp positions
    t_pre = t_tag + approach_dir * approach_offset_m
    t_grasp = t_tag + approach_dir * grasp_offset_m

    T_base_tag_no_rot = RT_to_T(np.eye(3), t_tag)
    # Orient along tag (could refine with side alignment if needed)
    T_base_oriented = T_base_tag.copy()
    T_pre = T_base_oriented.copy()
    T_pre[:3, 3] = t_pre
    T_grasp = T_base_oriented.copy()
    T_grasp[:3, 3] = t_grasp

    # Apply tool tweak
    T_pre = compose(T_pre, T_tool)
    T_grasp = compose(T_grasp, T_tool)
    return T_pre, T_grasp
