#!/usr/bin/env python3
# robot/tasks/task_manager.py
"""
Minimal pick-place state machine (offline).
离线最小取放状态机（示例）。
"""

from __future__ import annotations

from typing import Optional
import numpy as np

from adapters.g1_adapter import G1Adapter, Pose
from planning.grasp_planner import make_grasp_from_tag


def pick_and_place_once(
    adapter: G1Adapter,
    T_base_tag: np.ndarray,
    T_base_place: np.ndarray,
    approach_offset_m: float = 0.10,
    grasp_offset_m: float = 0.02,
) -> None:
    """Execute one pick-&-place cycle (offline stub).

    执行一次取放（离线桩）。

    Args:
        adapter (G1Adapter): Robot adapter (stub now).
        T_base_tag (np.ndarray): Tag pose in base frame (from vision).
        T_base_place (np.ndarray): Place pose in base frame (calibrated).
        approach_offset_m (float): Pre-grasp distance.
        grasp_offset_m (float): Final inset distance.

    Returns:
        None

    Raises:
        ValueError: If any transform invalid.
    """
    pre, grasp = make_grasp_from_tag(T_base_tag, approach_offset_m, grasp_offset_m)
    adapter.open_gripper()
    adapter.move_to_pose(Pose(pre))
    adapter.move_to_pose(Pose(grasp))
    adapter.close_gripper()
    adapter.move_to_pose(Pose(pre))  # retreat
    adapter.move_to_pose(Pose(T_base_place))
    adapter.open_gripper()
