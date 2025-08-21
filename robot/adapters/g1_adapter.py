#!/usr/bin/env python3
# robot/adapters/g1_adapter.py
"""
G1 adapter (stub): motion & gripper interfaces to be bound on-site.
G1 适配层（桩实现）：运动与夹爪接口，现场接入 SDK/ROS2 即可替换。
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional
import numpy as np


@dataclass
class Pose:
    """Simple SE(3) pose holder.

    简单 SE(3) 位姿容器。

    Args:
        T (np.ndarray): 4x4 homogeneous transform (row-major).

    Raises:
        ValueError: If ``T`` is not 4x4.
    """
    T: np.ndarray

    def __post_init__(self) -> None:
        if not (isinstance(self.T, np.ndarray) and self.T.shape == (4, 4)):
            raise ValueError("T must be a 4x4 numpy array")


class G1Adapter:
    """G1 robot adapter (stub).

    G1 机器人适配层（桩实现）。现场将这些方法接入实际 SDK/ROS2。

    Methods implement no-op logs for now.

    Raises:
        RuntimeError: If called with invalid inputs.
    """

    def __init__(self, ip: Optional[str] = None) -> None:
        """Initialize adapter.

        初始化适配层。

        Args:
            ip (Optional[str]): Robot IP or connection hint. Not used in stub.

        Returns:
            None
        """
        self.ip = ip

    def move_to_pose(self, pose: Pose, speed: float = 0.2) -> None:
        """Move robot to target pose (stub).

        运动到目标位姿（桩）。

        Args:
            pose (Pose): Target SE(3) pose in base frame.
            speed (float): Motion speed in m/s or normalized scale.

        Returns:
            None

        Raises:
            RuntimeError: If pose matrix is invalid.
        """
        if not isinstance(pose, Pose):
            raise RuntimeError("pose must be a Pose")
        print(f"[G1Adapter] move_to_pose @speed={speed}\n{pose.T}")

    def open_gripper(self) -> None:
        """Open gripper (stub).

        打开夹爪（桩）。

        Args:
            None

        Returns:
            None
        """
        print("[G1Adapter] open_gripper()")

    def close_gripper(self) -> None:
        """Close gripper (stub).

        闭合夹爪（桩）。

        Args:
            None

        Returns:
            None
        """
        print("[G1Adapter] close_gripper()")

    def execute_path(self, poses: Iterable[Pose], speed: float = 0.2) -> None:
        """Execute multi-waypoint path (stub).

        执行多路点路径（桩）。

        Args:
            poses (Iterable[Pose]): Waypoint poses in base frame.
            speed (float): Motion speed.

        Returns:
            None

        Raises:
            RuntimeError: If any pose is invalid.
        """
        for i, p in enumerate(poses):
            print(f"[G1Adapter] Waypoint {i}")
            self.move_to_pose(p, speed=speed)
