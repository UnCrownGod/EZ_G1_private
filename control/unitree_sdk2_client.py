# File: vision/control/unitree_sdk2_client.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Unitree SDK2 thin client placeholder with watchdog-friendly API.

The real Unitree SDK2 Python bindings are not publicly released yet. This module
provides a drop-in interface for higher layers (ROS nodes, behaviors) while we
prototype against simulation or replay data. Once the official SDK is available,
replace the TODO sections with real transport code.

API surface:
    client = UnitreeSDK2Client(endpoint="udp://192.168.12.1:8082")
    client.stand()
    client.go(vx=0.2, vy=0.0, wz=0.1, duration=0.2)
    client.stop()
    client.gripper_open(width=0.05)
    client.gripper_close()
    client.arm_move_joints([0, -0.5, 1.0, ...])

All calls are thread-safe and record the last command for debugging.
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple


@dataclass
class CommandLog:
    """Lightweight record of the latest SDK command."""

    name: str
    args: Tuple
    kwargs: dict
    timestamp: float = field(default_factory=time.time)


class UnitreeSDK2Client:
    """Mockable Unitree SDK2 client wrapper."""

    def __init__(self, endpoint: Optional[str] = None, watchdog_timeout: float = 1.5) -> None:
        self.endpoint = endpoint
        self.watchdog_timeout = float(watchdog_timeout)
        self.mock = True
        self._lock = threading.Lock()
        self._last_command: Optional[CommandLog] = None
        self._last_motion_time = time.monotonic()

        try:  # pragma: no cover - real SDK rarely available
            import unitree_sdk2  # type: ignore  # noqa

            self._sdk = unitree_sdk2
            self.mock = False
        except Exception:
            self._sdk = None
            self.mock = True
            print("[SDK2] Official SDK not found. Using mock client.")

    # ------------------------------------------------------------------
    # Helpers
    # ------------------------------------------------------------------
    def _record(self, name: str, *args, **kwargs) -> None:
        with self._lock:
            self._last_command = CommandLog(name=name, args=args, kwargs=kwargs)
            if name in {"go", "stand", "stop"}:
                self._last_motion_time = time.monotonic()

    def _log(self, message: str) -> None:
        prefix = "SDK2-REAL" if not self.mock else "SDK2-MOCK"
        print(f"[{prefix}] {message}")

    # ------------------------------------------------------------------
    # Public control interface
    # ------------------------------------------------------------------
    def stand(self) -> None:
        self._record("stand")
        self._log("stand()")
        if not self.mock:
            # TODO: invoke real SDK command
            pass

    def stop(self) -> None:
        self._record("stop")
        self._log("stop()")
        if not self.mock:
            # TODO: send zero velocity / disengage
            pass

    def go(self, vx: float, vy: float, wz: float, duration: float = 0.1) -> None:
        duration = max(0.0, float(duration))
        self._record("go", vx, vy, wz, duration=duration)
        self._log(f"go(vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}, duration={duration:.2f})")
        if not self.mock:
            # TODO: call SDK velocity command with duration gating
            pass
        time.sleep(duration)

    def gripper_open(self, width: float = 0.06) -> None:
        width = max(0.0, float(width))
        self._record("gripper_open", width)
        self._log(f"gripper_open(width={width:.3f})")
        if not self.mock:
            # TODO: send gripper width command
            pass

    def gripper_close(self, force: Optional[float] = None) -> None:
        self._record("gripper_close", force)
        if force is not None:
            self._log(f"gripper_close(force={force:.1f})")
        else:
            self._log("gripper_close()")
        if not self.mock:
            # TODO: close gripper with optional force
            pass

    def arm_move_joints(self, joints: Sequence[float], duration: float = 1.5) -> None:
        joints_list = list(joints)
        self._record("arm_move_joints", tuple(joints_list), duration=duration)
        self._log(f"arm_move_joints(joints={joints_list}, duration={duration:.2f})")
        if not self.mock:
            # TODO: command arm trajectory
            pass

    def arm_move_pose(self, pose: Sequence[float], duration: float = 2.0) -> None:
        pose_list = list(pose)
        self._record("arm_move_pose", tuple(pose_list), duration=duration)
        self._log(f"arm_move_pose(pose={pose_list}, duration={duration:.2f})")
        if not self.mock:
            # TODO: cartesian pose control
            pass

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------
    def last_command(self) -> Optional[CommandLog]:
        with self._lock:
            return self._last_command

    def seconds_since_motion(self) -> float:
        return time.monotonic() - self._last_motion_time

    def is_connected(self) -> bool:
        return not self.mock

