# File: vision/control/unitree_sdk2_client.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unitree SDK2 thin client (placeholder).
Unitree 机器人 SDK2 轻量封装（站立/停止/速度控制/抓手开合等接口占位），便于后续对接真机或仿真桥。

API:
    client = UnitreeSDK2Client(endpoint="udp://robot-ip")
    client.stand(); client.go(vx, vy, wz, dur); client.stop()
    client.gripper_open(); client.gripper_close()

Note:
    实机到货后替换内部实现，可保留同样方法签名不改上层代码。
"""

from __future__ import annotations
import time
from typing import Optional

class UnitreeSDK2Client:
    """
    统一封装。若官方 SDK2 Python 包可用，则在此对接；否则降级为打印日志（便于仿真/调试）。
    """
    def __init__(self, endpoint: Optional[str]=None):
        self.endpoint = endpoint
        self.mock = True
        try:
            import unitree_sdk2  # noqa
            self.mock = False
            # TODO: 初始化真实 SDK2 客户端
        except Exception:
            self.mock = True
            print("[SDK2] 未检测到官方 SDK2 包，使用 MOCK 客户端。")

    def stand(self):
        self._log("stand()")
    def stop(self):
        self._log("stop()")
    def go(self, vx: float, vy: float, wz: float, duration: float=0.1):
        self._log(f"go(vx={vx:.2f}, vy={vy:.2f}, wz={wz:.2f}, {duration:.2f}s)")
        time.sleep(duration)
    def gripper_open(self, width: float=0.06):
        self._log(f"gripper_open({width:.3f})")
    def gripper_close(self):
        self._log("gripper_close()")
    def _log(self, msg: str):
        if self.mock:
            print(f"[SDK2-MOCK] {msg}")
        else:
            # TODO: 调用真实 SDK2
            print(f"[SDK2-REAL] {msg}")
