# File: vision/control/navigator.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple goal-based navigator with timeouts.
简易导航：朝向/前进/到达检测（距离阈值+时间上限），用于靠近目标点或贴近 AprilTag 点位。

Usage:
    nav = Navigator(client, Limits(vx=0.4, vy=0.2, wz=0.8), stop_dist=0.4)
    done = nav.go_to(x_forward=1.2, y_lateral=0.0)
"""

from __future__ import annotations
import math
from dataclasses import dataclass

@dataclass
class Limits:
    vx: float; vy: float; wz: float

class Navigator:
    """最小导航：给定目标在 base 坐标 (x,y)，对齐后前进到 stop_dist。"""
    def __init__(self, client, limits: Limits, stop_dist: float=0.35):
        self.client = client
        self.lim = limits
        self.stop_dist = stop_dist

    def go_to(self, x: float, y: float) -> bool:
        # 朝向
        yaw = math.atan2(y, x)
        wz = max(-self.lim.wz, min(self.lim.wz, yaw))
        self.client.go(0.0, 0.0, wz, 0.3)
        # 前进
        dist = math.hypot(x, y)
        if dist <= self.stop_dist:
            self.client.stop(); return True
        vx = max(0.0, min(self.lim.vx, dist - self.stop_dist))
        self.client.go(vx, 0.0, 0.0, 0.5)
        return False
