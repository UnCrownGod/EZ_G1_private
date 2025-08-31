# File: vision/control/aligner.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Visual align controller (pixel PID → vy/wz).
视觉对准控制器：根据目标在图像中的横向偏差产生侧移/转向指令（PID 简化版）。

API:
    vy, wz = aligner.cmd(img_w, img_h, target_x)
"""

from __future__ import annotations

class Aligner:
    """基于图像中心误差的横摆/侧移对准（简化）。"""
    def __init__(self, kp_yaw=1.2, kp_lat=0.8, lim_wz=0.8, lim_vy=0.2):
        self.kp_yaw = kp_yaw; self.kp_lat = kp_lat
        self.lim_wz = lim_wz; self.lim_vy = lim_vy

    def cmd(self, img_w: int, img_h: int, target_px_x: float):
        cx = img_w/2
        ex = (target_px_x - cx)/cx  # -1..1
        wz = max(-self.lim_wz, min(self.lim_wz, -self.kp_yaw*ex))
        vy = max(-self.lim_vy, min(self.lim_vy, -self.kp_lat*ex))
        return vy, wz
