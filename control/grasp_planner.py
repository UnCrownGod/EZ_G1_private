# File: vision/control/grasp_planner.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Flat-surface grasp planner (placeholder).
平面抓取规划（示例）：给定目标点（base系），生成接近点与抓取点（高度偏置）。

API:
    p_app, p_grasp = plan_flat_grasp(p_target_base, approach_height=0.1)

Note:
    后续可接 MoveIt/手眼标定/夹爪学抓等模块替换。
"""

from __future__ import annotations
import numpy as np

def plan_flat_grasp(p_base: np.ndarray, approach_height: float=0.10):
    """
    输入: 抓取点在 base (x,y,0) 上方。
    输出: 进近位姿(上方 approach) 与着陆位姿(grasp)（仅位置，朝向后续接 SDK 臂控补全）。
    """
    p_grasp = np.array([p_base[0], p_base[1], 0.0])
    p_approach = p_grasp + np.array([0.0, 0.0, approach_height])
    return p_approach, p_grasp
