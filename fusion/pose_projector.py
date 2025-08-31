# File: vision/fusion/pose_projector.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Project 2D detections to 3D (ground-plane intersection).
将 2D bbox 中心转为相机射线，并与地面 z=const 求交 → 基座/世界坐标。

Functions:
    bbox_center_to_ray(bbox, K) -> (ray_cam)
    intersect_with_ground(ray_cam, T_base_cam, ground_z_in_base=0.)
"""

from __future__ import annotations
import numpy as np
from typing import Dict, Any

def bbox_center_to_ray(bbox, K):
    x1,y1,x2,y2 = bbox
    u = (x1+x2)/2.; v = (y1+y2)/2.
    Kinv = np.linalg.inv(K)
    pt = Kinv @ np.array([u,v,1.0])
    ray = pt/np.linalg.norm(pt)
    return ray

def intersect_with_ground(ray_cam: np.ndarray, T_base_cam: np.ndarray, ground_z_in_base: float=0.0):
    # cam origin in base:
    o_base = T_base_cam[:3,3]
    # ray dir in base:
    d_base = T_base_cam[:3,:3] @ ray_cam
    if abs(d_base[2]) < 1e-6:
        return None
    t = (ground_z_in_base - o_base[2]) / d_base[2]
    if t <= 0:  # looking up or behind
        return None
    p = o_base + t*d_base
    return p  # (x,y,z) in base
