#!/usr/bin/env python3
# vision/perception/calibration_utils.py
"""
Camera calibration & extrinsics loader.
兼容两种配置格式：
A) 你现有的 camera.yaml:
    K: [[fx,0,cx],[0,fy,cy],[0,0,1]]
    dist: [k1,k2,p1,p2,k3]
    tag_size_m: 0.05
    valid_tag_ids: [0, 1, ...]
    # 可无 T_base_cam
B) 示例 cameras.yaml:
    fx, fy, cx, cy, dist: [...], T_base_cam: 4x4

Functions:
- load_cam_from_yaml(path) -> (cam_cfg, K, dist, T_base_cam)
- load_tag_from_yaml(camera_yaml, apriltags_yaml=None)
"""

from __future__ import annotations
import yaml
import numpy as np
from pathlib import Path
from typing import Dict, Any, Tuple, List, Optional

def _default_T() -> np.ndarray:
    T = np.eye(4, dtype=float)
    # 如未提供外参，默认相机位于 base 前方 0.15m、高 0.9m（占位用）
    T[:3, 3] = [0.15, 0.0, 0.90]
    return T

def load_cam_from_yaml(path: str) -> Tuple[Dict[str, Any], np.ndarray, np.ndarray, np.ndarray]:
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError(f"camera yaml not found: {p}")
    cfg = yaml.safe_load(p.read_text(encoding="utf-8"))

    # 格式 A：含 K
    if "K" in cfg:
        K = np.array(cfg["K"], dtype=float)
        dist = np.array(cfg.get("dist", [0,0,0,0,0]), dtype=float).reshape(-1)
        T_base_cam = np.array(cfg.get("T_base_cam", _default_T()), dtype=float)
        return cfg, K, dist, T_base_cam

    # 格式 B：fx/fy/cx/cy
    fx, fy, cx, cy = cfg["fx"], cfg["fy"], cfg["cx"], cfg["cy"]
    K = np.array([[fx,0,cx],[0,fy,cy],[0,0,1]], dtype=float)
    dist = np.array(cfg.get("dist", [0,0,0,0,0]), dtype=float).reshape(-1)
    T_base_cam = np.array(cfg.get("T_base_cam", _default_T()), dtype=float)
    return cfg, K, dist, T_base_cam

def load_tag_from_yaml(camera_yaml: str, apriltags_yaml: Optional[str]=None) -> Tuple[str, float, List[int]]:
    """返回 (family, tag_size_m, valid_tag_ids)。优先 apriltags.yaml，否则从 camera.yaml 里取。"""
    family = "tag36h11"
    size = 0.05
    valid = [0]

    if apriltags_yaml and Path(apriltags_yaml).exists():
        cfg = yaml.safe_load(Path(apriltags_yaml).read_text(encoding="utf-8"))
        family = cfg.get("family", family)
        size = float(cfg.get("size_m", size))
        if "ids" in cfg and isinstance(cfg["ids"], dict):
            valid = [int(k) for k in cfg["ids"].keys()]
        return family, size, valid

    # 从 camera.yaml 读取（你现有文件）
    cam_cfg = yaml.safe_load(Path(camera_yaml).read_text(encoding="utf-8"))
    size = float(cam_cfg.get("tag_size_m", size))
    valid = cam_cfg.get("valid_tag_ids", valid)
    return family, size, valid
