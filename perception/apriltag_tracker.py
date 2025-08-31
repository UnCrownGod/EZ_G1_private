# File: vision/perception/apriltag_pose.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
AprilTag pose estimation via OpenCV ArUco (DICT_APRILTAG_36h11).
使用 OpenCV ArUco 的 AprilTag36h11 估计位姿：T_cam_tag、角点。

API:
    det = AprilTagPoseCV2(tag_size_m=0.05, family="DICT_APRILTAG_36h11")
    results = det.detect(gray, K, dist, valid_ids=[0,1])

Returns (list of dict):
    {'id': int, 'T_cam_tag': (4,4) np.ndarray, 'corners': (4,2) np.ndarray}

Requires:
    pip install opencv-contrib-python
"""

from __future__ import annotations
from typing import List, Dict, Any, Sequence
import numpy as np

class AprilTagPoseCV2:
    def __init__(self, tag_size_m: float=0.05, family: str="DICT_APRILTAG_36h11"):
        try:
            import cv2  # noqa
            from cv2 import aruco  # noqa
        except Exception as e:
            raise ImportError("需要 opencv-contrib-python，安装：pip install opencv-contrib-python") from e
        import cv2
        from cv2 import aruco
        if not hasattr(aruco, family):
            raise ValueError(f"OpenCV ArUco 不支持字典 {family}")
        self.cv2 = cv2
        self.aruco = aruco
        self.dict = getattr(aruco, family)
        self.board = aruco.getPredefinedDictionary(self.dict)
        self.tag_size = float(tag_size_m)

    def detect(self, gray: np.ndarray, K: np.ndarray, dist: np.ndarray, valid_ids: Sequence[int] = ()) -> List[Dict[str, Any]]:
        aruco = self.aruco
        corners, ids, _ = aruco.detectMarkers(gray, self.board)
        out = []
        if ids is None or len(ids)==0:
            return out

        ids = ids.reshape(-1)
        # 单个 tag 的“边长”用 square length；这里用 estimatePoseSingleMarkers
        rvecs, tvecs, _obj = aruco.estimatePoseSingleMarkers(corners, self.tag_size, K, dist)
        for i, tag_id in enumerate(ids):
            if valid_ids and int(tag_id) not in valid_ids:
                continue
            rvec, tvec = rvecs[i].reshape(-1), tvecs[i].reshape(-1)
            R, _ = self.cv2.Rodrigues(rvec)
            T = np.eye(4, dtype=float)
            T[:3, :3] = R
            T[:3, 3]  = tvec
            c = np.array(corners[i]).reshape(4,2)
            out.append({"id": int(tag_id), "T_cam_tag": T, "corners": c})
        return out
