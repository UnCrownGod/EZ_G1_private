#!/usr/bin/env python3
# robot/perception/apriltag_pose.py
"""
AprilTag pose estimation via OpenCV ArUco (DICT_APRILTAG_36h11).
使用 OpenCV ArUco 的 AprilTag36h11 估计位姿。

Requires:
    pip install opencv-contrib-python
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple
import numpy as np
import cv2


@dataclass
class CameraModel:
    """Camera intrinsics & distortion.

    相机内参与畸变。

    Args:
        K (np.ndarray): 3x3 camera matrix.
        dist (np.ndarray): Distortion coefficients (len 4,5,8, or 12).

    Raises:
        ValueError: If shapes are invalid.
    """
    K: np.ndarray
    dist: np.ndarray

    def __post_init__(self) -> None:
        if self.K.shape != (3, 3):
            raise ValueError("K must be 3x3")
        if self.dist.ndim != 1:
            raise ValueError("dist must be 1-D")


def rvec_tvec_to_T(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """Convert OpenCV rvec/tvec to 4x4 SE(3).

    将 OpenCV 的 rvec/tvec 转为 4x4 位姿矩阵。

    Args:
        rvec (np.ndarray): 3x1 rotation vector (Rodrigues).
        tvec (np.ndarray): 3x1 translation vector (meters).

    Returns:
        np.ndarray: 4x4 homogeneous transform.

    Raises:
        ValueError: If inputs have invalid shapes.
    """
    if rvec.size != 3 or tvec.size != 3:
        raise ValueError("rvec/tvec must be 3 elements")
    R, _ = cv2.Rodrigues(rvec.reshape(3, 1))
    T = np.eye(4, dtype=float)
    T[:3, :3] = R
    T[:3, 3] = tvec.reshape(3)
    return T


def detect_apriltags(
    bgr: np.ndarray,
    cam: CameraModel,
    tag_size_m: float,
    valid_ids: Optional[List[int]] = None,
) -> List[Tuple[int, np.ndarray]]:
    """Detect AprilTags and estimate per-tag pose in camera frame.

    检测 AprilTag 并输出相机坐标系下位姿。

    Args:
        bgr (np.ndarray): Input BGR image.
        cam (CameraModel): Camera model (K, dist).
        tag_size_m (float): Tag edge length in meters.
        valid_ids (Optional[List[int]]): If provided, only keep these IDs.

    Returns:
        List[Tuple[int, np.ndarray]]: List of (tag_id, T_cam_tag).

    Raises:
        ValueError: If image or tag size is invalid.
    """
    if bgr is None or bgr.size == 0:
        raise ValueError("empty image")
    if tag_size_m <= 0:
        raise ValueError("tag_size_m must be > 0")

    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    aruco = cv2.aruco
    dict_id = aruco.DICT_APRILTAG_36h11
    dictionary = aruco.getPredefinedDictionary(dict_id)
    params = aruco.DetectorParameters_create()
    corners, ids, _ = aruco.detectMarkers(gray, dictionary, parameters=params)
    results: List[Tuple[int, np.ndarray]] = []
    if ids is None:
        return results

    # Estimate pose per marker
    rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
        corners, tag_size_m, cam.K, cam.dist
    )
    for i, tag_id in enumerate(ids.flatten().tolist()):
        if valid_ids and tag_id not in valid_ids:
            continue
        T = rvec_tvec_to_T(rvecs[i], tvecs[i])
        results.append((tag_id, T))
    return results
