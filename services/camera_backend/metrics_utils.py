# services/camera_backend/monitor.py
"""
监控数据工具：从视频流抽样计算 fps、分辨率和丢帧数。
"""

import cv2
from typing import Dict


def get_metrics(url: str, sample_frames: int = 60) -> Dict[str, float]:
    """拉取样本帧并计算监控指标。

    Args:
        url (str): RTSP 或 HTTP 直播流地址。
        sample_frames (int): 抽样帧数。

    Returns:
        Dict[str, float]: {
            "fps": 实际帧率,
            "resolution": "宽x高",
            "dropped_frames": 丢帧数
        }
    """
    cap = cv2.VideoCapture(url)
    if not cap.isOpened():
        raise RuntimeError("无法打开视频流进行监控")
    count = 0
    dropped = 0
    while count < sample_frames:
        ret, _ = cap.read()
        if not ret:
            dropped += 1
            continue
        count += 1
    fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH) or 0)
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT) or 0)
    cap.release()
    return {
        "fps": fps,
        "resolution": f"{width}x{height}",
        "dropped_frames": dropped,
    }
