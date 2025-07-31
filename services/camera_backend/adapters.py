# services/camera_backend/adapters.py
"""
协议适配器：封装 ONVIF/RTSP 设备控制逻辑。
"""

import os
import cv2
from datetime import datetime, timezone

from . import models


class BaseAdapter:
    """设备协议适配器基类。"""

    def __init__(self, device: models.Device):
        """初始化适配器。

        Args:
            device (models.Device): 设备 ORM 实例。
        """
        self.device = device

    def capture_snapshot(self) -> str:
        """抓取单帧并保存 JPEG，返回文件路径。"""
        raise NotImplementedError


class RtspAdapter(BaseAdapter):
    """RTSP 协议适配器。"""

    def capture_snapshot(self) -> str:
        cap = cv2.VideoCapture(self.device.url)
        ret, frame = cap.read()
        cap.release()
        if not ret:
            raise RuntimeError("Failed to capture frame from RTSP stream")
        ts = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        folder = os.path.join("snapshots", str(self.device.id))
        os.makedirs(folder, exist_ok=True)
        path = os.path.join(folder, f"{ts}.jpg")
        cv2.imwrite(path, frame)
        return path


class OnvifAdapter(BaseAdapter):
    """ONVIF 协议适配器，复用 RTSP 实现。"""

    def capture_snapshot(self) -> str:
        return RtspAdapter(self.device).capture_snapshot()


def get_adapter(device: models.Device) -> BaseAdapter:
    """根据协议返回对应适配器实例。"""
    if device.protocol.lower() == "onvif":
        return OnvifAdapter(device)
    return RtspAdapter(device)
