# services/camera_backend/utils.py
"""
设备在线状态检测工具。
"""

from __future__ import annotations

import socket
from datetime import datetime, timezone
from urllib.parse import urlparse


def _tcp_ping(host: str, port: int, timeout: float) -> bool:
    """简单的 TCP 连接探测。

    Args:
        host (str): 目标主机。
        port (int): 目标端口。
        timeout (float): 秒级超时时间。

    Returns:
        bool: 如果能在超时时间内建立连接则为 True，否则 False。
    """
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def check_device_online(device: dict, timeout: float = 2.0) -> dict:
    """检查单个设备在线状态（快速，不阻塞）。

    Args:
        device (dict): 包含以下字段：
            - id (int)
            - name (str)
            - protocol (str): "onvif" 或 "rtsp"
            - url (str): ONVIF "IP:PORT" 或 RTSP URL。
        timeout (float): TCP 探测超时时间（秒）。

    Returns:
        dict: 包含以下字段：
            - id (int)
            - name (str)
            - protocol (str)
            - url (str)
            - online (bool)
            - last_seen (str | None): ISO 8601 UTC 时间
    """
    protocol = device["protocol"].lower()
    online = False
    last_seen: str | None = None

    try:
        if protocol == "onvif":
            ip, port_str = device["url"].split(":")
            port = int(port_str)
            online = _tcp_ping(ip, port, timeout)
        elif protocol == "rtsp":
            parsed = urlparse(device["url"])
            host = parsed.hostname or ""
            port = parsed.port or 554
            online = _tcp_ping(host, port, timeout)
        else:
            # 未知协议，直接判离线
            online = False

        if online:
            last_seen = datetime.now(timezone.utc).isoformat()
    except Exception:
        online = False
        last_seen = None

    return {
        "id": device["id"],
        "name": device["name"],
        "protocol": device["protocol"],
        "url": device["url"],
        "online": online,
        "last_seen": last_seen,
    }
