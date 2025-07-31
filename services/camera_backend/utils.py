# services/camera_backend/utils.py
"""
设备在线状态检测工具。
"""

from __future__ import annotations

import socket
from datetime import datetime, timezone
from typing import Any, Dict
from urllib.parse import urlparse


def _tcp_ping(host: str, port: int, timeout: float) -> bool:
    """简单的 TCP 连接探测。

    Args:
        host (str): 目标主机地址。
        port (int): 目标端口号。
        timeout (float): 超时时间，单位秒。

    Returns:
        bool: 在指定超时时间内能建立 TCP 连接返回 True，否则返回 False。
    """
    try:
        with socket.create_connection((host, port), timeout=timeout):
            return True
    except OSError:
        return False


def check_device_online(device: Dict[str, Any], timeout: float = 2.0) -> Dict[str, Any]:
    """检查单个设备的在线状态并记录最近在线时间。

    根据设备协议（ONVIF 或 RTSP）进行 TCP 探测，并在探测成功时记录当前 UTC 时间。

    Args:
        device (Dict[str, Any]): 包含以下字段的字典：
            - id (int): 设备唯一标识。
            - name (str): 设备名称。
            - protocol (str): 协议类型，"onvif" 或 "rtsp"。
            - url (str): 对应协议的连接地址，ONVIF 为 "IP:PORT"，RTSP 为 URL。
        timeout (float): TCP 探测超时时间，单位秒。

    Returns:
        Dict[str, Any]: 包含以下键：
            - id (int): 设备 ID。
            - name (str): 设备名称。
            - protocol (str): 协议类型。
            - url (str): 连接地址。
            - online (bool): 是否在线。
            - last_seen (str | None): 最近在线时间的 ISO 8601 UTC 字符串；如果未在线则为 None。
    """
    protocol = device.get("protocol", "").lower()
    online = False
    last_seen: str | None = None

    if protocol == "onvif":
        # ONVIF 格式: "IP:PORT"
        try:
            ip, port_str = device["url"].split(":")
            port = int(port_str)
        except (KeyError, ValueError):
            online = False
        else:
            online = _tcp_ping(ip, port, timeout)
    elif protocol == "rtsp":
        # RTSP 格式: "rtsp://hostname:port/..."
        parsed = urlparse(device.get("url", ""))
        host = parsed.hostname or ""
        port = parsed.port or 554
        online = _tcp_ping(host, port, timeout)
    else:
        # 未知协议，视为离线
        online = False

    if online:
        last_seen = datetime.now(timezone.utc).isoformat()

    return {
        "id": device.get("id"),
        "name": device.get("name"),
        "protocol": device.get("protocol"),
        "url": device.get("url"),
        "online": online,
        "last_seen": last_seen,
    }
