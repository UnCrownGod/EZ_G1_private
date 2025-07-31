# services/camera_backend/network.py
"""
网络发现工具：扫描子网内指定端口，返回可达 IP 列表。
"""

import socket
from typing import List


def discover_network(
    prefix: str = "192.168.1.",
    port: int = 554,
    timeout: float = 0.5,
) -> List[str]:
    """扫描给定子网中指定端口。

    Args:
        prefix (str): 子网前缀，不含最后一段，例如 "192.168.1."。
        port (int): 要扫描的端口号。
        timeout (float): 单次 TCP 连接超时时间（秒）。

    Returns:
        List[str]: 存活的 IP 地址列表。
    """
    alive: List[str] = []
    for i in range(1, 255):
        ip = f"{prefix}{i}"
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as sock:
            sock.settimeout(timeout)
            try:
                sock.connect((ip, port))
                alive.append(ip)
            except Exception:
                continue
    return alive
