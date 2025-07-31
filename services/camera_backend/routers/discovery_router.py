# services/camera_backend/routers/discovery_router.py
"""
网络发现：扫描给定 IP 前缀下所有主机在指定端口的在线状态。
"""

from concurrent.futures import ThreadPoolExecutor, as_completed
from typing import List

from fastapi import APIRouter, Depends, Query
from sqlalchemy.orm import Session

from ..db import get_db

router = APIRouter()


@router.get(
    "/network",
    response_model=List[str],
    summary="同网络设备发现",
)
def discover_devices(
    prefix: str = Query(..., description="IP 前缀，例如 '192.168.1.'"),
    port: int = Query(554, ge=1, le=65535, description="检测端口"),
    timeout: float = Query(0.5, ge=0.01, le=5.0, description="单次连接超时（秒）"),
    max_workers: int = Query(50, ge=1, le=200, description="并发线程数"),
    db: Session = Depends(get_db),
) -> List[str]:
    """扫描同子网中指定端口是否在线。

    Args:
        prefix (str): IP 前缀，如 '127.0.0.'。
        port (int): 端口号。
        timeout (float): 每个 IP 的连接超时。
        max_workers (int): 最大并发线程数。
        db (Session): 数据库会话（目前未使用，可用于后续保存或过滤）。

    Returns:
        List[str]: 所有在线设备的 IP 列表。
    """
    import socket

    def probe(ip: str) -> str | None:
        """尝试 TCP 连接，成功返回 IP，否则返回 None。"""
        try:
            with socket.create_connection((ip, port), timeout=timeout):
                return ip
        except OSError:
            return None

    # 构造待测 IP 列表 1–254
    targets = [f"{prefix}{i}" for i in range(1, 255)]
    online: List[str] = []

    # 并发执行 probe
    with ThreadPoolExecutor(max_workers=max_workers) as pool:
        futures = {pool.submit(probe, ip): ip for ip in targets}
        for fut in as_completed(futures):
            result = fut.result()
            if result:
                online.append(result)

    return online
