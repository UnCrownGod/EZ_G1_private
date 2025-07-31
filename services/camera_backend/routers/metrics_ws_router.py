# services/camera_backend/routers/monitor.py
"""
监控数据推送接口（WebSocket）。
"""

import asyncio

from fastapi import APIRouter, WebSocket
from sqlalchemy.orm import Session

from .. import models
from ..db import get_db
from ..metrics_utils import get_metrics

router = APIRouter()


@router.websocket("/ws/devices/{dev_id}/metrics")
async def metrics_ws(
    ws: WebSocket, dev_id: int, sample_frames: int = 60
) -> None:
    """实时推送设备监控指标。

    Args:
        ws (WebSocket): WebSocket 连接。
        dev_id (int): 设备 ID。
        sample_frames (int): 抽样帧数。

    Raises:
        WebSocketClose: 如果设备不存在，则关闭连接。
    """
    await ws.accept()
    db: Session = next(get_db())
    device = db.query(models.Device).get(dev_id)
    if not device:
        await ws.close(code=1008)
        return

    try:
        while True:
            metrics = get_metrics(device.url, sample_frames)
            await ws.send_json(metrics)
            await asyncio.sleep(1)
    finally:
        db.close()
