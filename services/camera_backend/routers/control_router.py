# services/camera_backend/routers/control.py
"""
设备控制接口（打开/关闭/拍照/录像）。
"""

from fastapi import APIRouter, Body, Depends, HTTPException
from sqlalchemy.orm import Session

from .. import adapters, models, schemas
from ..db import get_db

router = APIRouter()


@router.post(
    "/{dev_id}/control",
    response_model=schemas.ControlOut,
)
def control_device(
    dev_id: int,
    body: schemas.ControlIn = Body(...),
    db: Session = Depends(get_db),
) -> schemas.ControlOut:
    """执行设备远程控制指令。

    Args:
        dev_id (int): 设备 ID。
        body (schemas.ControlIn): 控制请求体，指定 action。
        db (Session): 数据库会话。

    Raises:
        HTTPException: 设备不存在或指令执行失败。

    Returns:
        schemas.ControlOut: 操作结果或文件路径。
    """
    device = db.query(models.Device).get(dev_id)
    if not device:
        raise HTTPException(status_code=404, detail="Device not found")

    adaptor = adapters.get_adapter(device)
    if body.action == "snapshot":
        try:
            path = adaptor.capture_snapshot()
            return schemas.ControlOut(action="snapshot", detail=path)
        except Exception as e:
            raise HTTPException(status_code=500, detail=str(e))

    raise HTTPException(status_code=400, detail=f"Unsupported action '{body.action}'")
