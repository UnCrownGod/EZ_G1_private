# services/camera_backend/app.py
"""
Camera Backend 服务：在线状态和参数管理接口。
"""

from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from datetime import datetime, timezone
from typing import Generator, List

from fastapi import Body, Depends, FastAPI, HTTPException, Query, Response, status
from sqlalchemy.orm import Session

from . import models, utils
from . import schemas

app = FastAPI(
    title="Camera Backend",
    version="0.1.0",
    description="提供摄像设备在线状态和参数管理功能",
)


def get_db() -> Generator[Session, None, None]:
    """创建并返回数据库会话，使用后自动关闭。"""
    db = models.SessionLocal()
    try:
        yield db
    finally:
        db.close()


@app.on_event("startup")
def _startup() -> None:
    """应用启动时确保数据库表存在。"""
    models.init_db()


@app.get("/")
async def root() -> dict:
    """服务健康检查。"""
    return {"msg": "Camera Backend is running!"}


@app.get(
    "/devices/status",
    response_model=List[schemas.DeviceStatusOut],
    response_model_exclude_none=True,
)
def list_device_status(
    skip_check: bool = Query(False, description="跳过实时在线检测，直接返回数据库信息"),
    timeout: float = Query(2.0, ge=0.1, le=10.0, description="单设备检测超时时间（秒）"),
    db: Session = Depends(get_db),
) -> List[schemas.DeviceStatusOut]:
    """列出所有设备及其在线状态。

    Args:
        skip_check (bool): 是否跳过实时检测。
        timeout (float): 单设备检测的超时（秒）。
        db (Session): 数据库会话。

    Returns:
        List[DeviceStatusOut]: 每项包含 id, name, protocol, url, online, last_seen。
    """
    devices = db.query(models.Device).all()

    if skip_check:
        # 不检测，直接返回基本信息
        return [
            schemas.DeviceStatusOut(
                id=d.id,
                name=d.name,
                protocol=d.protocol,
                url=d.url,
                online=None,
                last_seen=None,
            )
            for d in devices
        ]

    payloads = [
        {"id": d.id, "name": d.name, "protocol": d.protocol, "url": d.url}
        for d in devices
    ]

    # 并发检测，避免串行阻塞
    with ThreadPoolExecutor(max_workers=min(8, len(payloads) or 1)) as ex:
        results = list(ex.map(lambda dev: utils.check_device_online(dev, timeout), payloads))

    return [schemas.DeviceStatusOut(**r) for r in results]


@app.get(
    "/devices/{dev_id}/params",
    response_model=List[schemas.DeviceParamOut],
)
def get_params(dev_id: int, db: Session = Depends(get_db)) -> List[schemas.DeviceParamOut]:
    """获取指定设备的所有参数。

    Args:
        dev_id (int): 设备 ID。
        db (Session): 数据库会话。

    Returns:
        List[DeviceParamOut]: 参数对象列表。
    """
    return db.query(models.DeviceParam).filter_by(device_id=dev_id).all()


@app.post(
    "/devices/{dev_id}/params",
    response_model=schemas.DeviceParamOut,
    status_code=status.HTTP_201_CREATED,
)
def add_param(
    dev_id: int,
    body: schemas.DeviceParamCreate = Body(...),
    db: Session = Depends(get_db),
) -> schemas.DeviceParamOut:
    """新增设备参数。

    Args:
        dev_id (int): 设备 ID。
        body (DeviceParamCreate): 参数名与值。
        db (Session): 数据库会话。

    Returns:
        DeviceParamOut: 新增的参数实例。
    """
    param = models.DeviceParam(
        device_id=dev_id,
        key=body.key,
        value=body.value,
        updated_at=datetime.now(timezone.utc).isoformat(),
    )
    db.add(param)
    db.commit()
    db.refresh(param)
    return param


@app.put(
    "/devices/{dev_id}/params/{param_id}",
    response_model=schemas.DeviceParamOut,
)
def update_param(
    dev_id: int,
    param_id: int,
    body: schemas.DeviceParamUpdate = Body(...),
    db: Session = Depends(get_db),
) -> schemas.DeviceParamOut:
    """更新设备参数值。

    Args:
        dev_id (int): 设备 ID。
        param_id (int): 参数记录 ID。
        body (DeviceParamUpdate): 新的参数值。
        db (Session): 数据库会话。

    Raises:
        HTTPException: 如果参数不存在，则返回 404。

    Returns:
        DeviceParamOut: 更新后的参数实例。
    """
    param = db.query(models.DeviceParam).filter_by(device_id=dev_id, id=param_id).first()
    if not param:
        raise HTTPException(status_code=404, detail="Param not found")
    param.value = body.value
    param.updated_at = datetime.now(timezone.utc).isoformat()
    db.commit()
    db.refresh(param)
    return param


@app.delete(
    "/devices/{dev_id}/params/{param_id}",
    status_code=status.HTTP_204_NO_CONTENT,
)
def delete_param(dev_id: int, param_id: int, db: Session = Depends(get_db)) -> Response:
    """删除指定设备的参数记录。

    Args:
        dev_id (int): 设备 ID。
        param_id (int): 参数记录 ID。
        db (Session): 数据库会话。

    Raises:
        HTTPException: 如果参数不存在，则返回 404。
    """
    deleted = db.query(models.DeviceParam).filter_by(device_id=dev_id, id=param_id).delete()
    if not deleted:
        raise HTTPException(status_code=404, detail="Param not found")
    db.commit()
    return Response(status_code=status.HTTP_204_NO_CONTENT)
