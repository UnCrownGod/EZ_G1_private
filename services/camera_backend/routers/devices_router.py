# services/camera_backend/routers/devices_router.py
"""
设备 CRUD 与在线状态、链接测试接口。
"""

import json
from concurrent.futures import ThreadPoolExecutor
from typing import List

from fastapi import APIRouter, Depends, HTTPException, Query, status
from sqlalchemy.orm import Session

from ..db import get_db
from .. import models, schemas, utils

router = APIRouter()


@router.post(
    "",
    response_model=schemas.DeviceOut,
    status_code=status.HTTP_201_CREATED,
)
def create_device(
    body: schemas.DeviceCreate,
    db: Session = Depends(get_db),
) -> schemas.DeviceOut:
    """新增一台摄像设备及其配置。

    Args:
        body (schemas.DeviceCreate): 设备基本信息和完整配置。
        db (Session): 数据库会话。

    Returns:
        schemas.DeviceOut: 新增的设备及其配置信息。
    """
    device = models.Device(
        name=body.name,
        protocol=body.network.protocol,
        url=f"{body.network.protocol}://{body.network.ip}:{body.network.port}",
        config=body.model_dump_json(),
    )
    db.add(device)
    db.commit()
    db.refresh(device)

    # 将 JSON 字符串反序列化为 dict，以满足 Pydantic 模型
    config_dict = json.loads(device.config) if device.config else {}

    return schemas.DeviceOut(
        id=device.id,
        name=device.name,
        protocol=device.protocol,
        url=device.url,
        config=config_dict,
    )


@router.put(
    "/{dev_id}",
    response_model=schemas.DeviceOut,
)
def update_device(
    dev_id: int,
    body: schemas.DeviceCreate,
    db: Session = Depends(get_db),
) -> schemas.DeviceOut:
    """更新指定设备及其配置。

    Args:
        dev_id (int): 设备 ID。
        body (schemas.DeviceCreate): 设备基本信息和完整配置。
        db (Session): 数据库会话。

    Raises:
        HTTPException: 如果设备不存在，则返回 404。

    Returns:
        schemas.DeviceOut: 更新后的设备及其配置信息。
    """
    device = db.query(models.Device).get(dev_id)
    if not device:
        raise HTTPException(status_code=404, detail="Device not found")

    device.name = body.name
    device.protocol = body.network.protocol
    device.url = f"{body.network.protocol}://{body.network.ip}:{body.network.port}"
    device.config = body.model_dump_json()
    db.commit()
    db.refresh(device)

    config_dict = json.loads(device.config) if device.config else {}

    return schemas.DeviceOut(
        id=device.id,
        name=device.name,
        protocol=device.protocol,
        url=device.url,
        config=config_dict,
    )


@router.get(
    "/status",
    response_model=List[schemas.DeviceStatusOut],
    response_model_exclude_none=True,
)
def list_device_status(
    skip_check: bool = Query(
        False, description="跳过实时在线检测，直接返回数据库信息"
    ),
    timeout: float = Query(
        2.0, ge=0.1, le=10.0, description="单设备检测超时时间（秒）"
    ),
    db: Session = Depends(get_db),
) -> List[schemas.DeviceStatusOut]:
    """列出所有设备及其在线状态。

    Args:
        skip_check (bool): 是否跳过实时检测。
        timeout (float): 单设备检测超时时间（秒）。
        db (Session): 数据库会话。

    Returns:
        List[schemas.DeviceStatusOut]: 每项包含 id, name, protocol, url, online, last_seen。
    """
    devices = db.query(models.Device).all()

    if skip_check:
        return [
            schemas.DeviceStatusOut(
                id=d.id,
                name=d.name,
                protocol=d.protocol,
                url=d.url,
            )
            for d in devices
        ]

    payloads = [
        {"id": d.id, "name": d.name, "protocol": d.protocol, "url": d.url}
        for d in devices
    ]
    with ThreadPoolExecutor(max_workers=min(8, len(payloads) or 1)) as executor:
        results = list(executor.map(lambda dev: utils.check_device_online(dev, timeout), payloads))

    return [schemas.DeviceStatusOut(**r) for r in results]


@router.get(
    "/{dev_id}/test",
    response_model=schemas.DeviceStatusOut,
)
def test_device(
    dev_id: int,
    timeout: float = Query(
        2.0, ge=0.1, le=10.0, description="检测超时时间（秒）"
    ),
    db: Session = Depends(get_db),
) -> schemas.DeviceStatusOut:
    """对单台设备执行在线链接测试。

    Args:
        dev_id (int): 设备 ID。
        timeout (float): 单设备检测超时时间（秒）。
        db (Session): 数据库会话。

    Raises:
        HTTPException: 如果设备不存在，则返回 404。

    Returns:
        schemas.DeviceStatusOut: 测试结果，包含 online、last_seen。
    """
    device = db.query(models.Device).get(dev_id)
    if not device:
        raise HTTPException(status_code=404, detail="Device not found")

    payload = {
        "id": device.id,
        "name": device.name,
        "protocol": device.protocol,
        "url": device.url,
    }
    result = utils.check_device_online(payload, timeout)
    return schemas.DeviceStatusOut(**result)
