# services/camera_backend/routers/params.py
"""
设备参数管理 CRUD 接口。
"""

from datetime import datetime, timezone
from typing import List

from fastapi import APIRouter, Body, Depends, HTTPException, status
from sqlalchemy.orm import Session

from .. import models, schemas
from ..db import get_db

router = APIRouter()


@router.get(
    "/{dev_id}/params",
    response_model=List[schemas.DeviceParamOut],
)
def get_params(
    dev_id: int,
    db: Session = Depends(get_db),
) -> List[schemas.DeviceParamOut]:
    """获取指定设备的所有参数。

    Args:
        dev_id (int): 设备 ID。
        db (Session): 数据库会话。

    Returns:
        List[schemas.DeviceParamOut]: 设备参数列表。
    """
    return db.query(models.DeviceParam).filter_by(device_id=dev_id).all()


@router.post(
    "/{dev_id}/params",
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
        body (schemas.DeviceParamCreate): 参数名和值。
        db (Session): 数据库会话。

    Returns:
        schemas.DeviceParamOut: 新增的参数实例。
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


@router.put(
    "/{dev_id}/params/{param_id}",
    response_model=schemas.DeviceParamOut,
)
def update_param(
    dev_id: int,
    param_id: int,
    body: schemas.DeviceParamUpdate = Body(...),
    db: Session = Depends(get_db),
) -> schemas.DeviceParamOut:
    """更新指定设备的参数值。

    Args:
        dev_id (int): 设备 ID。
        param_id (int): 参数 ID。
        body (schemas.DeviceParamUpdate): 新的参数值。
        db (Session): 数据库会话。

    Raises:
        HTTPException: 若参数不存在，则返回 404。

    Returns:
        schemas.DeviceParamOut: 更新后的参数实例。
    """
    param = (
        db.query(models.DeviceParam)
        .filter_by(device_id=dev_id, id=param_id)
        .first()
    )
    if not param:
        raise HTTPException(status_code=404, detail="Param not found")
    param.value = body.value
    param.updated_at = datetime.now(timezone.utc).isoformat()
    db.commit()
    db.refresh(param)
    return param


@router.delete(
    "/{dev_id}/params/{param_id}",
    status_code=status.HTTP_204_NO_CONTENT,
)
def delete_param(
    dev_id: int,
    param_id: int,
    db: Session = Depends(get_db),
) -> None:
    """删除指定设备的参数。

    Args:
        dev_id (int): 设备 ID。
        param_id (int): 参数 ID。
        db (Session): 数据库会话。

    Raises:
        HTTPException: 若参数不存在，则返回 404。
    """
    deleted = (
        db.query(models.DeviceParam)
        .filter_by(device_id=dev_id, id=param_id)
        .delete()
    )
    if not deleted:
        raise HTTPException(status_code=404, detail="Param not found")
    db.commit()
