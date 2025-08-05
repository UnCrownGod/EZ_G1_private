# services/annotation_backend/routers/status.py

"""
数据集状态路由：提供获取与更新数据集状态。
"""

from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.orm import Session

from ..db import get_db
from .. import models, schemas

router = APIRouter()


@router.get(
    "",
    response_model=schemas.DatasetOut,
    summary="获取数据集详情（含状态）",
)
def get_dataset_status(
    dataset_id: int,
    db: Session = Depends(get_db),
) -> schemas.DatasetOut:
    """查询指定数据集的基本信息与状态。

    Args:
        dataset_id (int): 数据集 ID。
        db (Session): 数据库会话。

    Raises:
        HTTPException: 若数据集不存在，抛出 404。

    Returns:
        schemas.DatasetOut: 数据集信息与当前状态。
    """
    ds = db.get(models.Dataset,dataset_id)
    if not ds:
        raise HTTPException(status_code=404, detail="Dataset not found")
    return ds


@router.put(
    "",
    response_model=schemas.DatasetOut,
    summary="更新数据集状态",
)
def update_dataset_status(
    dataset_id: int,
    body: schemas.DatasetStatusUpdate,
    db: Session = Depends(get_db),
) -> schemas.DatasetOut:
    """修改数据集状态（draft、labeling、completed）。

    Args:
        dataset_id (int): 数据集 ID。
        body (schemas.DatasetStatusUpdate): 新状态。
        db (Session): 数据库会话。

    Raises:
        HTTPException: 若数据集不存在，抛出 404。

    Returns:
        schemas.DatasetOut: 更新后的数据集信息。
    """
    ds = db.get(models.Dataset,dataset_id)
    if not ds:
        raise HTTPException(status_code=404, detail="Dataset not found")

    ds.status = body.status
    db.commit()
    db.refresh(ds)
    return ds
