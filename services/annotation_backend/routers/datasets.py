"""
数据集管理路由：提供数据集的创建、查询、更新、删除及状态编辑。
"""

from typing import List

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session

from ..db import get_db
from .. import models, schemas

router = APIRouter()

@router.get(
    "",
    response_model=List[schemas.DatasetOut],
    summary="获取所有数据集",
)
def list_datasets(
    db: Session = Depends(get_db),
) -> List[schemas.DatasetOut]:
    """列出所有数据集。

    Args:
        db (Session): 数据库会话。

    Returns:
        List[schemas.DatasetOut]: 数据集列表。
    """
    return db.query(models.Dataset).all()


@router.post(
    "",
    response_model=schemas.DatasetOut,
    status_code=status.HTTP_201_CREATED,
    summary="创建新数据集",
)
def create_dataset(
    body: schemas.DatasetCreate,
    db: Session = Depends(get_db),
) -> schemas.DatasetOut:
    """新增一个数据集。

    Args:
        body (schemas.DatasetCreate): 数据集基本信息。
        db (Session): 数据库会话。

    Returns:
        schemas.DatasetOut: 创建成功的数据集信息。
    """
    ds = models.Dataset(
        name=body.name,
        description=body.description,
        status=body.status or models.DatasetStatus.DRAFT,
    )
    db.add(ds)
    db.commit()
    db.refresh(ds)
    return ds


@router.get(
    "/{dataset_id}",
    response_model=schemas.DatasetOut,
    summary="获取指定数据集详情",
)
def get_dataset(
    dataset_id: int,
    db: Session = Depends(get_db),
) -> schemas.DatasetOut:
    """查询单个数据集信息。

    Args:
        dataset_id (int): 数据集 ID。
        db (Session): 数据库会话。

    Raises:
        HTTPException: 若数据集不存在，抛出 404。

    Returns:
        schemas.DatasetOut: 数据集详细信息。
    """
    ds = db.get(models.Dataset,dataset_id)
    if not ds:
        raise HTTPException(status_code=404, detail="Dataset not found")
    return ds


@router.put(
    "/{dataset_id}",
    response_model=schemas.DatasetOut,
    summary="更新指定数据集",
)
def update_dataset(
    dataset_id: int,
    body: schemas.DatasetUpdate,
    db: Session = Depends(get_db),
) -> schemas.DatasetOut:
    """更新数据集基本信息（名称、描述等）。

    Args:
        dataset_id (int): 数据集 ID。
        body (schemas.DatasetUpdate): 可更新字段。
        db (Session): 数据库会话。

    Raises:
        HTTPException: 若数据集不存在，抛出 404。

    Returns:
        schemas.DatasetOut: 更新后数据集信息。
    """
    ds = db.get(models.Dataset,dataset_id)
    if not ds:
        raise HTTPException(status_code=404, detail="Dataset not found")

    for attr, val in body.model_dump(exclude_unset=True).items():
        setattr(ds, attr, val)
    db.commit()
    db.refresh(ds)
    return ds


@router.delete(
    "/{dataset_id}",
    status_code=status.HTTP_204_NO_CONTENT,
    summary="删除指定数据集",
)
def delete_dataset(
    dataset_id: int,
    db: Session = Depends(get_db),
) -> None:
    """删除一个数据集及其关联记录。

    Args:
        dataset_id (int): 数据集 ID。
        db (Session): 数据库会话。

    Raises:
        HTTPException: 若数据集不存在，抛出 404。
    """
    ds = db.get(models.Dataset,dataset_id)
    if not ds:
        raise HTTPException(status_code=404, detail="Dataset not found")

    db.delete(ds)
    db.commit()
    return None


@router.put(
    "/{dataset_id}/status",
    response_model=schemas.DatasetOut,
    summary="更新数据集状态",
)
def update_dataset_status(
    dataset_id: int,
    body: schemas.DatasetStatusUpdate,
    db: Session = Depends(get_db),
) -> schemas.DatasetOut:
    """修改数据集状态（如 draft, labeling, completed）。

    Args:
        dataset_id (int): 数据集 ID。
        body (schemas.DatasetStatusUpdate): 新状态信息。
        db (Session): 数据库会话。

    Raises:
        HTTPException: 若数据集不存在，抛出 404。

    Returns:
        schemas.DatasetOut: 状态更新后的数据集信息.
    """
    ds = db.get(models.Dataset,dataset_id)
    if not ds:
        raise HTTPException(status_code=404, detail="Dataset not found")

    ds.status = body.status
    db.commit()
    db.refresh(ds)
    return ds
