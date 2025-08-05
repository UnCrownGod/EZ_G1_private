# services/annotation_backend/routers/labels.py

"""
标签定义路由：提供数据集下标签（类别）及其配置的 CRUD。
"""

from typing import List

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session

from ..db import get_db
from .. import models, schemas

router = APIRouter()


@router.get(
    "",
    response_model=List[schemas.LabelOut],
    summary="获取数据集标签列表",
)
def list_labels(
    dataset_id: int,
    db: Session = Depends(get_db),
) -> List[schemas.LabelOut]:
    """列出指定数据集下所有标签定义。

    Args:
        dataset_id (int): 数据集 ID。
        db (Session): 数据库会话。

    Raises:
        HTTPException: 若数据集不存在，抛出 404。

    Returns:
        List[schemas.LabelOut]: 标签定义列表。
    """
    ds = db.get(models.Dataset, dataset_id)
    if not ds:
        raise HTTPException(status_code=404, detail="Dataset not found")

    labels = (
        db.query(models.LabelDefinition)
        .filter(models.LabelDefinition.dataset_id == dataset_id)
        .all()
    )
    return labels


@router.post(
    "",
    response_model=schemas.LabelOut,
    status_code=status.HTTP_201_CREATED,
    summary="创建新标签",
)
def create_label(
    dataset_id: int,
    body: schemas.LabelCreate,
    db: Session = Depends(get_db),
) -> schemas.LabelOut:
    """在指定数据集下新增一个标签。

    Args:
        dataset_id (int): 数据集 ID。
        body (schemas.LabelCreate): 标签名称及可选配色。
        db (Session): 数据库会话。

    Raises:
        HTTPException: 若数据集不存在，抛出 404。

    Returns:
        schemas.LabelOut: 创建成功的标签信息。
    """
    ds = db.get(models.Dataset, dataset_id)
    if not ds:
        raise HTTPException(status_code=404, detail="Dataset not found")

    label = models.LabelDefinition(
        dataset_id=dataset_id,
        name=body.name,
        color=body.color,
    )
    db.add(label)
    db.commit()
    db.refresh(label)
    return label


@router.put(
    "/{label_id}",
    response_model=schemas.LabelOut,
    summary="更新指定标签",
)
def update_label(
    dataset_id: int,
    label_id: int,
    body: schemas.LabelUpdate,
    db: Session = Depends(get_db),
) -> schemas.LabelOut:
    """更新指定标签的名称或配色。

    Args:
        dataset_id (int): 数据集 ID。
        label_id (int): 标签 ID。
        body (schemas.LabelUpdate): 待更新字段。
        db (Session): 数据库会话。

    Raises:
        HTTPException: 若标签不存在，抛出 404。

    Returns:
        schemas.LabelOut: 更新后的标签信息。
    """
    label = (
        db.query(models.LabelDefinition)
        .filter(
            models.LabelDefinition.id == label_id,
            models.LabelDefinition.dataset_id == dataset_id,
        )
        .first()
    )
    if not label:
        raise HTTPException(status_code=404, detail="Label not found")

    for attr, val in body.model_dump(exclude_unset=True).items():
        setattr(label, attr, val)
    db.commit()
    db.refresh(label)
    return label


@router.delete(
    "/{label_id}",
    status_code=status.HTTP_204_NO_CONTENT,
    summary="删除指定标签",
)
def delete_label(
    dataset_id: int,
    label_id: int,
    db: Session = Depends(get_db),
) -> None:
    """删除指定标签。

    Args:
        dataset_id (int): 数据集 ID。
        label_id (int): 标签 ID。
        db (Session): 数据库会话。

    Raises:
        HTTPException: 若标签不存在，抛出 404。
    """
    label = (
        db.query(models.LabelDefinition)
        .filter(
            models.LabelDefinition.id == label_id,
            models.LabelDefinition.dataset_id == dataset_id,
        )
        .first()
    )
    if not label:
        raise HTTPException(status_code=404, detail="Label not found")

    db.delete(label)
    db.commit()
    return None
