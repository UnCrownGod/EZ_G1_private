# services/annotation_backend/routers/annotations.py

"""
标注路由：提供标注记录的创建、查询、更新与删除。
"""

from typing import List

from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.orm import Session

from ..db import get_db
from .. import models, schemas

router = APIRouter()


@router.get(
    "",
    response_model=List[schemas.AnnotationOut],
    summary="列出数据集下所有标注记录",
)
def list_annotations(
    dataset_id: int,
    db: Session = Depends(get_db),
) -> List[schemas.AnnotationOut]:
    """获取指定数据集下的所有标注。

    Args:
        dataset_id (int): 数据集 ID。
        db (Session): 数据库会话。

    Raises:
        HTTPException: 若数据集不存在，抛出 404。

    Returns:
        List[schemas.AnnotationOut]: 标注记录列表。
    """
    ds = db.get(models.Dataset, dataset_id)
    if not ds:
        raise HTTPException(status_code=404, detail="Dataset not found")

    anns = (
        db.query(models.Annotation)
        .filter(models.Annotation.dataset_id == dataset_id)
        .all()
    )
    return anns


@router.get(
    "/{annotation_id}",
    response_model=schemas.AnnotationOut,
    summary="获取单条标注记录",
)
def get_annotation(
    dataset_id: int,
    annotation_id: int,
    db: Session = Depends(get_db),
) -> schemas.AnnotationOut:
    """查询指定标注记录。

    Args:
        dataset_id (int): 数据集 ID。
        annotation_id (int): 标注记录 ID。
        db (Session): 数据库会话。

    Raises:
        HTTPException: 若标注不存在或不属于该数据集，抛出 404。

    Returns:
        schemas.AnnotationOut: 标注详情。
    """
    ann = db.get(models.Annotation, annotation_id)
    if not ann or ann.dataset_id != dataset_id:
        raise HTTPException(status_code=404, detail="Annotation not found")
    return ann


@router.post(
    "",
    response_model=schemas.AnnotationOut,
    status_code=status.HTTP_201_CREATED,
    summary="创建新标注记录",
)
def create_annotation(
    dataset_id: int,
    body: schemas.AnnotationCreate,
    db: Session = Depends(get_db),
) -> schemas.AnnotationOut:
    """在指定数据集下新增一条标注记录。

    Args:
        dataset_id (int): 数据集 ID。
        body (schemas.AnnotationCreate): 标注参数。
        db (Session): 数据库会话。

    Raises:
        HTTPException: 若数据集、图片或标签不存在，则抛出 404。

    Returns:
        schemas.AnnotationOut: 创建成功的标注。
    """
    # 校验关联对象
    ds = db.get(models.Dataset,dataset_id)
    img = db.get(models.Image,body.image_id)
    lbl = db.get(models.LabelDefinition,body.label_id)
    if not ds or not img or not lbl \
       or img.dataset_id != dataset_id or lbl.dataset_id != dataset_id:
        raise HTTPException(status_code=404, detail="Dataset/Image/Label not found")

    # 计算宽高
    w = body.xmax - body.xmin
    h = body.ymax - body.ymin

    ann = models.Annotation(
        dataset_id=dataset_id,
        image_id=body.image_id,
        label_id=body.label_id,
        xmin=body.xmin,
        ymin=body.ymin,
        xmax=body.xmax,
        ymax=body.ymax,
        width=w,
        height=h,
    )
    db.add(ann)
    db.commit()
    db.refresh(ann)
    return ann


@router.put(
    "/{annotation_id}",
    response_model=schemas.AnnotationOut,
    summary="更新指定标注记录",
)
def update_annotation(
    dataset_id: int,
    annotation_id: int,
    body: schemas.AnnotationUpdate,
    db: Session = Depends(get_db),
) -> schemas.AnnotationOut:
    """修改指定标注记录的属性。

    Args:
        dataset_id (int): 数据集 ID。
        annotation_id (int): 标注记录 ID。
        body (schemas.AnnotationUpdate): 待更新字段。
        db (Session): 数据库会话.

    Raises:
        HTTPException: 若标注不存在或不属于该数据集，抛出 404。

    Returns:
        schemas.AnnotationOut: 更新后的标注信息。
    """
    ann = db.get(models.Annotation,annotation_id)
    if not ann or ann.dataset_id != dataset_id:
        raise HTTPException(status_code=404, detail="Annotation not found")

    # 更新字段
    data = body.model_dump(exclude_unset=True)
    for attr, val in data.items():
        setattr(ann, attr, val)

    # 如果坐标有变更，重新计算宽高
    if any(k in data for k in ("xmin", "ymin", "xmax", "ymax")):
        ann.width = ann.xmax - ann.xmin
        ann.height = ann.ymax - ann.ymin

    db.commit()
    db.refresh(ann)
    return ann


@router.delete(
    "/{annotation_id}",
    status_code=status.HTTP_204_NO_CONTENT,
    summary="删除指定标注记录",
)
def delete_annotation(
    dataset_id: int,
    annotation_id: int,
    db: Session = Depends(get_db),
) -> None:
    """删除一条标注记录。

    Args:
        dataset_id (int): 数据集 ID。
        annotation_id (int): 标注记录 ID。
        db (Session): 数据库会话。

    Raises:
        HTTPException: 若标注不存在或不属于该数据集，抛出 404。
    """
    ann = db.get(models.Annotation,annotation_id)
    if not ann or ann.dataset_id != dataset_id:
        raise HTTPException(status_code=404, detail="Annotation not found")

    db.delete(ann)
    db.commit()
    return None
