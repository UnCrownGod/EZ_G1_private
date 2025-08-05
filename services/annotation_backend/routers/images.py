# services/annotation_backend/routers/images.py

"""
图片管理路由：提供数据集下图片的批量导入与列表查询。
"""

from datetime import datetime
from typing import List

from fastapi import APIRouter, Depends, HTTPException, UploadFile, File, status
from sqlalchemy.orm import Session

from ..db import get_db
from ..utils import save_upload_file
from .. import models, schemas

router = APIRouter()


@router.post(
    "/import",
    response_model=List[schemas.ImageOut],
    status_code=status.HTTP_201_CREATED,
    summary="批量导入图片",
)
async def import_images(
    dataset_id: int,
    files: List[UploadFile] = File(..., description="待导入的图片文件列表"),
    db: Session = Depends(get_db),
) -> List[schemas.ImageOut]:
    """将多个图片文件保存到存储目录，并在数据库中创建对应记录。

    Args:
        dataset_id (int): 所属数据集 ID。
        files (List[UploadFile]): 上传的图片文件列表。
        db (Session): 数据库会话。

    Raises:
        HTTPException: 若指定的数据集不存在，则返回 404。

    Returns:
        List[schemas.ImageOut]: 已导入图片的详细信息列表。
    """
    # 验证数据集存在
    ds = db.get(models.Dataset, dataset_id)
    if not ds:
        raise HTTPException(status_code=404, detail="Dataset not found")

    imported: List[models.Image] = []
    for upload in files:
        # 调用统一的异步写入工具
        file_path = await save_upload_file(dataset_id, upload)

        # 在数据库中记录
        img = models.Image(
            dataset_id=dataset_id,
            file_path=file_path,
            created_at=datetime.utcnow(),
        )
        db.add(img)
        db.commit()
        db.refresh(img)
        imported.append(img)

    return imported


@router.get(
    "",
    response_model=List[schemas.ImageOut],
    summary="列出数据集下所有图片",
)
def list_images(
    dataset_id: int,
    db: Session = Depends(get_db),
) -> List[schemas.ImageOut]:
    """获取指定数据集下的所有图片记录。

    Args:
        dataset_id (int): 数据集 ID。
        db (Session): 数据库会话。

    Raises:
        HTTPException: 若指定的数据集不存在，则返回 404。

    Returns:
        List[schemas.ImageOut]: 图片信息列表，包含 id、dataset_id、file_path、created_at 等字段。
    """
    # 验证数据集存在
    ds = db.get(models.Dataset, dataset_id)
    if not ds:
        raise HTTPException(status_code=404, detail="Dataset not found")

    images = (
        db.query(models.Image)
        .filter(models.Image.dataset_id == dataset_id)
        .order_by(models.Image.created_at.desc())
        .all()
    )
    return images
