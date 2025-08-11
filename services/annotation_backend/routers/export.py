"""
导出标注路由：提供将标注数据导出为 JSON、COCO、YOLO 等格式。
"""

import io
import os
import zipfile
from typing import Literal

from fastapi import APIRouter, Depends, HTTPException, Query
from fastapi.responses import JSONResponse, StreamingResponse
from fastapi.encoders import jsonable_encoder
from sqlalchemy.orm import Session

from ..db import get_db
from .. import models, schemas

router = APIRouter()


@router.get(
    "",
    summary="导出指定数据集的标注数据",
)
def export_annotations(
    dataset_id: int,
    format: Literal["json", "coco", "yolo"] = Query(
        "json", description="导出格式：json, coco, yolo"
    ),
    db: Session = Depends(get_db),
):
    """根据指定格式导出数据集的标注数据。

    Args:
        dataset_id (int): 数据集 ID。
        format (str): 导出格式，可选 'json'、'coco'、'yolo'。
        db (Session): 数据库会话。

    Raises:
        HTTPException: 若数据集不存在或格式不支持。

    Returns:
        JSONResponse or StreamingResponse: 根据格式返回 JSON 或 ZIP 文件流。
    """
    ds = db.get(models.Dataset, dataset_id)
    if not ds:
        raise HTTPException(status_code=404, detail="Dataset not found")

    images = (
        db.query(models.Image)
        .filter(models.Image.dataset_id == dataset_id)
        .all()
    )
    annotations = (
        db.query(models.Annotation)
        .filter(models.Annotation.dataset_id == dataset_id)
        .all()
    )

    if format == "json":
        # 使用 jsonable_encoder 确保包含 datetime 在内的数据可被 JSON 序列化
        payload = schemas.ExportJSONOut(
            images=images,
            annotations=annotations,
        )
        content = jsonable_encoder(payload)
        return JSONResponse(content=content)

    elif format == "yolo":
        # 将每张图片的标注转换为 YOLO 格式 txt 并打包成 ZIP
        zip_buffer = io.BytesIO()
        with zipfile.ZipFile(zip_buffer, "w") as zf:
            for img in images:
                img_anns = [ann for ann in annotations if ann.image_id == img.id]
                lines = []
                for ann in img_anns:
                    cls_idx = ann.label_id - 1
                    x_center = (ann.xmin + ann.xmax) / 2 / ann.width
                    y_center = (ann.ymin + ann.ymax) / 2 / ann.height
                    w = (ann.xmax - ann.xmin) / ann.width
                    h = (ann.ymax - ann.ymin) / ann.height
                    lines.append(f"{cls_idx} {x_center:.6f} {y_center:.6f} {w:.6f} {h:.6f}")
                txt_name = os.path.splitext(os.path.basename(img.file_path))[0] + ".txt"
                zf.writestr(txt_name, "\n".join(lines))
        zip_buffer.seek(0)
        return StreamingResponse(
            zip_buffer,
            media_type="application/zip",
            headers={
                "Content-Disposition": f"attachment; filename=dataset_{dataset_id}_yolo.zip"
            },
        )

    elif format == "coco":
        # COCO 格式尚未实现
        raise HTTPException(status_code=501, detail="COCO export not implemented")

    else:
        raise HTTPException(status_code=400, detail="Unsupported export format")
