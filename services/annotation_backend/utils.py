# services/annotation_backend/utils.py

"""
工具函数：包含文件存储、格式转换与路径管理等公用模块。
"""

import io
import zipfile
from pathlib import Path
from typing import List, Tuple

from fastapi import UploadFile

from .config import settings


def get_dataset_dir(dataset_id: int) -> str:
    """获取并创建数据集对应的本地存储目录。

    Args:
        dataset_id (int): 数据集 ID。

    Returns:
        str: 本地存储目录路径，如 "./data/{dataset_id}"。
    """
    base_dir = Path(settings.storage_dir) / str(dataset_id)
    base_dir.mkdir(parents=True, exist_ok=True)
    return str(base_dir)


async def save_upload_file(dataset_id: int, upload: UploadFile) -> str:
    """保存上传的文件到数据集存储目录，防止路径穿越攻击。

    Args:
        dataset_id (int): 数据集 ID。
        upload (UploadFile): FastAPI 上传文件对象。

    Returns:
        str: 文件保存后的绝对路径。
    """
    # 1. 异步读取所有内容
    content = await upload.read()

    # 2. 确保数据集目录存在
    dataset_dir = Path(settings.storage_dir) / str(dataset_id)
    dataset_dir.mkdir(parents=True, exist_ok=True)

    # 3. 安全取文件名，丢弃任何路径部分
    safe_name = Path(upload.filename).name
    dest_path = dataset_dir / safe_name

    # 4. 将内容写入文件
    dest_path.write_bytes(content)

    return str(dest_path)


def convert_annotations_to_coco(
    images: List[dict],
    annotations: List[dict],
    categories: List[dict]
) -> dict:
    """将标注数据转换为 COCO JSON 格式。

    Args:
        images (List[dict]): 每张图片的基本信息列表（id, file_name, width, height）。
        annotations (List[dict]): 标注列表（id, image_id, category_id, bbox, area, iscrowd）。
        categories (List[dict]): 类别定义列表（id, name, supercategory）。

    Returns:
        dict: 符合 COCO 规范的 JSON 对象。
    """
    return {
        "images": images,
        "annotations": annotations,
        "categories": categories,
        "type": "instances",
        "info": {},
        "licenses": []
    }


def make_zip_bytes(file_paths: List[Tuple[str, bytes]]) -> bytes:
    """将给定文件内容打包为 ZIP 并返回字节流。

    Args:
        file_paths (List[Tuple[str, bytes]]): 
            列表项为 (文件名, 文件内容字节)。

    Returns:
        bytes: ZIP 文件的二进制内容。
    """
    buf = io.BytesIO()
    with zipfile.ZipFile(buf, "w") as zf:
        for name, content in file_paths:
            zf.writestr(name, content)
    buf.seek(0)
    return buf.read()


# TODO: 如有异步任务（Celery/RQ）需求，可在此处添加任务调度相关函数。
