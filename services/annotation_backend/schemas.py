# services/annotation_backend/schemas.py

"""
Pydantic 模型定义：数据校验与序列化 DTO。
"""

from enum import Enum
from datetime import datetime
from typing import List, Optional

from pydantic import BaseModel, Field

from .models import DatasetStatus


# ----- Dataset Schemas ----- #

class DatasetBase(BaseModel):
    """数据集基础属性。"""
    name: str = Field(..., description="数据集名称")
    description: Optional[str] = Field(None, description="数据集描述")


class DatasetCreate(DatasetBase):
    """创建数据集时使用的请求模型。"""
    status: Optional[DatasetStatus] = Field(
        None, description="初始状态，默认 draft"
    )


class DatasetUpdate(BaseModel):
    """更新数据集基本信息的请求模型。"""
    name: Optional[str] = Field(None, description="数据集名称")
    description: Optional[str] = Field(None, description="数据集描述")


class DatasetStatusUpdate(BaseModel):
    """更新数据集状态的请求模型。"""
    status: DatasetStatus = Field(..., description="新状态：draft、labeling、completed")


class DatasetOut(DatasetBase):
    """数据集响应模型，包含 ID 与状态。"""
    id: int = Field(..., description="数据集 ID")
    status: DatasetStatus = Field(..., description="数据集当前状态")

    class Config:
        orm_mode = True


# ----- Image Schemas ----- #

class ImageOut(BaseModel):
    """图片信息响应模型。"""
    id: int = Field(..., description="图片 ID")
    dataset_id: int = Field(..., description="所属数据集 ID")
    file_path: str = Field(..., description="图片在存储中的路径")
    created_at: datetime = Field(..., description="上传时间")

    class Config:
        orm_mode = True


# ----- Label Schemas ----- #

class LabelBase(BaseModel):
    """标签基础属性。"""
    name: str = Field(..., description="标签名称")
    color: Optional[str] = Field(None, description="标签显示颜色（可选）")


class LabelCreate(LabelBase):
    """创建标签时使用的请求模型。"""
    pass


class LabelUpdate(BaseModel):
    """更新标签的请求模型。"""
    name: Optional[str] = Field(None, description="标签名称")
    color: Optional[str] = Field(None, description="标签显示颜色")


class LabelOut(LabelBase):
    """标签响应模型。"""
    id: int = Field(..., description="标签 ID")
    dataset_id: int = Field(..., description="所属数据集 ID")

    class Config:
        orm_mode = True


# ----- Annotation Schemas ----- #

class AnnotationBase(BaseModel):
    """标注基础属性：边界框参数。"""
    xmin: float = Field(..., ge=0, description="边界框左上角 X 坐标")
    ymin: float = Field(..., ge=0, description="边界框左上角 Y 坐标")
    xmax: float = Field(..., ge=0, description="边界框右下角 X 坐标")
    ymax: float = Field(..., ge=0, description="边界框右下角 Y 坐标")


class AnnotationCreate(AnnotationBase):
    """创建标注时使用的请求模型。"""
    label_id: int = Field(..., description="所属标签 ID")
    image_id: int = Field(..., description="所属图片 ID")


class AnnotationUpdate(BaseModel):
    """更新标注时使用的请求模型。"""
    xmin: Optional[float] = Field(None, ge=0, description="边界框左上角 X 坐标")
    ymin: Optional[float] = Field(None, ge=0, description="边界框左上角 Y 坐标")
    xmax: Optional[float] = Field(None, ge=0, description="边界框右下角 X 坐标")
    ymax: Optional[float] = Field(None, ge=0, description="边界框右下角 Y 坐标")
    label_id: Optional[int] = Field(None, description="所属标签 ID")


class AnnotationOut(AnnotationCreate):
    """标注响应模型，包含 ID 与创建时间。"""
    id: int = Field(..., description="标注 ID")
    created_at: datetime = Field(..., description="创建时间")

    class Config:
        orm_mode = True


# ----- Export DTO ----- #

class ExportJSONOut(BaseModel):
    """JSON 导出格式：包含图片列表与对应标注。"""
    images: List[ImageOut] = Field(..., description="图片列表")
    annotations: List[AnnotationOut] = Field(..., description="标注记录列表")
