# services/annotation_backend/models.py

"""
ORM 模型定义：包含数据集、图片、标签定义与标注记录。
"""

import enum
from datetime import datetime
from sqlalchemy import (
    Column, Integer, String, Text, Enum, ForeignKey,
    DateTime, Float
)
from sqlalchemy.orm import relationship

from .db import Base


class DatasetStatus(enum.Enum):
    """数据集状态枚举。"""
    DRAFT = "draft"
    LABELING = "labeling"
    COMPLETED = "completed"


class Dataset(Base):
    """数据集表，存储标注项目的基本信息与状态。"""
    __tablename__ = "datasets"

    id = Column(Integer, primary_key=True, index=True)
    name = Column(String(100), unique=True, nullable=False)
    description = Column(Text, nullable=True)
    status = Column(Enum(DatasetStatus), default=DatasetStatus.DRAFT, nullable=False)

    images = relationship(
        "Image",
        back_populates="dataset",
        cascade="all, delete-orphan",
    )
    labels = relationship(
        "LabelDefinition",
        back_populates="dataset",
        cascade="all, delete-orphan",
    )
    annotations = relationship(
        "Annotation",
        back_populates="dataset",
        cascade="all, delete-orphan",
    )


class Image(Base):
    """图片表，保存上传的图片路径与归属数据集。"""
    __tablename__ = "images"

    id = Column(Integer, primary_key=True, index=True)
    dataset_id = Column(Integer, ForeignKey("datasets.id"), nullable=False, index=True)
    file_path = Column(String, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)

    dataset = relationship("Dataset", back_populates="images")
    annotations = relationship(
        "Annotation",
        back_populates="image",
        cascade="all, delete-orphan",
    )


class LabelDefinition(Base):
    """标签定义表，用于存储类别名称及可选配色。"""
    __tablename__ = "label_definitions"

    id = Column(Integer, primary_key=True, index=True)
    dataset_id = Column(Integer, ForeignKey("datasets.id"), nullable=False, index=True)
    name = Column(String(50), nullable=False)
    color = Column(String(20), nullable=True)

    dataset = relationship("Dataset", back_populates="labels")
    annotations = relationship(
        "Annotation",
        back_populates="label",
        cascade="all, delete-orphan",
    )


class Annotation(Base):
    """标注记录表，存储每个对象的边界框与所属标签。"""
    __tablename__ = "annotations"

    id = Column(Integer, primary_key=True, index=True)
    dataset_id = Column(Integer, ForeignKey("datasets.id"), nullable=False, index=True)
    image_id = Column(Integer, ForeignKey("images.id"), nullable=False, index=True)
    label_id = Column(Integer, ForeignKey("label_definitions.id"), nullable=False, index=True)

    xmin = Column(Float, nullable=False)
    ymin = Column(Float, nullable=False)
    xmax = Column(Float, nullable=False)
    ymax = Column(Float, nullable=False)
    width = Column(Float, nullable=False)
    height = Column(Float, nullable=False)
    created_at = Column(DateTime, default=datetime.utcnow, nullable=False)

    dataset = relationship("Dataset", back_populates="annotations")
    image = relationship("Image", back_populates="annotations")
    label = relationship("LabelDefinition", back_populates="annotations")
