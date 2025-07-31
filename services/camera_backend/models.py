# services/camera_backend/models.py
"""
数据模型：摄像设备和设备参数。

保证 SQLite 数据库文件始终在本模块目录下。
"""

import os

from sqlalchemy import Column, ForeignKey, Integer, String, Text, create_engine
from sqlalchemy.orm import declarative_base, sessionmaker

# 1. 计算出本脚本所在目录
BASE_DIR = os.path.dirname(os.path.abspath(__file__))

# 2. 构造绝对路径的 SQLite URL
DB_PATH = os.path.join(BASE_DIR, "camera.db")
SQLALCHEMY_DATABASE_URL = f"sqlite:///{DB_PATH}"

# 3. 定义 ORM 基类
Base = declarative_base()


class Device(Base):
    """摄像设备表。

    Attributes:
        id (int): 主键，自增。
        name (str): 设备名称。
        protocol (str): 协议，\"onvif\" 或 \"rtsp\"。
        url (str): 设备访问 URL。
        config (str | None): 前端配置表单 JSON。
    """
    __tablename__ = "devices"

    id = Column(Integer, primary_key=True, autoincrement=True)
    name = Column(String, nullable=False)
    protocol = Column(String, nullable=False)
    url = Column(String, nullable=False)
    config = Column(Text, nullable=True)  # 存 JSON 字符串


class DeviceParam(Base):
    """设备参数表。

    Attributes:
        id (int): 主键，自增。
        device_id (int): 外键，关联 Device.id。
        key (str): 参数名。
        value (str): 参数值。
        updated_at (str): ISO 8601 UTC 时间戳。
    """
    __tablename__ = "device_params"

    id = Column(Integer, primary_key=True, autoincrement=True)
    device_id = Column(Integer, ForeignKey("devices.id"), nullable=False)
    key = Column(String, nullable=False)
    value = Column(Text, nullable=False)
    updated_at = Column(String, nullable=False)


# 4. 创建 Engine 和 Session
engine = create_engine(
    SQLALCHEMY_DATABASE_URL,
    echo=False,
    connect_args={"check_same_thread": False},  # SQLite 多线程需要
)

SessionLocal = sessionmaker(bind=engine, autocommit=False, autoflush=False)


def init_db() -> None:
    """创建所有表。首次运行或模型变更后调用。"""
    Base.metadata.create_all(engine)
