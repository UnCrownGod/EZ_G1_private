# services/camera_backend/db.py
"""
数据库会话与初始化依赖模块。
"""

from typing import Generator

from sqlalchemy.orm import Session

from . import models


def get_db() -> Generator[Session, None, None]:
    """创建并返回数据库会话，使用后自动关闭。

    Yields:
        Session: 数据库会话。
    """
    db = models.SessionLocal()
    try:
        yield db
    finally:
        db.close()


def init_db() -> None:
    """初始化数据库表，应用启动时调用。"""
    models.init_db()
