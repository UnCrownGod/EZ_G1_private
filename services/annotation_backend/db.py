"""
数据库模块：初始化数据库引擎、会话工厂及表模型创建。
"""

from typing import Generator
from sqlalchemy.orm import Session
from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker, declarative_base

from .config import settings

# -----------------------------------------------------------------------------
# 引擎与会话配置
# -----------------------------------------------------------------------------
# SQLite 默认使用本地文件，也可通过环境变量配置为其他数据库
engine = create_engine(
    settings.database_url,
    connect_args={"check_same_thread": False},
)
SessionLocal = sessionmaker(
    autocommit=False,
    autoflush=False,
    bind=engine,
)

# 基类，所有 ORM 模型均需继承自 Base
Base = declarative_base()


def init_db() -> None:
    """
    应用启动时初始化数据库表。

    调用此函数会扫描所有继承自 Base 的 ORM 模型并创建对应表。
    """
    # import module(s) to register all models with Base.metadata
    from . import models  # noqa: F401

    Base.metadata.create_all(bind=engine)


def get_db() -> Generator[Session, None, None]:
    """
    数据库会话依赖，用于 FastAPI 路由注入。

    Yields:
        Session: SQLAlchemy 会话对象。
    """
    db: Session = SessionLocal()
    try:
        yield db
    finally:
        db.close()