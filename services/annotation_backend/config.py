# services/annotation_backend/config.py

"""
配置模块：全局配置与环境变量管理。
"""

from pydantic_settings import BaseSettings, SettingsConfigDict

class Settings(BaseSettings):
    """
    基础设置，支持通过环境变量或 .env 文件覆盖。
    """

    # 数据库连接 URL，支持 sqlite、postgresql 等
    database_url: str = "sqlite:///./annotation.db"
    # 本地文件存储根目录
    storage_dir: str = "./data"

    # 异步任务队列（Celery）配置
    redis_url: str = "redis://localhost:6379/0"

    # Pydantic v2 + pydantic-settings 指定 .env 文件
    model_config = SettingsConfigDict(
        env_file=".env",
        env_file_encoding="utf-8",
    )

# 全局配置实例
settings = Settings()
