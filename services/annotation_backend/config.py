"""
配置模块：全局配置与环境变量管理。
"""
from pydantic import BaseSettings


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

    class Config:
        # 从项目根目录的 .env 文件加载环境变量
        env_file = ".env"
        env_file_encoding = "utf-8"


# 全局配置实例
settings = Settings()
