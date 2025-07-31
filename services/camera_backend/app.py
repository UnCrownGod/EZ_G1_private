# services/camera_backend/app.py
"""
Camera Backend 服务：路由注册与全局配置模块。
"""

from fastapi import FastAPI
from .db import init_db
from .routers import (
    control_router,
    devices_router,
    discovery_router,
    metrics_ws_router,
    params_router,
    snapshots_router,
)

app = FastAPI(
    title="Camera Backend",
    version="0.1.0",
    description="提供摄像设备管理、在线检测、控制、监控、网络发现与快照功能",
)


@app.on_event("startup")
def _startup() -> None:
    """应用启动时初始化数据库表。"""
    init_db()


@app.get("/", summary="服务健康检查")
async def root() -> dict:
    """返回服务运行状态。

    Returns:
        dict: 包含运行信息的字典。
    """
    return {"msg": "Camera Backend is running!"}


# ------------------------------------------------------------------------------
# 按功能模块注册子路由
# ------------------------------------------------------------------------------
app.include_router(
    devices_router,
    prefix="/devices",
    tags=["devices"],
)
app.include_router(
    control_router,
    prefix="/devices",
    tags=["control"],
)
app.include_router(
    params_router,
    prefix="/devices",
    tags=["params"],
)
app.include_router(
    discovery_router,
    prefix="/devices",
    tags=["network"],
)
app.include_router(
    metrics_ws_router,
    tags=["monitor"],
)
app.include_router(
    snapshots_router,
    prefix="/devices",
    tags=["snapshots"],
)
