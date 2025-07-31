# services/camera_backend/routers/__init__.py
"""
路由包：将各功能路由统一导出，便于在 app.py 中一次性导入注册。
"""

from .devices_router import router as devices_router
from .control_router import router as control_router
from .params_router import router as params_router
from .discovery_router import router as discovery_router
from .metrics_ws_router import router as metrics_ws_router
from .snapshots_router import router as snapshots_router

__all__ = [
    "devices_router",
    "control_router",
    "params_router",
    "discovery_router",
    "metrics_ws_router",
    "snapshots_router",
]
