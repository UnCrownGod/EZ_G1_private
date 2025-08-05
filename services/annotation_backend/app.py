"""
Annotation Backend 服务：路由注册与全局配置模块。
"""

from fastapi import FastAPI
from .db import init_db
from .routers.datasets import router as datasets_router
from .routers.images import router as images_router
from .routers.labels import router as labels_router
from .routers.annotations import router as annotations_router
from .routers.export import router as export_router
from .routers.status import router as status_router

app = FastAPI(
    title="Annotation Backend",
    version="0.1.0",
    description="提供数据集创建、图片导入、标签定义、标注管理与导出功能",
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
    return {"msg": "Annotation Backend is running!"}


# -------------------------------------------------------------------------------
# 按功能模块注册子路由
# -------------------------------------------------------------------------------
app.include_router(
    datasets_router,
    prefix="/datasets",
    tags=["datasets"],
)
app.include_router(
    images_router,
    prefix="/datasets/{dataset_id}/images",
    tags=["images"],
)
app.include_router(
    labels_router,
    prefix="/datasets/{dataset_id}/labels",
    tags=["labels"],
)
app.include_router(
    annotations_router,
    prefix="/datasets/{dataset_id}/annotations",
    tags=["annotations"],
)
app.include_router(
    export_router,
    prefix="/datasets/{dataset_id}/export",
    tags=["export"],
)
app.include_router(
    status_router,
    prefix="/datasets/{dataset_id}/status",
    tags=["status"],
)
