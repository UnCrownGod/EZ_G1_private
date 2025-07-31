# services/camera_backend/routers/snapshots.py
"""
快照管理接口：列出、下载、删除快照文件。
"""

import os
from typing import List

from fastapi import APIRouter, HTTPException, status
from fastapi.responses import FileResponse

router = APIRouter()


@router.get(
    "/{dev_id}/snapshots",
    response_model=List[str],
)
def list_snapshots(dev_id: int) -> List[str]:
    """列出指定设备的所有快照文件名。

    Args:
        dev_id (int): 设备 ID。

    Returns:
        List[str]: 按名称排序的快照文件列表。
    """
    folder = os.path.join("snapshots", str(dev_id))
    if not os.path.isdir(folder):
        return []
    return sorted(os.listdir(folder))


@router.get("/{dev_id}/snapshots/{filename}")
def get_snapshot(dev_id: int, filename: str) -> FileResponse:
    """下载指定设备的快照文件。

    Args:
        dev_id (int): 设备 ID。
        filename (str): 快照文件名。

    Raises:
        HTTPException: 如果文件不存在，则返回 404。

    Returns:
        FileResponse: 返回文件流。
    """
    path = os.path.join("snapshots", str(dev_id), filename)
    if not os.path.isfile(path):
        raise HTTPException(status_code=404, detail="Snapshot not found")
    return FileResponse(path)


@router.delete(
    "/{dev_id}/snapshots/{filename}",
    status_code=status.HTTP_204_NO_CONTENT,
)
def delete_snapshot(dev_id: int, filename: str) -> None:
    """删除指定设备的快照文件。

    Args:
        dev_id (int): 设备 ID。
        filename (str): 快照文件名。

    Raises:
        HTTPException: 如果文件不存在，则返回 404。
    """
    path = os.path.join("snapshots", str(dev_id), filename)
    if not os.path.isfile(path):
        raise HTTPException(status_code=404, detail="Snapshot not found")
    os.remove(path)
