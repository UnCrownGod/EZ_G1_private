# services/camera_backend/schemas.py
"""
Pydantic 模型：用于请求与响应的数据验证与序列化。
"""

from pydantic import BaseModel, ConfigDict


class DeviceBase(BaseModel):
    """基础设备信息。"""
    id: int
    name: str
    protocol: str
    url: str

    model_config = ConfigDict(from_attributes=True)


class DeviceStatusOut(DeviceBase):
    """设备在线状态输出模型。"""
    online: bool | None = None
    last_seen: str | None = None


class DeviceParamBase(BaseModel):
    """设备参数基础字段。"""
    key: str
    value: str


class DeviceParamCreate(DeviceParamBase):
    """创建参数时的请求体。"""
    pass


class DeviceParamUpdate(BaseModel):
    """更新参数时的请求体。"""
    value: str


class DeviceParamOut(DeviceParamBase):
    """参数输出模型。"""
    id: int
    device_id: int
    updated_at: str

    model_config = ConfigDict(from_attributes=True)
