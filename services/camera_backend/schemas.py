# services/camera_backend/schemas.py
"""
Pydantic 模型：用于请求与响应的数据验证与序列化。
"""

from typing import Literal, Optional, List, Dict, Any
from pydantic import BaseModel, ConfigDict

# ------------------------------------------------------------------------------
# 在线状态 & 参数管理模型
# ------------------------------------------------------------------------------

class DeviceBase(BaseModel):
    """基础设备信息。

    Attributes:
        id (int): 设备 ID。
        name (str): 设备名称。
        protocol (str): 通信协议，"onvif" 或 "rtsp"。
        url (str): ONVIF 地址或 RTSP URL。
    """
    id: int
    name: str
    protocol: str
    url: str

    model_config = ConfigDict(from_attributes=True)


class DeviceStatusOut(DeviceBase):
    """设备在线状态输出模型。

    Attributes:
        online (bool | None): 是否在线；None 表示未检测。
        last_seen (str | None): 最近在线时间，ISO 8601 UTC；None 表示未检测。
    """
    online: bool | None = None
    last_seen: str | None = None


class DeviceParamBase(BaseModel):
    """设备参数基础字段。

    Attributes:
        key (str): 参数名。
        value (str): 参数值（JSON 或普通文本）。
    """
    key: str
    value: str


class DeviceParamCreate(DeviceParamBase):
    """创建参数时的请求体。

    继承自 DeviceParamBase。
    """
    pass


class DeviceParamUpdate(BaseModel):
    """更新参数时的请求体。

    Attributes:
        value (str): 新的参数值。
    """
    value: str


class DeviceParamOut(DeviceParamBase):
    """参数输出模型。

    Attributes:
        id (int): 参数记录 ID。
        device_id (int): 关联的设备 ID。
        updated_at (str): ISO 8601 UTC 时间戳，表示最后更新时间。
    """
    id: int
    device_id: int
    updated_at: str

    model_config = ConfigDict(from_attributes=True)


# ------------------------------------------------------------------------------
# 设备创建 & 更新相关模型
# ------------------------------------------------------------------------------
class NetworkConfig(BaseModel):
    """网络配置。

    Attributes:
        ip (str): 摄像头 IP 地址。
        port (int): 摄像头端口，例如 554。
        protocol (Literal["onvif", "rtsp"]): 通信协议。
        username (str | None): 登录用户名。
        password (str | None): 登录密码。
    """
    ip: str
    port: int
    protocol: Literal["onvif", "rtsp"]
    username: Optional[str] = None
    password: Optional[str] = None


class ImageParams(BaseModel):
    """图像参数配置。

    Attributes:
        resolution (str): 分辨率，如 "1920x1080"。
        fps (int): 帧率。
        exposure_us (int): 曝光时间（微秒）。
        gain_db (float): 增益（分贝）。
        white_balance (str): 白平衡模式，如 "auto"。
        focus (str): 对焦模式，如 "auto"。
        colour_space (str): 色彩空间，如 "RGB/YUV"。
        bit_depth (str): 位深，如 "8bit"、"10bit"。
        denoise (str): 降噪开关，如 "on"/"off"。
    """
    resolution: str
    fps: int
    exposure_us: int
    gain_db: float
    white_balance: str
    focus: str
    colour_space: str
    bit_depth: str
    denoise: str


class LensDistortion(BaseModel):
    """镜头畸变参数。

    Attributes:
        k1 (float): 径向畸变系数 k1。
        k2 (float): 径向畸变系数 k2。
        k3 (float): 径向畸变系数 k3。
        p1 (float): 切向畸变系数 p1。
        p2 (float): 切向畸变系数 p2。
    """
    k1: float
    k2: float
    k3: float
    p1: float
    p2: float


class IntrinsicMatrix(BaseModel):
    """相机内参矩阵。

    Attributes:
        fx (float): 焦距 fx。
        fy (float): 焦距 fy。
        cx (float): 主点 cx。
        cy (float): 主点 cy。
    """
    fx: float
    fy: float
    cx: float
    cy: float


class SyncConfig(BaseModel):
    """同步设置。

    Attributes:
        hardware_sync (str): 硬件同步开关，如 "on"/"off"。
        trigger_mode (str): 触发模式，如 "free"/"external"。
    """
    hardware_sync: str
    trigger_mode: str


class DeviceCreate(BaseModel):
    """创建设备时的请求体。

    Attributes:
        name (str): 设备名称。
        manufacturer (str): 厂商名称。
        model (str): 型号。
        network (NetworkConfig): 网络配置。
        image_params (ImageParams): 图像参数配置。
        lens_distortion (LensDistortion): 镜头畸变参数。
        intrinsic (IntrinsicMatrix): 相机内参矩阵。
        sync (SyncConfig): 同步设置。
    """
    name: str
    manufacturer: str
    model: str
    network: NetworkConfig
    image_params: ImageParams
    lens_distortion: LensDistortion
    intrinsic: IntrinsicMatrix
    sync: SyncConfig


class DeviceOut(BaseModel):
    """设备输出模型（含完整配置）。

    Attributes:
        id (int): 设备 ID。
        name (str): 设备名称。
        url (str): 访问 URL。
        protocol (str): 协议类型。
        config (Dict[str, Any]): 存储的配置 JSON 反序列化结果。
    """
    id: int
    name: str
    url: str
    protocol: str
    config: Dict[str, Any]

    model_config = ConfigDict(from_attributes=True)

# ------------------------------------------------------------------------------
# control in & control out
# ------------------------------------------------------------------------------

class ControlIn(BaseModel):
    """控制接口请求体。

    Attributes:
        action (Literal["open","close","snapshot","start_record","stop_record"]):
            控制动作名称。
    """
    action: Literal["open", "close", "snapshot", "start_record", "stop_record"]


class ControlOut(BaseModel):
    """控制接口响应模型。

    Attributes:
        action (str): 执行的动作名称。
        detail (str): 操作结果描述或文件路径。
    """
    action: str
    detail: str