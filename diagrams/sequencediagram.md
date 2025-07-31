```mermaid

%% 摄像设备后端完整交互图（含中文说明，版本 2025-07-31）
sequenceDiagram
    %% ───────── 参与者（对应真实文件） ─────────
    participant Client as 客户端（浏览器 / PowerShell）
    participant App as app.py
    participant DR as devices_router.py
    participant CR as control_router.py
    participant PR as params_router.py
    participant DiR as discovery_router.py
    participant MR as metrics_ws_router.py
    participant SR as snapshots_router.py
    participant Utils as utils.py
    participant Adapters as adapters.py
    participant Schemas as schemas.py
    participant DB as db.py<br/>(SessionLocal, SQLite)
    participant FS as 文件系统
    participant Monitor as monitor.py

    %% ───────── 1. 在线状态列表 ─────────
    note over Client,App: 客户端请求“所有摄像头实时在线状态”
    Client->>App: **GET** /devices/status
    note right of App: FastAPI 路由 → DR.list_device_status
    App->>DR: list_device_status()

    note right of DR: 调用 SessionLocal<br/>查询 devices 表
    DR->>DB: SessionLocal().query(Device).all()
    note right of DB: 执行 SQL<br/>SELECT * FROM devices
    DB-->>DR: ORM 结果列表

    loop 遍历每台设备
        note right of Utils: TCP 端口探测<br/>RTSP 554 / ONVIF 自带端口
        DR->>Utils: check_device_online(dev)
        Utils-->>DR: online ✔/✘ + last_seen
    end

    note right of Schemas: DeviceStatusOut → JSON
    DR->>Schemas: 序列化
    DR-->>App: List[DeviceStatusOut]
    App-->>Client: 200 OK(JSON)

    %% ───────── 2. 单设备在线测试 ─────────
    note over Client: 点击“测试连接”按钮
    Client->>App: **GET** /devices/1/test
    App->>DR: test_device()
    DR->>DB: SessionLocal().query(Device).get(1)
    DR->>Utils: check_device_online(...)
    Utils-->>DR: 结果
    DR->>Schemas: DeviceStatusOut
    DR-->>App: DeviceStatusOut
    App-->>Client: 200 OK

    %% ───────── 3. 设备创建 / 更新 ─────────
    note over Client: 提交摄像头配置表单
    Client->>App: **POST** /devices {DeviceCreate}
    note right of Schemas: DeviceCreate 校验字段
    App->>DR: create_device()
    DR->>DB: INSERT INTO devices
    DR->>Schemas: DeviceOut
    DR-->>App: DeviceOut
    App-->>Client: 201 Created

    Client->>App: **PUT** /devices/1 {DeviceCreate}
    App->>DR: update_device()
    DR->>DB: SELECT + UPDATE
    DR->>Schemas: DeviceOut
    DR-->>App: DeviceOut
    App-->>Client: 200 OK

    %% ───────── 4. 参数管理 CRUD ─────────
    note over PR: 设备自定义键值对
    Client->>App: **POST** /devices/1/params
    App->>PR: add_param()
    PR->>DB: INSERT INTO device_params
    PR->>Schemas: DeviceParamOut
    PR-->>App: DeviceParamOut
    App-->>Client: 201 Created

    %% ───────── 5. 快照拍照 ─────────
    note over CR: 根据协议选择适配器
    Client->>App: **POST** /devices/1/control {"action":"snapshot"}
    App->>CR: control_device()
    CR->>DB: SELECT Device BY id
    CR->>Adapters: get_adapter(device)
    note right of Adapters: OpenCV 抓帧 → 保存 JPEG
    Adapters->>FS: snapshots/1/<ts>.jpg
    Adapters-->>CR: 文件路径
    CR->>Schemas: ControlOut
    CR-->>App: ControlOut
    App-->>Client: 200 OK

    %% ───────── 6. 快照文件管理 ─────────
    note over SR: 列出 / 下载 / 删除快照
    Client->>App: **GET** /devices/1/snapshots
    App->>SR: list_snapshots()
    SR->>FS: os.listdir()
    SR-->>App: 文件名数组
    App-->>Client: 200 OK(JSON)

    %% ───────── 7. 同网段设备发现 ─────────
    note over DiR: 线程池并发扫描 IP:port
    Client->>App: **GET** /devices/network?prefix=…
    App->>DiR: discover_devices()
    DiR->>Socket: create_connection ×N
    Socket-->>DiR: 成功/失败
    DiR-->>App: 在线 IP 列表
    App-->>Client: 200 OK(JSON)

    %% ───────── 8. WebSocket 实时监控 ─────────
    note over MR: 每秒采样 N 帧计算 FPS
    ClientWebSocket->>App: **WS** /ws/devices/1/metrics
    App->>MR: metrics_ws()
    loop 每秒
        MR->>Monitor: get_metrics(url,N)
        Monitor-->>MR: {"fps","resolution","dropped"}
        MR-->>ClientWebSocket: send_json()
    end
