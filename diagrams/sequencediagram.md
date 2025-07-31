```mermaid

sequenceDiagram
    %% ───────────────────────── 参与方 ─────────────────────────
    participant U  as 客户端
    participant A  as app.py
    participant DR as devicesRouter
    participant PR as paramsRouter
    participant CR as controlRouter
    participant SR as snapshotsRouter
    participant NR as discoveryRouter
    participant MR as metricsWS
    participant DB as camera.db
    participant UT as utils.py
    participant AD as adapters.py
    participant FS as 文件系统

    %% 1️⃣ 在线列表
    U  ->> A  : GET /devices/status
    note right of A  : FastAPI 路由到 devicesRouter.listDeviceStatus
    A  ->> DR : listDeviceStatus()
    note right of DR : 启动线程池并查表
    DR ->> DB : SessionLocal().query(Device).all()
    note right of DB : ORM 查询 devices 表
    DR ->> UT : checkDeviceOnline(dev,t)
    note right of UT : TCP / RTSP / ONVIF 探测连通性
    UT -->> DR : 返回 online、last_seen
    DR -->> A  : DeviceStatusOut[]
    note right of A  : Pydantic 序列化为列表
    A  -->> U  : 200 OK（JSON）

    %% 1-b️⃣ 单设备测试
    U  ->> A  : GET /devices/1/test
    note right of A  : 路由到 devicesRouter.testDevice
    A  ->> DR : testDevice(1)
    DR ->> UT : checkDeviceOnline()
    UT -->> DR : 在线状态
    DR -->> A  : DeviceStatusOut
    A  -->> U  : 200 OK

    %% 2️⃣ 设备创建
    U  ->> A  : POST /devices（DeviceCreate）
    note right of A  : 路由到 devicesRouter.createDevice
    A  ->> DR : createDevice(body)
    DR ->> DB : INSERT devices …
    note right of DB : 写入并返回自增 ID
    DR ->> DB : refresh(device)
    DR -->> A  : DeviceOut
    A  -->> U  : 201 已创建

    %% 2-b️⃣ 设备更新
    U  ->> A  : PUT /devices/1
    note right of A  : 路由到 devicesRouter.updateDevice
    A  ->> DR : updateDevice(1,body)
    DR ->> DB : UPDATE devices SET …
    DR ->> DB : refresh(device)
    DR -->> A  : DeviceOut
    A  -->> U  : 200 OK

    %% 3️⃣ 参数列表
    U  ->> A  : GET /devices/1/params
    note right of A  : 路由到 paramsRouter.getParams
    A  ->> PR : getParams(1)
    PR ->> DB : 查询 device_params 表
    PR -->> A  : DeviceParamOut[]
    A  -->> U  : 200 OK

    %% 3-b️⃣ 参数新增
    U  ->> A  : POST /devices/1/params
    note right of A  : 路由到 paramsRouter.addParam
    A  ->> PR : addParam(1,body)
    PR ->> DB : INSERT device_params …
    PR ->> DB : refresh(param)
    PR -->> A  : DeviceParamOut
    A  -->> U  : 201 已创建

    %% 3-c️⃣ 参数修改
    U  ->> A  : PUT /devices/1/params/1
    note right of A  : 路由到 paramsRouter.updateParam
    A  ->> PR : updateParam(1,1,body)
    PR ->> DB : UPDATE device_params …
    PR ->> DB : refresh(param)
    PR -->> A  : DeviceParamOut
    A  -->> U  : 200 OK

    %% 3-d️⃣ 参数删除
    U  ->> A  : DELETE /devices/1/params/1
    note right of A  : 路由到 paramsRouter.deleteParam
    A  ->> PR : deleteParam(1,1)
    PR ->> DB : DELETE device_params
    PR -->> A  : 204
    A  -->> U  : 204 无内容

    %% 4️⃣ 拍照快照
    U  ->> A  : POST /devices/1/control snapshot
    note right of A  : 路由到 controlRouter.controlDevice
    A  ->> CR : controlDevice(1)
    CR ->> DB : 查询设备
    CR ->> AD : getAdapter(device)
    AD ->> AD : captureSnapshot() 使用 cv2
    AD ->> FS : 保存 snapshots/1/xxx.jpg
    AD -->> CR : 返回文件路径
    CR -->> A  : ControlOut
    A  -->> U  : 200 OK（路径）

    %% 5️⃣ 快照文件
    U  ->> A  : GET /devices/1/snapshots
    note right of A  : 路由到 snapshotsRouter.listSnapshots
    A  ->> SR : listSnapshots(1)
    SR ->> FS : os.listdir()
    SR -->> A  : 文件名数组
    A  -->> U  : 200 OK

    U  ->> A  : GET /devices/1/snapshots/{f}
    note right of A  : downloadSnapshot
    A  ->> SR : downloadSnapshot
    SR ->> FS : 打开文件
    SR -->> A  : FileResponse
    A  -->> U  : JPEG 二进制

    U  ->> A  : DELETE /devices/1/snapshots/{f}
    note right of A  : deleteSnapshot
    A  ->> SR : deleteSnapshot
    SR ->> FS : 删除文件
    SR -->> A  : 204
    A  -->> U  : 204

    %% 6️⃣ 网络发现
    U  ->> A  : GET /devices/network
    note right of A  : 路由到 discoveryRouter.discoverDevices
    A  ->> NR : discoverDevices()
    NR ->> UT : 并发探测 IP 列表
    UT -->> NR : 返回在线 IP
    NR -->> A  : JSON
    A  -->> U  : 200 OK

    %% 7️⃣ 实时监控 WS
    U  ->> A  : WS /ws/devices/1/metrics
    note right of A  : 路由到 metrics_ws_router.metricsWS
    A  ->> MR : metricsWS(ws,1)
    loop 每秒
        MR ->> DB : 查询设备信息
        MR ->> MR : getMetrics(采样帧数)
        MR -->> U  : {"fps":30,"resolution":"…"}
    end
