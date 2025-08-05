# Vision-Platform

> **简介**：面向实验室场景的视觉自动化平台，覆盖 **“设备接入 → 图像采集 → 数据标注 → 训练推理 → 机器人执行”** 全链路，并预留 ROS 2 对接。
>
> 当前版本已完成 **摄像设备管理子系统 (`camera_backend`)**，后续将逐步补齐训练、推理、标注与监控能力。

---

## 1 · 当前成果

| 模块                      | 功能摘要                                                                                                                                                   | 进度 |
| ----------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------ | -- |
| **camera\_backend**     | • 设备 CRUD（添加 / 编辑 / 删除）<br>• 在线状态检测（并发 TCP ping）<br>• 同网段自动发现<br>• ONVIF / RTSP 快捷测试<br>• 远程控制：拍照（已完成）/ 录像（筹备）<br>• WebSocket 实时指标推送 (fps / 分辨率 / 丢帧率) | ✅  |
| **inference\_service**  | • FastAPI + YOLOv8 最小推理接口 (完成)<br>• WebSocket 批量流推理（进行中）                                                                                               | 🔄 |
| **annotation\_backend** | • CVAT 容器化部署（完成）<br>• 数据集接口（增删改查）                                                                                                                      | 🔄 |
| **training\_service**   | • Ultralytics YOLOv8 Docker 化<br>• 训练任务队列 / GPU 调度                                                                                                     | 🔄 |
| **monitor stack**       | • Prometheus + Grafana / Loki 日志                                                                                                                       | 🔄 |
| **ros2\_bridge**        | • rclpy ↔ HTTP 桥接                                                                                                                                      | 🔄 |

> ✅ 已完成 🔄 开发中 ⭕ 计划中

---

## 2 · 系统架构（高层）

```text
Vision-Platform
├─ 前端
│   ├─ 设备管理         — 调参 / 状态 / 日志
│   ├─ 标注 / 训练      — 数据集 & 模型
│   └─ 监控面板         — FPS / GPU / 告警
├─ 后端微服务
│   ├─ camera_backend   — 摄像头 & 机械臂管理
│   ├─ inference_srv    — 推理 REST / WS
│   ├─ annotation_srv   — 数据集 & 标注 API
│   ├─ training_srv     — 训练任务调度
│   └─ ros2_bridge      — ROS 2 ↔ HTTP
├─ 基础设施
│   ├─ MediaMTX + FFmpeg — RTSP 转发 / 录像
│   ├─ MinIO + DVC      — 数据 / 模型版本
│   ├─ Prometheus stack — 监控告警
│   └─ docker-compose   — 本地一键启动
└─ 部署
    ├─ Nginx / Traefik LB
    └─ GitHub Actions CI/CD
```

---

## 3 · 目录结构

```text
vision/
├─ scripts/                 # 推流、抓帧、初始化
├─ services/
│   ├─ camera_backend/
│   │   ├─ app.py           # FastAPI 主入口
│   │   ├─ routers/         # devices / control / metrics / ...
│   │   ├─ adapters.py      # ONVIF / RTSP 操作
│   │   ├─ models.py        # SQLAlchemy ORM
│   │   ├─ schemas.py       # Pydantic DTO
│   │   └─ utils.py         # 在线检测 / 快照
│   ├─ inference/           # YOLO 推理服务
│   ├─ annotation_backend/  # 标注服务
│   ├─ training/            # 训练容器
│   └─ ros2_bridge/         # ROS2 节点
├─ docker-compose.yml       # 一键环境
└─ README.md                # 本文件
```

---

## 4 · 快速试用 `camera_backend`

```bash
# 1. 环境准备
git clone https://<your-repo>/vision.git && cd vision
python -m venv .venv && source .venv/bin/activate        # Windows 用 venv\Scripts\activate
pip install -r requirements.txt

# 2. 启动后端
uvicorn services.camera_backend.app:app --reload --port 8000

# 3. 初始化测试设备
python scripts/seed_devices.py

# 4. Swagger 文档
open http://127.0.0.1:8000/docs
```

> **RTSP 推流**
>
> `./scripts/ingest_video.sh sample.mp4` → `rtsp://localhost:8554/mystream`

\### 常用端点

| 方法       | 路径                                                      | 说明                 |
| -------- | ------------------------------------------------------- | ------------------ |
| **GET**  | `/devices/status`                                       | 列出所有设备 + 在线状态      |
| **POST** | `/devices`                                              | 新增设备（含网络 & 镜头配置）   |
| **POST** | `/devices/{id}/control` Payload `{"action":"snapshot"}` | 远程拍照，返回快照路径        |
| **WS**   | `/ws/devices/{id}/metrics?sample_frames=60`             | 推送 fps / 分辨率 / 丢帧率 |

---

## 5 · 技术栈

| 领域     | 组件                                | 说明           |
| ------ | --------------------------------- | ------------ |
| 视频流    | MediaMTX / FFmpeg                 | RTSP 推流 + 转发 |
| 标注     | CVAT                              | Web 标注       |
| 深度学习   | PyTorch / Ultralytics YOLOv8      | 训练 & 推理      |
| 服务端    | FastAPI                           | REST / WS    |
| 数据库存储  | SQLite (Dev) / MySQL (Prod)       | 元数据          |
| 对象存储   | MinIO (S3) + DVC                  | 数据集 / 模型     |
| 监控日志   | Prometheus + Grafana / Loki       | 指标 & 日志      |
| DevOps | Docker / Compose · GitHub Actions | 部署 & CI      |
| 机器人    | ROS 2 (rclpy)                     | 与 Unitree 通讯 |

---

## 6 · 下一步计划

1. **标注后端**：CVAT Webhook → 自动整理数据集 → API 暴露下载；
2. **训练服务**：容器化 YOLOv8，支持 GPU 任务队列；
3. **推理服务升级**：Triton / TorchServe，支持 batch & 伸缩；
4. **ROS 2 Bridge**：发布 `/vision/detections`，供机器人订阅；
5. **可观测性**：Prometheus exporter + Grafana dashboard；Loki 收集日志。

---

## 7 · 贡献规范

```text
feat(camera): 支持 RTSP 预置位控制
fix(router): 修复设备在线检测空列表问题
docs(README): 新增快速开始
```

* **提交格式**：`type(scope): message`
* **代码风格**：Google Python Style，`ruff` 自动格式化
* **测试**：新增 / 修改功能需附 curl / pytest 用例

---

## 8 · License

MIT © 2025 Vision‑Lab
