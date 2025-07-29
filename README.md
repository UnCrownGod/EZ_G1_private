# Vision-Platform 

> **概述**：打造一个可工业化落地的视觉平台，实现“视频流 → 标注 → 训练 → 推理”全链路闭环，并预留 ROS 2 桥接接（用于Unitree后续开发)。

---

## 📌 目录

1. [项目背景与目标](#项目背景与目标)
2. [整体路线 & 里程碑](#整体路线--里程碑)
3. [技术栈一览](#技术栈一览)
4. [目录结构说明](#目录结构说明)
5. [快速开始（Day 0 → Day 2）](#快速开始day-0--day-2)

   * [Day 0：仓库脚手架](#day-0仓库脚手架)
   * [Day 1：容器环境 & 推流/标注](#day-1容器环境--推流标注)
   * [Day 2：数据整理 & 首轮训练](#day-2数据整理--首轮训练)
6. [常用命令速查](#常用命令速查)
7. [开发规范](#开发规范)
8. [FAQ / 常见问题](#faq--常见问题)
9. [后续规划（MVP-1 / MVP-2）](#后续规划mvp-1--mvp-2)
10. [License](#license)

---

## 项目背景与目标

* **业务场景**：YOLO识别板架，验证准确率与稳定性；若技术可行，扩展到识别试管。
* **技术策略**：打标→训练→推理
* **设计原则**：模块化、容器化、可复用、可监控、可快速迭代。

---

## 整体路线 & 里程碑

| 阶段        | 目标                                             | 关键产出                                                | 预计用时    |
| --------- | ---------------------------------------------- | --------------------------------------------------- | ------- |
| **MVP-0** | 跑通RTSP 推流 → CVAT 标注 → YOLO 训练 → FastAPI 推理 | `docker-compose.yml`、首版模型 `model_v0.pt`、推理 REST API | \~5 天   |
| **MVP-1** | 数据/模型版本管理 + 实验追踪 + CI/CD                       | DVC + MinIO、MLflow/或ClearML、GitHub/Coding CI        | \~3-4 天 |
| **MVP-2** | 推理服务工业化 + ROS2 桥接 + 监控告警                       | Triton/TorchServe 服务化、ROS2 node、Prometheus/Grafana  | \~4-5 天 |


---

## 技术栈一览

| 领域      | 组件/工具                                         | 用途                      |
| ------- | --------------------------------------------- | ----------------------- |
| 视频流     | **RTSP / MediaMTX**, FFmpeg                   | 模拟摄像头、推流与接流             |
| 标注      | **CVAT**                                      | 网页化标注工具，支持视频抽帧、检测/分割等标签 |
| 深度学习    | **PyTorch / Ultralytics YOLOv8**              | 训练目标检测/分割模型             |
| 推理服务    | **FastAPI** (+ OpenCV)                        | HTTP/WS 接口，部署模型推理       |
| 数据/模型版本 | **DVC + MinIO(S3)**                           | 数据集与模型的版本控制与远端存储        |
| 实验追踪    | **MLflow / ClearML**                          | 记录实验参数、指标、模型注册表         |
| DevOps  | **Git + Gitflow、Coding CI / GitHub Actions**  | 代码版本控制、自动化流水线           |
| 容器化     | **Docker / docker-compose**                   | 一键启动所有服务，环境隔离           |
| 监控日志    | Prometheus + Grafana、ELK                      | 性能指标、日志收集与可视化           |
| 机器人接口   | **ROS 2 (rclpy)**、TF、MoveIt                   | 与机器人侧通讯、坐标转换与运动规划       |

---

## 目录结构说明

```text
vision/                     # 项目根目录
├─ pics/                    # 存放抓取的帧图片、可视化结果等
├─ scripts/                 # 辅助脚本：推流、帧抓取、数据初始化等
│   ├─ __pycache__/         # Python 字节码缓存
│   ├─ grab_frames.py       # 从视频/RTSP 流抓取帧的脚本
│   ├─ ingest_video.sh      # 视频数据导入/预处理的 Shell 脚本
│   └─ seed_devices.py      # 初始化并插入测试设备到 SQLite 的脚本
├─ services/                # 各微服务代码目录
│   ├─ __init__.py          # 包标识
│   ├─ camera_backend/      # “摄像设备管理”后端服务
│   │   ├─ __init__.py      
│   │   ├─ app.py           # FastAPI 主入口，路由定义
│   │   ├─ camera.db        # 本地 SQLite 数据库文件
│   │   ├─ models.py        # SQLAlchemy ORM 模型定义
│   │   ├─ quick_add.py     # 快速插入设备测试数据的小工具
│   │   ├─ schemas.py       # Pydantic 请求/响应模型
│   │   └─ utils.py         # 设备在线检测等工具函数
│   ├─ cvat/                # CVAT 容器定制（nginx 反代、插件等）
│   ├─ inference/           # 推理服务（FastAPI + 模型加载）
│   ├─ ros2_bridge/         # ROS2-HTTP 桥接节点服务
│   ├─ rtsp/                # RTSP 服务配置与 Dockerfile
│   └─ training/            # 训练服务（YOLOv8 训练脚本 & Dockerfile）
├─ test/                    # 测试用例（单元测试、集成测试）
├─ .gitignore               # Git 忽略配置
├─ docker-compose.yml       # 一键启动所有服务的编排文件
├─ README.md                # 项目说明文档
└─ remind_me.md             # 个人待办/笔记  

```

> 空目录通过 `.gitkeep` 或 README 占位。

---

## 快速开始（Day 0 → Day 2）

### Day 0：仓库脚手架

* 目标：搭出目录、初始化 Git 仓库、写好 README 初版。


### Day 1：容器环境 & 推流/标注

* 目标：本地能跑 RTSP 服务器和 CVAT，并用 ffmpeg 推送测试视频流。
* 环境要求：安装 **Docker Desktop**
* 编辑 `docker-compose.yml`（最小可运行版本）：

  ```yaml
  version: "3.9"

  services:
    rtsp:
      image: bluenviron/mediamtx:latest
      container_name: rtsp
      ports:
        - "8554:8554"
      restart: unless-stopped

    cvat:
      image: cvat/server:latest
      container_name: cvat
      ports:
        - "8080:8080"
      restart: unless-stopped
      environment:
        ALLOW_SIGNUP: "yes"
  ```
* 启动：

  ```bash
  docker compose up -d rtsp cvat
  docker ps
  ```
* 推流脚本 `scripts/ingest_video.sh`：

  ```bash
  #!/usr/bin/env bash
  # 用法: ./scripts/ingest_video.sh path/to/video.mp4
  VIDEO=${1:-sample.mp4}
  ffmpeg -re -stream_loop -1 \
    -i "$VIDEO" \
    -c copy -f rtsp rtsp://localhost:8554/stream
  ```

  ```bash
  chmod +x scripts/ingest_video.sh
  ./scripts/ingest_video.sh sample.mp4
  ```
* 打开 CVAT：[http://localhost:8080](http://localhost:8080)

  * 首次登录默认账号：`admin/admin`（或自行创建账号）
  * 创建 Task → 从服务器/上传抽帧 → 开始标注

### Day 2：数据整理 & 首轮训练

* 目标：把标注数据导出 → 用 YOLOv8 训练出第一版模型。
* 步骤：

  1. **CVAT 导出数据集**（COCO / YOLO 格式任一种）
  2. **准备训练环境**：

     * 本地 CPU 训练（小数据集可行）
     * 或远程 GPU（云服务器 / 实验室机器）
  3. **训练命令示例**（Ultralytics CLI）：

     ```bash
     pip install ultralytics==0.7
     yolo detect train data=./data/dataset.yaml model=yolov8n.pt epochs=50 imgsz=640
     ```
  4. **得到模型**：`runs/detect/train/weights/best.pt`
  5. **编写最小推理服务 (FastAPI)**（后续 Day 3 可容器化）：

     ```python
     # services/inference/app.py
     from fastapi import FastAPI, Query
     import cv2
     from ultralytics import YOLO

     app = FastAPI()
     model = YOLO("weights/best.pt")

     @app.get("/infer")
     def infer(url: str = Query(...)):
         cap = cv2.VideoCapture(url)
         ret, frame = cap.read()
         if not ret:
             return {"error": "cannot read frame"}
         results = model(frame)[0]
         # 转换为可序列化 JSON
         dets = []
         for box in results.boxes:
             x1, y1, x2, y2 = box.xyxy[0].tolist()
             score = float(box.conf[0])
             cls = int(box.cls[0])
             dets.append({"bbox": [x1,y1,x2,y2], "score": score, "cls": cls})
         return {"detections": dets}
     ```

---

## 常用命令速查表

```bash
# 启动/停止服务
docker compose up -d rtsp cvat
docker compose down

# 查看容器状态/日志
docker ps
docker logs -f rtsp

# 推流
./scripts/ingest_video.sh sample.mp4

# Git 提交流程
git add .
git commit -m "feat: xxx"
git push
```

---

## 开发规范

* **Git 提交信息格式**：`type(scope): message`

  * type: feat / fix / chore / docs / refactor / test
  * example：`feat(inference): add websocket endpoint`
* **分支**：MVP-0 阶段先在 `master` 开发；MVP-1 后转 Gitflow（master/develop/feature/\*）。

---

## FAQ / 常见问题

* **Q: 为什么要用 Docker？**
  A: 部署简单、环境一致、隔离干净。RTSP/CVAT 官方都给了容器镜像，几乎无原生安装教程。

* **Q: Windows 环境如何推流？**
  A: 推荐用 Git Bash 或 WSL2 运行 `ingest_video.sh`，或在 Docker 容器里直接用 ffmpeg。

* **Q: CVAT 看不到 RTSP 流？**
  A: CVAT 不直接拉 RTSP，你需要先抽帧：用脚本抓帧保存成图片，然后上传标注；或用 CVAT 的 "From server" 功能读取服务器中的静态文件。

* **Q: 模型训练太慢怎么办？**
  A: 小数据集可本地 CPU 训练；正式训练建议租用 GPU 或实验室服务器。

---

## 后续规划（MVP-1 / MVP-2）

### MVP-1：可持续迭代

* **DVC + MinIO**：数据集和模型用 DVC 追踪，存到 MinIO 桶。
* **MLflow**：记录每次实验的参数、指标、模型文件；建立 Model Registry。
* **CI/CD**：Coding CI 或 GitHub Actions 在 push 时自动跑 lint/test/构建镜像。

### MVP-2：线上服务 & ROS2 桥接

* **推理服务工业化**：Triton/TorchServe，支持 batch、并发、监控。
* **ROS2 Node**：单独包订阅推理结果，发布 `/vision/detections` 话题；坐标转换(TF)、MoveIt集成。
* **监控告警**：Prometheus + Grafana 抓取推理延迟、吞吐；ELK 收集日志。

---

## License

默认 MIT
