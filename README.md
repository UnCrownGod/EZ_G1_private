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

---

## 整体路线 & 里程碑



---

## 技术栈一览



---

## 目录结构说明

```text
vision-platform/
├─ services/                  # 每个微服务独立容器
│   ├─ rtsp/                  # MediaMTX 配置 + Dockerfile
│   ├─ cvat/                  # CVAT 容器定制（nginx 反代等）
│   ├─ training/              # YOLOv8 训练脚本 & Dockerfile
│   ├─ inference/             # FastAPI 推理服务
│   └─ ros2_bridge/           # 后期 ROS2 Node
├─ scripts/                   # 辅助脚本（推流、数据转换…）
│   └─ ingest_video.sh
├─ data/                      # 挂载的数据占位
├─ docs/                      # 规范、设计稿、会议纪要
├─ .github/
│   └─ workflows/             # 需要改
├─ docker-compose.yml         # 一键编排所有服务
├─ README.md                  # 本文件
├─ .gitignore
└─ LICENSE                    # 开源协议
```

---

## 快速开始（Day 0 → Day 2）

### Day 0：仓库脚手架



### Day 1：容器环境 & 推流/标注



### Day 2：数据整理 & 首轮训练



---

## 常用命令速查



---

## 开发规范



---

## FAQ / 常见问题



---

## 后续规划（MVP-1 / MVP-2）

### MVP-1：可持续迭代



### MVP-2：线上服务 & ROS2 桥接Unitree


---

## License

默认 MIT
