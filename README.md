# Vision-Platform

> 面向实验室的视觉自动化平台：**相机接入 → 数据标注 → 训练/评估 → 推理服务 →（预留）机器人对接**。
> 当前重点：**板架（plate holder）检测**的离线训练与本地推理；相机后台与数据管理脚本已就绪，ROS2/机械臂对接留待到货后在现场落地。

---

## 0. 已实现功能

* ✅ **相机后台 `camera_backend`**：设备 CRUD、在线探测、快照、WS 指标。
* ✅ **数据集工作流（两种）**

  * A. **纯文件法（推荐）**：`datasets/<name>/images|labels/{train,val,test}` + `data.yaml` → 直接训练/验证/预测。
  * B. **后端导出法**：`annotation_backend` 入库 → `export_yolo_dataset.py` 一键导出为 YOLO 目录。
* ✅ **训练脚本**：`scripts/train_yolo.py`（已在你机上产出 `runs/detect/w_lid_v1`）。
* ✅ **标注质量检查**：`scripts/lint_yolo_labels.py`。
* ✅ **批量预测与可视化**：`scripts/predict_visualize.py`。
* 🧩 **配置**：`configs/*.yaml` 保存相机、标定、抓取工作点等参数（文本可版控）。
* 🔌 **（预留）ROS 2 桥**：`services/ros2_bridge/`，待实物到位后连通话题。

---

## 1. 目录结构（与仓库保持一致）

```text
vision/
├─ configs/                 # 文本配置：相机/几何/抓取参数等（可版控）
├─ datasets/                # 训练数据（标准 YOLO 目录：images/labels + data.yaml）
├─ docs/                    # 说明与图
├─ exports/                 # 环境/依赖导出等
├─ pics/                    # 原始图片/临时预测输入
├─ robot/                   # 未来：抓取/标定脚本与ROS2接口
├─ runs/                    # 训练与预测输出（Ultralytics 自动生成）
├─ scripts/                 # 训练/导出/校验/预测等脚本
├─ services/                # camera_backend / annotation_backend / inference / ...
├─ docker-compose.yml
└─ requirements.txt
```

> 提示：`runs/` 与 `datasets/` **不入库大文件**时，可结合 DVC/MinIO；目前先本地开发为主。

---

## 2. 快速开始（Windows PowerShell）

> 目标：**不依赖任何 .db**，用**纯文件法**从 0 到训练/验证/预测。
> 已有数据可直接把 `datasets/<你的数据集>/` 放到仓库里；若使用后端导出法，见下一节。

### 2.1 环境准备

```powershell
cd vision
python -m venv .venv
.\.venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

> 说明：仓库里 `yolov8n.pt` 是 YOLOv8 nano 预训练权重；若决定更改 YOLO11，修改为 `--model yolo11n.pt` 即可。

### 2.2（可选）检查标注质量

```powershell
python scripts\lint_yolo_labels.py `
  --images .\datasets\w_lid\images\train `
  --labels .\datasets\w_lid\labels\train `
  --classes "plate holder,1d barcode,2d barcode"
```

### 2.3 训练

```powershell
python scripts\train_yolo.py `
  --data .\datasets\w_lid\data.yaml `
  --model yolov8n.pt `
  --imgsz 640 `
  --epochs 50 `
  --batch 8 `
  --device 0 `
  --name w_lid_v1
```

产物：`runs\detect\w_lid_v1\weights\best.pt`、曲线图与混淆矩阵等。

### 2.4 验证（测试集）

```powershell
yolo val model="runs\detect\w_lid_v1\weights\best.pt" `
         data="datasets\w_lid\data.yaml" `
         split=test device=0
```

### 2.5 批量预测与可视化

```powershell
python scripts\predict_visualize.py `
  --weights .\runs\detect\w_lid_v1\weights\best.pt `
  --source  .\pics\wo_lid `
  --out-dir .\runs\demo\wo_lid
```

输出：标框后的图片与 `labels/xxx.txt`。

---

## 3. 另一种数据工作流：**后端导出法**（需要 .db）

> 适用于要**统一管理多人数据**、或从 CVAT 同步导出的场景。你之前就是这样做的。

1. 启动后端（已内置 SQLite）

```powershell
uvicorn services.annotation_backend.app:app --port 8000 --reload
```

2. 通过 API 创建数据集 & 导入图片/YOLO标签（PowerShell 略，同你之前用法）
3. 导出为标准 YOLO 目录：

```powershell
python scripts\export_yolo_dataset.py `
  --dataset-id <id> `
  --out-dir .\datasets\w_lid `
  --val-ratio 0.15 `
  --test-ratio 0.05 `
  --api-base http://127.0.0.1:8000
```

接下去用第 2 节的训练/验证/预测命令即可。

> `.db` 是**后端的元数据库**；**训练/推理并不依赖它**。两种工作流任选其一。

---

## 4. 相机后台（可选）

快速试用：

```powershell
uvicorn services.camera_backend.app:app --reload --port 8000
python scripts\seed_devices.py
# 打开 http://127.0.0.1:8000/docs
```

常用端点：

| 方法   | 路径                                                | 说明          |
| ---- | ------------------------------------------------- | ----------- |
| GET  | `/devices/status`                                 | 列设备与在线状态    |
| POST | `/devices`                                        | 新增设备        |
| POST | `/devices/{id}/control` + `{"action":"snapshot"}` | 拍照          |
| WS   | `/ws/devices/{id}/metrics?sample_frames=60`       | fps/分辨率/丢帧率 |

---

## 5. 重要脚本说明

| 脚本                                   | 用途（最常用参数）                                                                           |
| ------------------------------------ | ----------------------------------------------------------------------------------- |
| `scripts/train_yolo.py`              | 训练：`--data <data.yaml> --model yolov8n.pt --epochs --imgsz --batch --device --name` |
| `scripts/lint_yolo_labels.py`        | 标注体检：`--images images/train --labels labels/train --classes "a,b,c"`                |
| `scripts/predict_visualize.py`       | 批量预测：`--weights best.pt --source pics\foo --out-dir runs\demo\foo`                  |
| `scripts/export_yolo_dataset.py`     | 从标注后端导出 YOLO 目录                                                                     |
| `scripts/import_yolo_annotations.py` | 将本地 YOLO 标签入库（若走后端工作流）                                                              |
| `scripts/grab_frams.py`              | 从 RTSP/视频抓帧（配合相机或流媒体）                                                               |
| `scripts/ingest_video.sh`            | 把 mp4 推到本地 MediaMTX 形成 RTSP 源                                                       |

**docstring 规范 (Args/Returns/Raises)** 已在这些脚本中统一遵循。

---

## 6. 配置与权重：FAQ

* **`configs/*.yaml`**：纯文本配置，保存相机参数、坐标系、抓取位姿、ROI 等，将来给推理服务与机器人侧统一读取。
* **`*.db`**：仅用于后端的**元数据**（数据集索引/相机清单）；**不是训练必需品**。
* **`yolov8n.pt` vs `yolo11n.pt`**：均为官方预训练权重。当前流程默认用 `yolov8n.pt`；想试新版，把 `--model yolo11n.pt` 即可。
* **Ultralytics 输出路径**：控制台里若出现 `path\to\dir` 文案，那是占位字符串；真实路径请看末行 `Results saved to ...` 与 `runs/...` 目录。

---

## 7. 下一步（offline）

1. 扩充/打磨标注集（含遮挡、倾斜、不同光照），持续用 `lint_yolo_labels.py` 体检。
2. 训练若干版本（`n/s/m` 模型、不同 `imgsz`/增强策略），在固定验证集对比 `mAP50-95` & 混淆矩阵。
3. 在 `services/inference/` 补齐**轻量推理 REST/WS**（读取 `configs/inference.yaml`）。
4. 在 `configs/` 整理**相机外参/场景标定**模板，准备到货后根据现场测量填入。
5. 预置 `robot/` 中的**抓取参数结构**与**ROS2 消息协议**（话题：`/vision/detections`），等实物到位连通。

---

## 8. 贡献规范

* 提交信息：`type(scope): message`（如 `feat(training): support yolov11n`）
* 代码风格：Google Python Style，`ruff` 自动格式化
* 测试：提供 `curl` 或最小复现指令（尤其是训练与导出脚本）

---

## 9. License

MIT © 2025 Vision-Lab

---
