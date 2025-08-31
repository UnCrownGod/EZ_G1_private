# File: vision/bridges/ws_http/server.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Minimal HTTP/WS bridge (non-ROS2).
轻量 HTTP/WS 桥：提供 /predict 与 /cmd 接口以便前端或外部服务调用，无 ROS2 依赖。

Features
- /health                     健康检查
- /predict/image  (POST)      上传图片推理（返回JSON，支持保存可视化）
- /predict/source (POST)      读取视频/RTSP/文件首帧推理
- /ws/predict?source=...      WebSocket 连续推理（按帧推送结果，演示用）
- /cmd            (POST)      机器人控制桥（对接 Unitree SDK2 占位封装）

Config
- yolo 配置默认读取 vision/configs/yolo.yaml（可用环境变量 YOLO_CFG 覆盖）
- 输出保存目录 runs/bridge_predict/<timestamp>（可通过 save 参数控制）

Requires
    pip install fastapi uvicorn pydantic opencv-python numpy ultralytics pyyaml
"""

from __future__ import annotations

import os
import io
import cv2
import json
import time
import base64
import typing as T
from pathlib import Path
from datetime import datetime

import numpy as np
import yaml
from fastapi import FastAPI, File, UploadFile, WebSocket, WebSocketDisconnect
from fastapi import Body, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse, StreamingResponse

from ultralytics import YOLO
from pydantic import BaseModel, Field

# 可选：对接你仓库里的 SDK2/工具（若不存在也能运行，因为我们在此文件里不强依赖）
try:
    from vision.control.unitree_sdk2_client import UnitreeSDK2Client
except Exception:
    UnitreeSDK2Client = None


# -----------------------------
# 实用函数
# -----------------------------
def now_str() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def ensure_dir(p: T.Union[str, Path]) -> Path:
    p = Path(p)
    p.mkdir(parents=True, exist_ok=True)
    return p


def bytes_to_bgr(img_bytes: bytes) -> np.ndarray:
    """Decode image bytes to BGR (cv2)."""
    arr = np.frombuffer(img_bytes, np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("Failed to decode image bytes.")
    return img


def draw_boxes(
    img: np.ndarray,
    dets: T.List[dict],
    names: T.Dict[int, str],
    score_fmt: str = "{:.2f}",
) -> np.ndarray:
    """简单画框。dets: [{'bbox':[x1,y1,x2,y2], 'conf':float, 'cls':int}, ...]"""
    out = img.copy()
    for d in dets:
        x1, y1, x2, y2 = [int(v) for v in d["bbox"]]
        cls = int(d["cls"])
        conf = float(d.get("conf", 0.0))
        name = names.get(cls, str(cls))
        cv2.rectangle(out, (x1, y1), (x2, y2), (0, 200, 255), 2)
        label = f"{name} {score_fmt.format(conf)}"
        (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.6, 1)
        cv2.rectangle(out, (x1, y1 - th - 6), (x1 + tw + 4, y1), (0, 200, 255), -1)
        cv2.putText(out, label, (x1 + 2, y1 - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (20, 20, 20), 1, cv2.LINE_AA)
    return out


def bgr_png_bytes(img: np.ndarray) -> bytes:
    ok, buf = cv2.imencode(".png", img)
    if not ok:
        raise ValueError("Failed to encode image to PNG.")
    return buf.tobytes()


# -----------------------------
# 配置/模型上下文
# -----------------------------
class YolovCfg(BaseModel):
    weights: str = "runs/detect/w_lid_v1/weights/best.pt"
    imgsz: int = 640
    conf_thres: float = 0.25
    iou_thres: float = 0.7
    device: str = "0"
    names: T.List[str] = Field(default_factory=list)  # 可选覆盖类别名


class BridgeContext:
    def __init__(self, yolo_cfg_path: T.Union[str, Path] | None = None):
        # 载入 yolo 配置
        cfg_path = Path(
            yolo_cfg_path
            or os.getenv("YOLO_CFG", "vision/configs/yolo.yaml")
        )
        if cfg_path.exists():
            with open(cfg_path, "r", encoding="utf-8") as f:
                raw = yaml.safe_load(f) or {}
        else:
            raw = {}

        self.yolo_cfg = YolovCfg(**raw) if isinstance(raw, dict) else YolovCfg()
        # 载入模型
        self.model = YOLO(self.yolo_cfg.weights)
        # 类别名
        if self.yolo_cfg.names:
            self.names = {i: n for i, n in enumerate(self.yolo_cfg.names)}
        else:
            self.names = self.model.model.names if hasattr(self.model, "model") else self.model.names

        # Unitree SDK2 客户端（可用占位）
        if UnitreeSDK2Client is not None:
            self.robot = UnitreeSDK2Client(endpoint=os.getenv("UNITREE_ENDPOINT", "udp://192.168.12.1"))
        else:
            self.robot = None  # 未安装也可运行

        # 输出目录
        self.out_root = ensure_dir("runs/bridge_predict")

    def infer_frame(self, frame_bgr: np.ndarray) -> T.Tuple[T.List[dict], np.ndarray]:
        """对单帧推理并返回 det 列表和可视化图。"""
        res = self.model.predict(
            source=frame_bgr,
            imgsz=self.yolo_cfg.imgsz,
            conf=self.yolo_cfg.conf_thres,
            iou=self.yolo_cfg.iou_thres,
            device=self.yolo_cfg.device,
            verbose=False,
        )
        dets: T.List[dict] = []
        if len(res) > 0 and hasattr(res[0], "boxes") and res[0].boxes is not None:
            boxes = res[0].boxes
            xyxy = boxes.xyxy.cpu().numpy() if hasattr(boxes, "xyxy") else None
            conf = boxes.conf.cpu().numpy() if hasattr(boxes, "conf") else None
            cls = boxes.cls.cpu().numpy().astype(int) if hasattr(boxes, "cls") else None
            if xyxy is not None and conf is not None and cls is not None:
                for i in range(xyxy.shape[0]):
                    x1, y1, x2, y2 = xyxy[i].tolist()
                    dets.append({
                        "bbox": [float(x1), float(y1), float(x2), float(y2)],
                        "conf": float(conf[i]),
                        "cls": int(cls[i]),
                        "name": self.names.get(int(cls[i]), str(int(cls[i])))
                    })
        vis = draw_boxes(frame_bgr, dets, self.names)
        return dets, vis


# -----------------------------
# FastAPI
# -----------------------------
app = FastAPI(title="Vision WS/HTTP Bridge", version="0.1.0")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"], allow_credentials=True,
    allow_methods=["*"], allow_headers=["*"],
)

CTX: BridgeContext | None = None


@app.on_event("startup")
def _on_startup():
    global CTX
    CTX = BridgeContext()
    print(f"[Bridge] YOLO loaded: {CTX.yolo_cfg.weights}")
    print(f"[Bridge] Names: {CTX.names}")


@app.get("/health")
def health():
    return {"ok": True, "ts": time.time()}


# ---------- /predict/image ----------
class PredictImageQuery(BaseModel):
    save: bool = True
    return_image: bool = False  # 直接返回可视化 PNG（Streaming）
    save_dir: str | None = None


@app.post("/predict/image")
async def predict_image(
    file: UploadFile = File(...),
    q: PredictImageQuery = Body(PredictImageQuery()),
):
    if CTX is None:
        raise HTTPException(503, "Model not ready")

    img_bytes = await file.read()
    try:
        frame = bytes_to_bgr(img_bytes)
    except Exception as e:
        raise HTTPException(400, f"Bad image: {e}")

    dets, vis = CTX.infer_frame(frame)

    saved_path = None
    if q.save:
        out_dir = ensure_dir(q.save_dir or (CTX.out_root / now_str()))
        out_img = out_dir / f"{Path(file.filename).stem}_pred.png"
        cv2.imwrite(str(out_img), vis)
        # 同步保存 JSON
        (out_dir / f"{Path(file.filename).stem}_pred.json").write_text(
            json.dumps({"detections": dets}, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        saved_path = str(out_img)

    if q.return_image:
        png_bytes = bgr_png_bytes(vis)
        return StreamingResponse(io.BytesIO(png_bytes), media_type="image/png")

    return JSONResponse({"detections": dets, "saved_image": saved_path})


# ---------- /predict/source ----------
class PredictSourceBody(BaseModel):
    source: str = Field(..., description="本地视频/图片路径、RTSP串或摄像头索引(如 '0')")
    save: bool = True
    save_dir: str | None = None
    return_image: bool = False
    read_mode: str = Field("first_frame", description="first_frame|single_image")


@app.post("/predict/source")
def predict_source(body: PredictSourceBody):
    if CTX is None:
        raise HTTPException(503, "Model not ready")

    # 尝试解析整数摄像头索引
    cap_source: T.Union[int, str]
    try:
        cap_source = int(body.source)
    except Exception:
        cap_source = body.source

    # 若是图片路径且 read_mode=single_image，直接读取图片
    if body.read_mode == "single_image" and isinstance(cap_source, str) and Path(cap_source).exists():
        img = cv2.imread(cap_source)
        if img is None:
            raise HTTPException(400, f"Cannot read image: {body.source}")
        dets, vis = CTX.infer_frame(img)
    else:
        cap = cv2.VideoCapture(cap_source)
        if not cap.isOpened():
            raise HTTPException(400, f"Cannot open source: {body.source}")
        ok, frame = cap.read()
        cap.release()
        if not ok or frame is None:
            raise HTTPException(400, "Failed to read first frame from source.")
        dets, vis = CTX
