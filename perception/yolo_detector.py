# File: vision/perception/yolo_detector.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ultralytics YOLO detector wrapper.
YOLO 推理封装：加载权重、推理图片/帧、返回 bbox/score/cls。

Config (yolo.yaml):
- weights: runs/detect/xxx/weights/best.pt
- names: ["plate holder", "1d barcode", ...]
- conf_thres / iou_thres / device

Usage:
    det = YoloDetector(cfg) ; det.infer(frame)

Requires:
    pip install ultralytics torch
"""

from __future__ import annotations
import numpy as np, cv2, json
from ultralytics import YOLO
from typing import List, Dict, Any

class YoloDetector:
    def __init__(self, cfg: Dict[str,Any]):
        self.model = YOLO(cfg["model"])
        self.conf = float(cfg.get("conf", 0.25))
        self.iou = float(cfg.get("iou", 0.5))
        self.max_det = int(cfg.get("max_det", 100))
        self.device = cfg.get("device", None)
        self.class_names = cfg.get("classes", [])

    def infer(self, img: np.ndarray) -> List[Dict[str,Any]]:
        res = self.model.predict(img, conf=self.conf, iou=self.iou, max_det=self.max_det, device=self.device, verbose=False)[0]
        out = []
        for b, c, s in zip(res.boxes.xyxy.cpu().numpy(), res.boxes.cls.cpu().numpy(), res.boxes.conf.cpu().numpy()):
            x1,y1,x2,y2 = b.tolist()
            cls_id = int(c)
            out.append({"bbox":[x1,y1,x2,y2], "score":float(s), "class_id":cls_id, "class_name": self.class_names[cls_id] if self.class_names and 0<=cls_id<len(self.class_names) else str(cls_id)})
        return out

    @staticmethod
    def draw(img: np.ndarray, dets: List[Dict[str,Any]]) -> np.ndarray:
        vis = img.copy()
        for d in dets:
            x1,y1,x2,y2 = map(int, d["bbox"])
            cv2.rectangle(vis, (x1,y1),(x2,y2),(0,255,0),2)
            cv2.putText(vis, f'{d["class_name"]}:{d["score"]:.2f}', (x1, max(0,y1-5)), cv2.FONT_HERSHEY_SIMPLEX, 0.6,(0,255,0),2, cv2.LINE_AA)
        return vis
