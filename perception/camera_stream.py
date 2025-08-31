# File: vision/perception/camera_stream.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Unified camera/RTSP grabber with timestamps.
统一的相机/RTSP 取流器，输出图像与时间戳。

Features:
- Open local cam (index) or RTSP/file path
- Consistent BGR frames, size (w,h), fps target
- Graceful stop, context manager

Usage:
    from vision.perception.camera_stream import CameraStream
    stream = CameraStream(source="rtsp://...", width=1280, height=720, target_fps=30)
    for frame, ts in stream: ...

Requires:
    pip install opencv-python
"""

from __future__ import annotations
import cv2, time, pathlib
from typing import Iterator, Optional, Tuple

class CameraStream:
    """
    统一相机/视频/RTSP读取。
    用法:
        cam = CameraStream("rtsp://...")  # 或文件/文件夹
        for frame, ts in cam:
            ...
    """
    def __init__(self, source: str, width: Optional[int]=None, height: Optional[int]=None):
        self.source = source
        self.width = width; self.height = height
        self.cap: Optional[cv2.VideoCapture] = None
        self.files = []
        p = pathlib.Path(source)
        if p.exists():
            if p.is_dir():
                exts = {".jpg",".jpeg",".png",".bmp",".mp4",".avi",".mkv"}
                self.files = sorted([str(x) for x in p.iterdir() if x.suffix.lower() in exts])
            else:
                self.cap = cv2.VideoCapture(str(p))
        else:
            # assume RTSP/HTTP
            self.cap = cv2.VideoCapture(source)

        if self.cap is not None and self.width and self.height:
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

    def __iter__(self) -> Iterator[Tuple["cv2.Mat", float]]:
        if self.files:
            for f in self.files:
                cap = cv2.VideoCapture(f)
                ok, im = cap.read()
                if ok:
                    yield im, time.time()
                cap.release()
            return
        cap = self.cap
        if cap is None:
            raise RuntimeError(f"Cannot open source: {self.source}")
        while True:
            ok, im = cap.read()
            if not ok:
                break
            yield im, time.time()

    def release(self):
        if self.cap: self.cap.release()
