# File: vision/scripts/record_run.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Record images/metrics during a run.
运行期间录制视频/保存关键帧与指标（fps、latency），方便离线排错与复现实验。

CLI:
    python vision/scripts/record_run.py --source 0 --out runs/records/run_001
"""

from __future__ import annotations
import argparse, cv2, time
from vision.perception.camera_stream import CameraStream

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--src", required=True)
    ap.add_argument("--out", default="record.avi")
    ap.add_argument("--fps", default=25, type=int)
    args = ap.parse_args()

    cam = CameraStream(args.src)
    fourcc = cv2.VideoWriter_fourcc(*"XVID")
    writer = None
    for im, ts in cam:
        if writer is None:
            h,w = im.shape[:2]
            writer = cv2.VideoWriter(args.out, fourcc, args.fps, (w,h))
        writer.write(im)
        cv2.imshow("record", im)
        if cv2.waitKey(1)==27: break
    if writer: writer.release()

if __name__ == "__main__":
    main()
