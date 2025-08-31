# File: vision/scripts/eval_testset.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Evaluate model on a YOLO dataset split.
对 YOLO 数据集 split 进行评估并保存 PR/F1/混淆矩阵等可视化，便于回归测试。

CLI:
    python vision/scripts/eval_testset.py --weights runs/detect/xxx/weights/best.pt \
        --data runs/yolo_ds_x/data.yaml --split test --device 0 --out runs/detect/test_eval
"""

from __future__ import annotations
import argparse, os, shutil, yaml, cv2
from ultralytics import YOLO
from pathlib import Path

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--model", default="runs/detect/w_lid_v2/weights/best.pt")
    ap.add_argument("--data",  default="runs/yolo_ds_3/data.yaml")
    ap.add_argument("--images", default="runs/yolo_ds_3/images/test")
    ap.add_argument("--out", default="runs/detect/test_eval_pred")
    ap.add_argument("--device", default=0, type=int)
    args = ap.parse_args()

    m = YOLO(args.model)
    # 评估（mAP、混淆矩阵）
    try:
        m.val(data=args.data, device=args.device, save_json=False, plots=True, split="test")
    except Exception as e:
        print("[WARN] val() 失败（可能无 data.yaml），跳过：", e)

    # 可视化预测
    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)
    for p in sorted(Path(args.images).glob("*.*")):
        r = m.predict(str(p), conf=0.25, device=args.device, verbose=False, save=False)[0]
        im = r.plot()
        cv2.imwrite(str(out / p.name), im)
    print(f"[DONE] 可视化保存到: {out}")

if __name__ == "__main__":
    main()
