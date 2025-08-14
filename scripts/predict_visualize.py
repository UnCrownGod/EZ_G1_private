#!/usr/bin/env python3
# scripts/predict_visualize.py
"""
Run YOLO inference on a folder and save visualized results.
对文件夹做推理并保存可视化结果图。

Usage:
    python scripts/predict_visualize.py ^
        --weights runs/detect/ds1_v8n/weights/best.pt ^
        --source .\yolo_dataset_ds1\images\val ^
        --out-dir .\pred_vis ^
        --conf 0.25
"""

from __future__ import annotations

import argparse
from pathlib import Path
from ultralytics import YOLO


def main() -> None:
    """CLI entry. 命令行入口。"""
    parser = argparse.ArgumentParser(description="YOLO Inference and Visualization")
    parser.add_argument("--weights", required=True, type=str, help="Path to model weights (.pt)")
    parser.add_argument("--source", required=True, type=str, help="Folder or file for inference")
    parser.add_argument("--out-dir", default="./pred_vis", type=str, help="Output directory for visualized images")
    parser.add_argument("--conf", default=0.25, type=float, help="Confidence threshold")
    args = parser.parse_args()

    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)

    model = YOLO(args.weights)
    model.predict(
        source=args.source,
        conf=args.conf,
        save=True,
        save_txt=False,
        project=str(out),
        name="",
        exist_ok=True,
        imgsz=None,  # 使用训练默认
        verbose=True,
    )
    print(f"[DONE] Predictions saved to: {out.resolve()}")


if __name__ == "__main__":
    main()
