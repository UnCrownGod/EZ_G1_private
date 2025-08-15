#!/usr/bin/env python3
# scripts/train_yolo.py
"""
Train YOLO (Ultralytics) with a given data.yaml and preset options.
使用 Ultralytics YOLO 进行训练，封装常用参数与输出。

Usage (PowerShell 示例):
    python scripts/train_yolo.py `
      --data .\runs\yolo_ds_3\data.yaml `
      --model yolov8n.pt `
      --imgsz 640 `
      --epochs 50 `
      --batch 8 `
      --device 0 `
      --project .\runs\train `
      --name w_lid_v1
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Optional

from ultralytics import YOLO


def latest_metrics_json(save_dir: Path) -> Optional[Path]:
    """Return the path to ``metrics.json`` if it exists.

    返回 ``metrics.json`` 的路径（如存在）。

    Args:
        save_dir (Path): Directory where training artifacts are saved.

    Returns:
        Optional[Path]: Path to ``metrics.json`` if present; otherwise ``None``.

    Raises:
        TypeError: If ``save_dir`` is not a ``pathlib.Path`` instance.
    """
    if not isinstance(save_dir, Path):
        raise TypeError("save_dir must be a pathlib.Path")
    candidate = save_dir / "metrics.json"
    return candidate if candidate.exists() else None


def train_model(
    data: str,
    model: str = "yolov8n.pt",
    imgsz: int = 1280,
    epochs: int = 100,
    batch: Optional[int] = None,
    device: str = "0",
    project: str = "runs/detect",
    name: str = "train",
) -> Path:
    """Train a YOLO model and return the run directory.

    训练 YOLO 模型并返回训练输出目录路径。

    Args:
        data (str): Path to ``data.yaml`` produced by your dataset export.
        model (str): Base model weights or cfg (e.g., ``yolov8n.pt``).
        imgsz (int): Image size for training/validation.
        epochs (int): Number of training epochs.
        batch (Optional[int]): Batch size；``None`` 表示由 YOLO 自动决定。
        device (str): CUDA device id (e.g., ``'0'``) or ``'cpu'``.
        project (str): Root directory for saving runs (e.g., ``runs/train``).
        name (str): Run name under ``project`` directory.

    Returns:
        Path: The directory where training artifacts are saved.

    Raises:
        FileNotFoundError: If ``data`` file does not exist.
        ValueError: If ``imgsz`` or ``epochs`` is invalid (<= 0).
        RuntimeError: If Ultralytics training failed to produce an output dir.
    """
    data_p = Path(data)
    if not data_p.exists():
        raise FileNotFoundError(f"data.yaml not found: {data_p}")
    if imgsz <= 0:
        raise ValueError("imgsz must be > 0")
    if epochs <= 0:
        raise ValueError("epochs must be > 0")

    yolo = YOLO(model)
    train_kwargs = dict(
        data=str(data_p),
        imgsz=imgsz,
        epochs=epochs,
        device=device,
        project=project,
        name=name,
        exist_ok=True,
    )
    # 仅在用户显式提供时传递 batch，避免类型校验报错
    if batch is not None:
        train_kwargs["batch"] = batch

    results = yolo.train(**train_kwargs)

    save_dir = Path(getattr(results, "save_dir", Path(project) / name))
    if not save_dir.exists():
        raise RuntimeError("Training finished but save_dir not found.")
    return save_dir


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments.

    解析命令行参数。

    Args:
        None

    Returns:
        argparse.Namespace: Parsed arguments with fields ``data``, ``model``,
        ``imgsz``, ``epochs``, ``batch``, ``device``, ``project``, ``name``.

    Raises:
        SystemExit: If arguments are invalid (raised by ``argparse``).
    """
    parser = argparse.ArgumentParser(description="Train YOLO (Ultralytics)")
    parser.add_argument("--data", required=True, type=str, help="Path to data.yaml")
    parser.add_argument("--model", default="yolov8n.pt", type=str, help="YOLO weights or cfg")
    parser.add_argument("--imgsz", default=1280, type=int, help="Image size")
    parser.add_argument("--epochs", default=100, type=int, help="Training epochs")
    parser.add_argument(
        "--batch",
        default=None,
        type=int,
        help="Batch size (int). Omit for YOLO auto-batch.",
    )
    parser.add_argument("--device", default="0", type=str, help="CUDA id (e.g. '0') or 'cpu'")
    parser.add_argument("--project", default="runs/detect", type=str, help="Project directory for runs")
    parser.add_argument("--name", default="train", type=str, help="Run name under project/")
    return parser.parse_args()


def main() -> None:
    """Entry point for CLI execution.

    脚本入口：读取参数、执行训练并打印摘要。

    Args:
        None

    Returns:
        None

    Raises:
        Exception: 训练流程中抛出的任何异常都会向上冒泡，以便 CI/终端捕获。
    """
    args = parse_args()
    save_dir = train_model(
        data=args.data,
        model=args.model,
        imgsz=args.imgsz,
        epochs=args.epochs,
        batch=args.batch,     # None => auto-batch by YOLO
        device=args.device,
        project=args.project,
        name=args.name,
    )
    print(f"[DONE] Training artifacts in: {save_dir.resolve()}")

    mj = latest_metrics_json(save_dir)
    if mj:
        try:
            metrics = json.loads(mj.read_text(encoding="utf-8"))
            print("[METRICS]", json.dumps(metrics, ensure_ascii=False)[:1200], "...")
        except Exception as exc:
            print(f"[INFO] metrics.json not readable: {exc!r}")


if __name__ == "__main__":
    main()
