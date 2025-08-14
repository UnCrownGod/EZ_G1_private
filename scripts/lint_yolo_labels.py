#!/usr/bin/env python3
# scripts/lint_yolo_labels.py
"""
Lint YOLO labels for common issues: missing pairs, invalid numbers, out-of-range boxes.
检查 YOLO 标签的常见问题：图标不匹配、格式错误、坐标越界、零面积。

Usage:
    python scripts/lint_yolo_labels.py ^
        --images .\yolo_dataset_ds1\images\train ^
        --labels .\yolo_dataset_ds1\labels\train ^
        --classes "plate holder,1d barcode,2d barcode,tube,markers,human_face_1"
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import List, Sequence, Tuple, Union


def parse_line(line: str) -> Tuple[int, float, float, float, float]:
    """Parse one YOLO label line into structured fields.

    解析 YOLO 标签的一行文本为结构化数据。

    Args:
        line (str): A single line from a YOLO ``.txt`` label file.

    Returns:
        Tuple[int, float, float, float, float]: Parsed fields as
        ``(class_id, x_center, y_center, width, height)``.

    Raises:
        ValueError: If the line cannot be parsed into exactly five values or
            if any value is not a valid number.
    """
    parts = line.strip().split()
    if len(parts) != 5:
        raise ValueError(f"Invalid YOLO line (expect 5 fields): {line!r}")
    try:
        c = int(float(parts[0]))
        x, y, w, h = map(float, parts[1:])
    except Exception as exc:
        raise ValueError(f"Parse error in line: {line!r}") from exc
    return c, x, y, w, h


def lint_split(
    images_dir: Union[str, Path],
    labels_dir: Union[str, Path],
    class_names: Sequence[str],
) -> dict:
    """Lint a single split's images and labels.

    对一个 split（如 train/val/test）的图像与标签执行一致性与范围检查。

    Args:
        images_dir (Union[str, Path]): Path to folder containing images.
        labels_dir (Union[str, Path]): Path to folder containing YOLO label files.
        class_names (Sequence[str]): Class name list used to validate class ids.

    Returns:
        dict: A report dictionary with keys:
            - ``missing_label_files`` (List[str])
            - ``missing_image_files`` (List[str])
            - ``bad_lines`` (List[Tuple[str, int, str]])
            - ``out_of_range`` (List[Tuple[str, int, Union[int, Tuple[float,float,float,float]]]])
            - ``zero_area`` (List[Tuple[str, int, Tuple[float, float]]])

    Raises:
        FileNotFoundError: If ``images_dir`` or ``labels_dir`` does not exist.
        ValueError: If ``class_names`` is empty.
    """
    img_dir = Path(images_dir)
    lbl_dir = Path(labels_dir)
    if not img_dir.exists():
        raise FileNotFoundError(f"images_dir not found: {img_dir}")
    if not lbl_dir.exists():
        raise FileNotFoundError(f"labels_dir not found: {lbl_dir}")
    if not class_names:
        raise ValueError("class_names must not be empty")

    valid_img_exts = {".bmp", ".jpg", ".jpeg", ".png"}
    n_cls = len(class_names)

    missing_lbl: List[str] = []
    missing_img: List[str] = []
    bad_lines: List[Tuple[str, int, str]] = []
    out_of_range: List[Tuple[str, int, Union[int, Tuple[float, float, float, float]]]] = []
    zero_area: List[Tuple[str, int, Tuple[float, float]]] = []

    # A) image -> label
    for img in img_dir.glob("*"):
        if img.suffix.lower() not in valid_img_exts:
            continue
        lbl = lbl_dir / (img.stem + ".txt")
        if not lbl.exists():
            missing_lbl.append(img.name)

    # B) label -> image, and per-line checks
    for lbl in lbl_dir.glob("*.txt"):
        # try match any common image ext
        has_img = any((img_dir / (lbl.stem + ext)).exists() for ext in valid_img_exts)
        if not has_img:
            missing_img.append(lbl.name)

        lines = lbl.read_text(encoding="utf-8").splitlines()
        for i, line in enumerate(lines, start=1):
            try:
                c, x, y, w, h = parse_line(line)
            except ValueError:
                bad_lines.append((lbl.name, i, line))
                continue
            if c < 0 or c >= n_cls:
                out_of_range.append((lbl.name, i, c))
            if not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0 and 0.0 <= w <= 1.0 and 0.0 <= h <= 1.0):
                out_of_range.append((lbl.name, i, (x, y, w, h)))
            if w <= 0.0 or h <= 0.0:
                zero_area.append((lbl.name, i, (w, h)))

    return {
        "missing_label_files": missing_lbl,
        "missing_image_files": missing_img,
        "bad_lines": bad_lines,
        "out_of_range": out_of_range,
        "zero_area": zero_area,
    }


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments.

    解析命令行参数。

    Args:
        None

    Returns:
        argparse.Namespace: 包含 ``images``, ``labels``, ``classes`` 的解析结果。

    Raises:
        SystemExit: If arguments are invalid (raised by ``argparse``).
    """
    parser = argparse.ArgumentParser(description="Lint YOLO labels")
    parser.add_argument("--images", required=True, type=str, help="Path to images/<split>")
    parser.add_argument("--labels", required=True, type=str, help="Path to labels/<split>")
    parser.add_argument(
        "--classes",
        required=True,
        type=str,
        help="Comma-separated class names (e.g. 'plate holder,1d barcode,2d barcode,tube')",
    )
    return parser.parse_args()


def main() -> None:
    """Entry point for CLI execution.

    脚本入口：执行 Lint，并打印简要报告。

    Args:
        None

    Returns:
        None

    Raises:
        Exception: 执行过程中产生的异常会向上抛出，便于 CI/终端捕获。
    """
    args = parse_args()
    class_names = [s.strip() for s in args.classes.split(",") if s.strip()]
    report = lint_split(args.images, args.labels, class_names)

    print("=== LINT REPORT ===")
    for k, v in report.items():
        # 做简短打印，避免刷屏
        head = v[:8] if isinstance(v, list) else v
        print(f"{k}: {len(v) if isinstance(v, list) else 'n/a'} -> {head}")
    print("===================")


if __name__ == "__main__":
    main()
