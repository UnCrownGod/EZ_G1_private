#!/usr/bin/env python3
# scripts/lint_yolo_labels.py
"""
Lint YOLO labels for common issues: missing pairs, invalid numbers, out-of-range boxes.
检查 YOLO 标签的常见问题：图像/标签不匹配、格式错误、坐标越界、零面积、过小目标等。

Usage:
    # 从 data.yaml 自动读取类别名（推荐）
    python scripts/lint_yolo_labels.py ^
        --images .\yolo_dataset\images\train ^
        --labels .\yolo_dataset\labels\train ^
        --data-yaml .\yolo_dataset\data.yaml

    # 或者手动传入类别名
    python scripts/lint_yolo_labels.py ^
        --images .\yolo_dataset\images\train ^
        --labels .\yolo_dataset\labels\train ^
        --classes "plate holder,1d barcode,2d barcode"

    # 可选阈值
    --min-side 0.0 --min-area 0.0
"""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Dict, List, Sequence, Tuple, Union

try:
    import yaml  # PyYAML
except Exception:  # pragma: no cover
    yaml = None


def parse_line(line: str) -> Tuple[int, float, float, float, float]:
    """Parse one YOLO label line into structured fields.

    解析 YOLO 标签的一行文本为结构化数据。

    Args:
        line (str): A single non-empty, non-comment line from a YOLO ``.txt`` label file.

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
        # 允许 "0", "0.0", "0e0" 等表现形式
        c = int(float(parts[0]))
        x, y, w, h = (float(parts[1]), float(parts[2]), float(parts[3]), float(parts[4]))
    except Exception as exc:
        raise ValueError(f"Parse error in line: {line!r}") from exc
    return c, x, y, w, h


def load_classes_from_yaml(data_yaml: Union[str, Path]) -> List[str]:
    """Load class names from a YOLO ``data.yaml`` file.

    从 YOLO 的 ``data.yaml`` 中读取类别名称（兼容列表或索引映射两种写法）。

    Args:
        data_yaml (Union[str, Path]): Path to ``data.yaml``.

    Returns:
        List[str]: Class name list in index order.

    Raises:
        FileNotFoundError: If ``data_yaml`` does not exist.
        RuntimeError: If PyYAML is missing or the YAML does not contain ``names``.
        TypeError: If ``names`` is of an unsupported type.
    """
    p = Path(data_yaml)
    if not p.exists():
        raise FileNotFoundError(f"data.yaml not found: {p}")
    if yaml is None:
        raise RuntimeError("PyYAML is required to parse data.yaml (pip install pyyaml).")

    data = yaml.safe_load(p.read_text(encoding="utf-8"))
    names = data.get("names", None)
    if names is None:
        raise RuntimeError("data.yaml has no 'names' field.")

    if isinstance(names, list):
        return [str(n) for n in names]
    if isinstance(names, dict):
        # 形如 {0: 'cls0', 1: 'cls1', ...}
        # 按键排序后取值
        return [str(names[k]) for k in sorted(names.keys(), key=int)]
    raise TypeError(f"Unsupported 'names' type in data.yaml: {type(names)}")


def lint_split(
    images_dir: Union[str, Path],
    labels_dir: Union[str, Path],
    class_names: Sequence[str],
    min_side: float = 0.0,
    min_area: float = 0.0,
) -> Dict[str, List]:
    """Lint a single split's images and labels.

    对一个 split（如 train/val/test）的图像与标签执行一致性与范围检查。

    Args:
        images_dir (Union[str, Path]): Path to folder containing images.
        labels_dir (Union[str, Path]): Path to folder containing YOLO label files.
        class_names (Sequence[str]): Class name list used to validate class ids.
        min_side (float): Minimum normalized side length threshold for ``w`` and ``h``.
        min_area (float): Minimum normalized area threshold for ``w*h``.

    Returns:
        Dict[str, List]: A report dictionary with keys:
            - ``missing_label_files`` (List[str])
            - ``missing_image_files`` (List[str])
            - ``bad_lines`` (List[Tuple[str, int, str]])
            - ``out_of_range`` (List[Tuple[str, int, Union[int, Tuple[float,float,float,float]]]])
            - ``zero_area`` (List[Tuple[str, int, Tuple[float, float]]])
            - ``too_small`` (List[Tuple[str, int, Tuple[float, float, float]]])  # (w, h, area)

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
    too_small: List[Tuple[str, int, Tuple[float, float, float]]] = []

    # A) image -> label
    for img in img_dir.glob("*"):
        if img.suffix.lower() not in valid_img_exts:
            continue
        lbl = lbl_dir / (img.stem + ".txt")
        if not lbl.exists():
            missing_lbl.append(img.name)

    # B) label -> image, and per-line checks
    for lbl in lbl_dir.glob("*.txt"):
        has_img = any((img_dir / (lbl.stem + ext)).exists() for ext in valid_img_exts)
        if not has_img:
            missing_img.append(lbl.name)

        lines = lbl.read_text(encoding="utf-8").splitlines()
        for i, raw in enumerate(lines, start=1):
            line = raw.strip()
            if not line or line.startswith("#"):  # 忽略空行/注释
                continue
            try:
                c, x, y, w, h = parse_line(line)
            except ValueError:
                bad_lines.append((lbl.name, i, raw))
                continue

            if c < 0 or c >= n_cls:
                out_of_range.append((lbl.name, i, c))
            if not (0.0 <= x <= 1.0 and 0.0 <= y <= 1.0 and 0.0 <= w <= 1.0 and 0.0 <= h <= 1.0):
                out_of_range.append((lbl.name, i, (x, y, w, h)))
            if w <= 0.0 or h <= 0.0:
                zero_area.append((lbl.name, i, (w, h)))
            else:
                area = w * h
                if (min_side > 0 and (w < min_side or h < min_side)) or (min_area > 0 and area < min_area):
                    too_small.append((lbl.name, i, (w, h, area)))

    return {
        "missing_label_files": missing_lbl,
        "missing_image_files": missing_img,
        "bad_lines": bad_lines,
        "out_of_range": out_of_range,
        "zero_area": zero_area,
        "too_small": too_small,
    }


def parse_args() -> argparse.Namespace:
    """Parse CLI arguments.

    解析命令行参数。

    Args:
        None

    Returns:
        argparse.Namespace: 包含 ``images``, ``labels``, ``classes``, ``data_yaml``,
        ``min_side``, ``min_area`` 的解析结果。

    Raises:
        SystemExit: If arguments are invalid (raised by ``argparse``).
    """
    parser = argparse.ArgumentParser(description="Lint YOLO labels")
    parser.add_argument("--images", required=True, type=str, help="Path to images/<split>")
    parser.add_argument("--labels", required=True, type=str, help="Path to labels/<split>")
    parser.add_argument(
        "--classes",
        type=str,
        default="",
        help="Comma-separated class names (e.g. 'plate holder,1d barcode,2d barcode')",
    )
    parser.add_argument(
        "--data-yaml",
        type=str,
        default="",
        help="Path to data.yaml; if provided, classes will be read from it.",
    )
    parser.add_argument("--min-side", type=float, default=0.0, help="Min normalized side length (w or h)")
    parser.add_argument("--min-area", type=float, default=0.0, help="Min normalized area (w*h)")
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

    # 1) 类别来源优先级：--classes > --data-yaml
    class_names: List[str]
    if args.classes.strip():
        class_names = [s.strip() for s in args.classes.split(",") if s.strip()]
    elif args.data_yaml.strip():
        class_names = load_classes_from_yaml(args.data_yaml)
    else:
        raise ValueError("Either --classes or --data-yaml must be provided.")

    # 2) 执行 Lint
    report = lint_split(
        args.images,
        args.labels,
        class_names,
        min_side=float(args.min_side),
        min_area=float(args.min_area),
    )

    # 3) 打印报告（截断显示头几个）
    print("=== LINT REPORT ===")
    for k, v in report.items():
        if isinstance(v, list):
            head = v[:8]
            print(f"{k}: total={len(v)}")
            for item in head:
                print("  -", item)
            if len(v) > len(head):
                print(f"  ... ({len(v) - len(head)} more)")
        else:
            print(f"{k}: n/a")
    print("===================")


if __name__ == "__main__":
    main()
