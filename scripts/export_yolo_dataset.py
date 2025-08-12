#!/usr/bin/env python3
# scripts/export_yolo_dataset.py
"""
Export a dataset from Annotation Backend into a YOLO training folder.
从 Annotation Backend 导出指定数据集为 YOLO 训练目录结构，并自动划分 train/val/test。

Usage:
    python scripts/export_yolo_dataset.py ^
        --dataset-id 1 ^
        --out-dir .\yolo_dataset_ds1 ^
        --val-ratio 0.15 ^
        --test-ratio 0.05 ^
        --api-base http://127.0.0.1:8000
"""

from __future__ import annotations

import os
import argparse
import random
import shutil
from collections import defaultdict
from pathlib import Path
from typing import Dict, List, Sequence, Tuple

import requests
from PIL import Image

# 支持的图片后缀
IMG_EXTS = {".bmp", ".jpg", ".jpeg", ".png"}


def fetch_dataset(api: str, ds_id: int) -> tuple[list[dict], list[dict], dict[int, str], dict[str, int], list[str]]:
    """Fetch images, annotations and label definitions of a dataset.

    从后端拉取图片、标注与标签定义，并构建索引。

    Args:
        api: Annotation Backend API base URL.
        ds_id: Dataset ID.

    Returns:
        A tuple (images, annotations, id2name, name2idx, classes):
            images: 来自 /images 的字典列表。
            annotations: 来自 /annotations 的字典列表。
            id2name: {label_id -> class_name}
            name2idx: {class_name -> class_index}（按 label_id 升序，尽量保持创建顺序）
            classes: 类别名称列表，索引与 name2idx 一致。
    """
    imgs = requests.get(f"{api}/datasets/{ds_id}/images").json()
    anns = requests.get(f"{api}/datasets/{ds_id}/annotations").json()
    labels = requests.get(f"{api}/datasets/{ds_id}/labels").json()

    # 使用 label_id 升序保证与“创建顺序/团队 classes.txt”尽量一致
    labels_sorted = sorted(labels, key=lambda x: x["id"])
    classes = [x["name"] for x in labels_sorted]
    id2name = {x["id"]: x["name"] for x in labels_sorted}
    name2idx = {name: i for i, name in enumerate(classes)}
    return imgs, anns, id2name, name2idx, classes


def split_items(items: Sequence[dict], val_ratio: float, test_ratio: float, seed: int = 42) -> tuple[list[dict], list[dict], list[dict]]:
    """Split items into train/val/test by ratios.

    随机划分 train/val/test（后续可替换为按场景/时间划分）。

    Args:
        items: 输入条目（这里是图片字典列表）。
        val_ratio: 验证集比例。
        test_ratio: 测试集比例。
        seed: 随机种子，保证可复现。

    Returns:
        (train_list, val_list, test_list)
    """
    rng = random.Random(seed)
    items = list(items)
    rng.shuffle(items)
    n = len(items)
    n_test = int(n * test_ratio)
    n_val = int(n * val_ratio)
    test = items[:n_test]
    val = items[n_test:n_test + n_val]
    train = items[n_test + n_val:]
    return train, val, test


def to_yolo_line(box: Tuple[float, float, float, float], img_w: int, img_h: int, cls_idx: int) -> str:
    """Convert pixel bbox (xmin, ymin, xmax, ymax) to YOLO format line.

    将像素坐标边界框转换为 YOLO 文本行。

    Args:
        box: (xmin, ymin, xmax, ymax) in pixels.
        img_w: Image width.
        img_h: Image height.
        cls_idx: YOLO class index.

    Returns:
        A single YOLO line: "cls x_center y_center w h".
    """
    xmin, ymin, xmax, ymax = box
    xcen = (xmin + xmax) / 2.0 / img_w
    ycen = (ymin + ymax) / 2.0 / img_h
    w = (xmax - xmin) / img_w
    h = (ymax - ymin) / img_h
    return f"{cls_idx} {xcen:.6f} {ycen:.6f} {w:.6f} {h:.6f}"


def main() -> None:
    """CLI entry. 命令行入口：解析参数，导出 YOLO 训练目录结构。"""
    parser = argparse.ArgumentParser(description="Export YOLO dataset from backend")
    parser.add_argument("--api-base", default="http://127.0.0.1:8000", type=str, help="Annotation Backend API base URL")
    parser.add_argument("--dataset-id", required=True, type=int, help="Dataset ID in the backend")
    parser.add_argument("--out-dir", required=True, type=str, help="Output directory for YOLO dataset folder")
    parser.add_argument("--val-ratio", default=0.15, type=float, help="Validation ratio (0~1)")
    parser.add_argument("--test-ratio", default=0.05, type=float, help="Test ratio (0~1)")
    parser.add_argument("--copy-images", action="store_true", help="Copy images instead of hardlink (default tries hardlink)")
    parser.add_argument("--seed", default=42, type=int, help="Random seed for split")
    args = parser.parse_args()

    out = Path(args.out_dir)
    for sub in ["images/train", "images/val", "images/test", "labels/train", "labels/val", "labels/test"]:
        (out / sub).mkdir(parents=True, exist_ok=True)

    imgs, anns, id2name, name2idx, classes = fetch_dataset(args.api_base, args.dataset_id)

    # group annotations by image_id
    anns_by_image: Dict[int, List[dict]] = defaultdict(list)
    for a in anns:
        anns_by_image[a["image_id"]].append(a)

    # split images
    train_imgs, val_imgs, test_imgs = split_items(imgs, args.val_ratio, args.test_ratio, seed=args.seed)
    split_map = {
        "train": {x["id"] for x in train_imgs},
        "val": {x["id"] for x in val_imgs},
        "test": {x["id"] for x in test_imgs},
    }

    def place_img(src_path: Path, split: str) -> Path:
        """Place an image into split folder via hardlink/copy. 将图像硬链接或复制到分割目录。"""
        dst = out / "images" / split / src_path.name
        if args.copy_images:
            shutil.copy2(src_path, dst)
        else:
            try:
                os.link(src_path, dst)  # Windows 下同盘符可行
            except Exception:
                shutil.copy2(src_path, dst)
        return dst

    # export images + labels
    exported = 0
    for img in imgs:
        img_id = img["id"]
        src_path = Path(img["file_path"])
        if src_path.suffix.lower() not in IMG_EXTS or not src_path.exists():
            print(f"[WARN] skip non-image or missing: {src_path}")
            continue

        with Image.open(src_path) as im:
            W, H = im.size

        yolo_lines: List[str] = []
        for a in anns_by_image.get(img_id, []):
            cls_name = id2name[a["label_id"]]
            cls_idx = name2idx[cls_name]
            yolo_lines.append(to_yolo_line((a["xmin"], a["ymin"], a["xmax"], a["ymax"]), W, H, cls_idx))

        split = "train" if img_id in split_map["train"] else ("val" if img_id in split_map["val"] else "test")
        dst_img = place_img(src_path, split)
        dst_lbl = out / "labels" / split / (dst_img.stem + ".txt")
        dst_lbl.write_text("\n".join(yolo_lines), encoding="utf-8")
        exported += 1

    yaml_text = (
        f"path: {out.resolve()}\n"
        f"train: images/train\n"
        f"val: images/val\n"
        f"test: images/test\n"
        f"names:\n" + "".join([f"  {i}: {n}\n" for i, n in enumerate(classes)])
    )
    (out / "data.yaml").write_text(yaml_text, encoding="utf-8")

    print(f"[DONE] Exported {exported} images to {out.resolve()}")
    print(f"[INFO] Classes: {classes}")


if __name__ == "__main__":
    main()
