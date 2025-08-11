#!/usr/bin/env python3
# scripts/import_yolo_annotations.py

"""
Import YOLO-format annotations into Annotation Backend service.
将 YOLO 格式的标注数据导入 Annotation Backend 服务。

Usage:
    python import_yolo_annotations.py \
        --dataset-id 1 \
        --img-dir D:/work/vision/pics/QRcode_bottom \
        --classes-file D:/work/vision/pics/QRcode_bottom/classes.txt \
        --api-base http://127.0.0.1:8000
"""

import os
import glob
import argparse
import requests
from PIL import Image
from typing import List, Dict

# 支持的图片后缀列表
SUPPORTED_EXT = [".bmp", ".jpg", ".jpeg", ".png"]


def load_classes(classes_file: str) -> List[str]:
    """Load class names from classes.txt 文件.

    从 classes.txt 中加载类别名称列表。

    Args:
        classes_file (str): Path to the classes.txt file.

    Returns:
        List[str]: List of class names.
    """
    with open(classes_file, encoding="utf-8") as f:
        return [line.strip() for line in f if line.strip()]


def ensure_labels_exist(api_base: str, dataset_id: int, classes: List[str]) -> Dict[str, int]:
    """Ensure each class in classes exists as a label in the backend.

    确保每个类别在后端都有对应的 LabelDefinition，没有则创建。

    Args:
        api_base (str): Base URL of the Annotation Backend API.
        dataset_id (int): 数据集 ID.
        classes (List[str]): List of class names.

    Returns:
        Dict[str, int]: Mapping from class name to label_id.
    """
    url = f"{api_base}/datasets/{dataset_id}/labels"
    resp = requests.get(url)
    resp.raise_for_status()
    existing = {item["name"]: item["id"] for item in resp.json()}

    for cls in classes:
        if cls not in existing:
            r = requests.post(url, json={"name": cls})
            r.raise_for_status()
            lbl = r.json()
            existing[cls] = lbl["id"]
            print(f"[LABEL CREATED] {cls} -> id {lbl['id']}")

    return existing


def get_image_id_map(api_base: str, dataset_id: int) -> Dict[str, int]:
    """Build a mapping from image filename to image_id in the backend.

    通过调用 /images 接口，获取所有图片记录并建立 filename->id 映射。

    Args:
        api_base (str): Base URL of the Annotation Backend API.
        dataset_id (int): 数据集 ID.

    Returns:
        Dict[str, int]: Mapping from filename to image_id.
    """
    url = f"{api_base}/datasets/{dataset_id}/images"
    resp = requests.get(url)
    resp.raise_for_status()
    images = resp.json()
    return {
        os.path.basename(item["file_path"]): item["id"]
        for item in images
    }


def import_annotations(
    api_base: str,
    dataset_id: int,
    img_dir: str,
    name2id: Dict[str, int],
    classes: List[str],
    classes2id: Dict[str, int],
) -> None:
    """Parse YOLO .txt files and post annotations to the backend.

    解析 YOLO 格式的 .txt，计算像素坐标，并调用 /annotations 接口创建记录。

    Args:
        api_base (str): Base URL of the Annotation Backend API.
        dataset_id (int): 数据集 ID.
        img_dir (str): 图片与 YOLO 标注所在目录.
        name2id (Dict[str, int]): filename->image_id 映射.
        classes (List[str]): 类别名称列表，顺序对应 YOLO 索引.
        classes2id (Dict[str, int]): class name->label_id 映射.
    """
    for txt_path in glob.glob(os.path.join(img_dir, "*.txt")):
        stem = os.path.splitext(os.path.basename(txt_path))[0]
        # 找到对应图片文件（支持多种后缀）
        img_path = None
        for ext in SUPPORTED_EXT:
            candidate = os.path.join(img_dir, stem + ext)
            if os.path.exists(candidate):
                img_path = candidate
                break
        if img_path is None:
            print(f"[WARN] 找不到对应图片，跳过: {stem} (支持后缀: {SUPPORTED_EXT})")
            continue

        img_name = os.path.basename(img_path)
        if img_name not in name2id:
            print(f"[WARN] 图片未导入到后端，跳过: {img_name}")
            continue
        image_id = name2id[img_name]

        # 获取图片尺寸
        with Image.open(img_path) as img:
            W, H = img.size

        # 解析 YOLO 标注并提交
        with open(txt_path, encoding="utf-8") as f:
            for line in f:
                parts = line.strip().split()
                if len(parts) != 5:
                    continue
                cls_idx, xcen, ycen, w_norm, h_norm = map(float, parts)
                xmin = (xcen - w_norm / 2) * W
                ymin = (ycen - h_norm / 2) * H
                xmax = xmin + w_norm * W
                ymax = ymin + h_norm * H

                class_name = classes[int(cls_idx)]
                label_id = classes2id[class_name]
                payload = {
                    "image_id": image_id,
                    "label_id": label_id,
                    "xmin": xmin,
                    "ymin": ymin,
                    "xmax": xmax,
                    "ymax": ymax,
                }
                r = requests.post(
                    f"{api_base}/datasets/{dataset_id}/annotations",
                    json=payload,
                )
                if r.status_code == 201:
                    print(f"[OK] {img_name}: {class_name} ({xmin:.1f},{ymin:.1f},{xmax:.1f},{ymax:.1f})")
                else:
                    print(f"[ERR] {img_name} -> {r.status_code} {r.text}")


def main():
    """脚本入口：解析参数，依次执行标签和标注导入流程。"""
    parser = argparse.ArgumentParser(
        description="Import YOLO annotations into Annotation Backend"
    )
    parser.add_argument(
        "--api-base", type=str, default="http://127.0.0.1:8000",
        help="Annotation Backend API base URL"
    )
    parser.add_argument(
        "--dataset-id", type=int, required=True,
        help="Dataset ID in the backend"
    )
    parser.add_argument(
        "--img-dir", type=str, required=True,
        help="Directory containing images and YOLO .txt files"
    )
    parser.add_argument(
        "--classes-file", type=str, required=True,
        help="Path to classes.txt"
    )
    args = parser.parse_args()

    # 1. Load classes and ensure labels exist
    classes = load_classes(args.classes_file)
    classes2id = ensure_labels_exist(args.api_base, args.dataset_id, classes)

    # 2. Build image filename->id map
    name2id = get_image_id_map(args.api_base, args.dataset_id)

    # 3. Import YOLO annotations
    import_annotations(
        args.api_base,
        args.dataset_id,
        args.img_dir,
        name2id,
        classes,
        classes2id,
    )


if __name__ == "__main__":
    main()
