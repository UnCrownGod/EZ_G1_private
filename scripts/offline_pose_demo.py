#!/usr/bin/env python3
# scripts/offline_pose_demo.py
"""
Offline pose demo: image -> tag pose (cam) -> base pose -> grasp poses.
离线演示：图片 → Tag 位姿（相机系）→ 基座系 → 抓取位姿。

Usage:
  python scripts/offline_pose_demo.py ^
      --image "D:/work/vision/pics/train1/21411753181149_.pic.jpg" ^
      --camera-config .\configs\camera.yaml ^
      --frames-config .\configs\frames.yaml ^
      --grasp-config .\configs\grasp.yaml ^
      [--yolo-weights runs/detect/w_lid_v1/weights/best.pt]

Notes:
  - If --yolo-weights provided, we keep top-1 'plate holder' box as ROI for tag.
  - 含 YOLO 时，将使用板架框作为 ROI 加速/稳健化（可选）。
"""

from __future__ import annotations

import argparse
from typing import Optional, Tuple
import numpy as np
import cv2

from robot.perception.apriltag_pose import CameraModel, detect_apriltags
from robot.perception.yolo_detect import YoloDetector
from robot.planning.handeye import load_yaml, compose
from robot.planning.grasp_planner import make_grasp_from_tag


def crop_to_roi(bgr: np.ndarray, xyxy: Tuple[float, float, float, float]) -> tuple[np.ndarray, tuple[int,int,int,int]]:
    """Crop image to ROI with clamping.

    按 ROI 裁剪并返回偏移。

    Args:
        bgr (np.ndarray): Input image.
        xyxy (Tuple[float,float,float,float]): ROI box.

    Returns:
        tuple[np.ndarray, tuple[int,int,int,int]]: (cropped, (x0,y0,x1,y1))

    Raises:
        ValueError: If invalid sizes.
    """
    H, W = bgr.shape[:2]
    x1, y1, x2, y2 = map(int, xyxy)
    x1 = max(0, min(W - 1, x1)); x2 = max(0, min(W, x2))
    y1 = max(0, min(H - 1, y1)); y2 = max(0, min(H, y2))
    if x2 <= x1 or y2 <= y1:
        raise ValueError("invalid ROI")
    return bgr[y1:y2, x1:x2].copy(), (x1, y1, x2, y2)


def parse_args() -> argparse.Namespace:
    """Parse CLI args.

    解析命令行参数。

    Args:
        None

    Returns:
        argparse.Namespace: Parsed args.
    """
    p = argparse.ArgumentParser(description="Offline rack pose demo")
    p.add_argument("--image", required=True, type=str)
    p.add_argument("--camera-config", required=True, type=str)
    p.add_argument("--frames-config", required=True, type=str)
    p.add_argument("--grasp-config", required=True, type=str)
    p.add_argument("--yolo-weights", default=None, type=str)
    return p.parse_args()


def main() -> None:
    """Main.

    主流程。

    Args:
        None

    Returns:
        None

    Raises:
        RuntimeError: On missing tag/pose.
    """
    args = parse_args()
    cam_cfg = load_yaml(args.camera_config)
    frm_cfg = load_yaml(args.frames_config)
    gsp_cfg = load_yaml(args.grasp_config)

    K = np.array(cam_cfg["K"], dtype=float)
    dist = np.array(cam_cfg["dist"], dtype=float).reshape(-1)
    cam = CameraModel(K=K, dist=dist)
    tag_size_m = float(cam_cfg["tag_size_m"])
    valid_ids = list(map(int, cam_cfg.get("valid_tag_ids", [])))

    T_base_cam = np.array(frm_cfg["T_base_cam"], dtype=float)
    T_base_place = np.array(frm_cfg["T_base_place"], dtype=float)

    bgr = cv2.imread(args.image)
    if bgr is None:
        raise RuntimeError(f"failed to read image: {args.image}")

    # Optional YOLO ROI
    roi_img = bgr
    roi_offset = (0, 0)
    if args.yolo_weights:
        det = YoloDetector(args.yolo_weights, class_filter=["plate holder"])
        ds = det.infer(bgr, conf=0.25)
        if ds:
            roi_img, (x1, y1, x2, y2) = crop_to_roi(bgr, ds[0].xyxy)
            roi_offset = (x1, y1)
            print(f"[INFO] ROI from YOLO: {ds[0]}")
        else:
            print("[INFO] YOLO found nothing; using full frame")

    # Detect tags in ROI
    tags = detect_apriltags(roi_img, cam, tag_size_m, valid_ids=valid_ids or None)
    if not tags:
        raise RuntimeError("no AprilTag found")

    tag_id, T_cam_tag_local = tags[0]
    # Offset correction: rotation unaffected; translation must add ROI offset in pixels — NOT NEEDED
    # because pose is metric from solvePnP; cropping does not change camera pose solution.

    T_base_tag = compose(T_base_cam, T_cam_tag_local)
    pre, grasp = make_grasp_from_tag(
        T_base_tag,
        approach_offset_m=float(gsp_cfg["approach_offset_m"]),
        grasp_offset_m=float(gsp_cfg["grasp_offset_m"]),
        rpy_tool_rad=tuple(map(float, gsp_cfg["rpy_tool_rad"])),
    )

    np.set_printoptions(precision=4, suppress=True)
    print(f"[OK] Tag ID: {tag_id}")
    print("[T_base_tag]\n", T_base_tag)
    print("[T_base_pregrasp]\n", pre)
    print("[T_base_grasp]\n", grasp)
    print("[T_base_place]\n", T_base_place)
    print("[DONE] Ready to feed poses to G1Adapter on-site.")

if __name__ == "__main__":
    main()
