#!/usr/bin/env python3
# vision/behavior/task_grab_tube.py
"""
Task: grab tube (示例任务流水线)
- 打开相机
- 用 OpenCV ArUco 检测 AprilTag 并靠近/对准
- YOLO 目标检测，规划简化抓取点（地面交点）并执行占位抓取

依赖:
    pip install opencv-contrib-python ultralytics pyyaml numpy
"""
from __future__ import annotations
import time, yaml, numpy as np, cv2
from typing import Dict, Any
from vision.perception.camera_stream import CameraStream
from vision.perception.yolo_detector import YoloDetector
from vision.perception.apriltag_pose import AprilTagPoseCV2
from vision.perception.calibration_utils import load_cam_from_yaml, load_tag_from_yaml
from vision.fusion.pose_projector import bbox_center_to_ray, intersect_with_ground
from vision.control.unitree_sdk2_client import UnitreeSDK2Client
from vision.control.navigator import Navigator, Limits
from vision.control.aligner import Aligner
from vision.control.grasp_planner import plan_flat_grasp
from vision.behavior.states import State
from pathlib import Path

class GrabTubeTask:
    def __init__(self, configs_dir: str="vision/configs"):
        cam_yaml = str(Path(configs_dir) / "camera.yaml")  # 你的现有文件名
        if not Path(cam_yaml).exists():
            # 兼容我之前的示例命名
            cam_yaml = str(Path(configs_dir) / "cameras.yaml")

        self.cfg_robot = yaml.safe_load(open(f"{configs_dir}/robot.yaml","r",encoding="utf-8"))
        self.cam_cfg, self.K, self.dist, self.T_base_cam = load_cam_from_yaml(cam_yaml)
        self.family, self.tag_size_m, self.valid_tag_ids = load_tag_from_yaml(cam_yaml, Path(configs_dir,"apriltags.yaml"))
        self.cfg_yolo = yaml.safe_load(open(f"{configs_dir}/yolo.yaml","r",encoding="utf-8"))
        self.cfg_task = yaml.safe_load(open(f"{configs_dir}/tasks.yaml","r",encoding="utf-8"))["grab_tube"]

        self.client = UnitreeSDK2Client(self.cfg_robot.get("sdk2",{}).get("endpoint"))
        self.det = YoloDetector(self.cfg_yolo)
        self.tag = AprilTagPoseCV2(tag_size_m=self.tag_size_m, family="DICT_APRILTAG_36h11")
        src = self.cam_cfg.get("source", "0")
        w, h = int(self.cam_cfg.get("width", 1280)), int(self.cam_cfg.get("height", 720))
        self.stream = CameraStream(src, w, h)

        self.nav = Navigator(self.client, Limits(**{"vx":0.4,"vy":0.2,"wz":0.8}), self.cfg_task.get("stop_dist",0.4))
        self.align = Aligner()

    def run(self):
        state = State.SEEK_TAG
        t0 = time.time()
        self.client.stand()
        for frame, ts in self.stream:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            h,w = gray.shape

            if state == State.SEEK_TAG:
                tags = self.tag.detect(gray, self.K, self.dist, valid_ids=self.valid_tag_ids)
                found = next((t for t in tags if t["id"]==self.cfg_task["seek_tag_id"]), None)
                if found:
                    state = State.GO; t0 = time.time()
                elif time.time()-t0 > self.cfg_task["timeouts"]["seek_tag_s"]:
                    print("[FAIL] tag not found in time"); return False
                continue

            if state == State.GO:
                done = self.nav.go_to(1.2, 0.0)  # 简化：向前走近
                if done: state = State.ALIGN; t0=time.time()
                continue

            if state == State.ALIGN:
                tags = self.tag.detect(gray, self.K, self.dist, valid_ids=self.valid_tag_ids)
                tgt = next((t for t in tags if t["id"]==self.cfg_task["seek_tag_id"]), None)
                if not tgt:
                    if time.time()-t0>self.cfg_task["timeouts"]["align_s"]: print("[FAIL] lost tag"); return False
                    continue
                u = tgt["corners"][:,0].mean()
                vy,wz = self.align.cmd(w,h,u)
                self.client.go(0.0, vy, wz, 0.2)
                if abs(u - w/2) < 10:
                    state = State.DETECT; t0=time.time()
                continue

            if state == State.DETECT:
                dets = self.det.infer(frame)
                goal = [d for d in dets if d["class_name"]==self.cfg_task["detect_class"]]
                if goal:
                    bbox = goal[0]["bbox"]
                    ray = bbox_center_to_ray(bbox, self.K)
                    p_base = intersect_with_ground(ray, self.T_base_cam, ground_z_in_base=0.0)
                    if p_base is not None:
                        p_app, p_grasp = plan_flat_grasp(np.array(p_base), self.cfg_task.get("approach_height",0.1))
                        print("[GRASP_PLAN]", p_app, p_grasp)
                        self.client.stop()
                        self.client.gripper_open()
                        # TODO: 接机械臂 SDK 后到位 -> close
                        self.client.gripper_close()
                        print("[TASK] done")
                        return True
                elif time.time()-t0 > self.cfg_task["timeouts"]["detect_s"]:
                    print("[FAIL] object not detected in time"); return False
                continue

        return False
