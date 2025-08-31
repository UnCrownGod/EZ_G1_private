# File: vision/bridges/ros2_nodes/vision_node.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 node for vision outputs.
ROS2 视觉节点：发布 /vision/detections（YOLO）与 /vision/tag_poses（AprilTag），
并可订阅触发拍照/重置等服务。

Requires:
    pip install rclpy
"""

# 需安装 ROS2 与 rclpy。发布 /vision/detections 与 /vision/tag_poses
from __future__ import annotations
import rclpy, json, cv2
from rclpy.node import Node
from std_msgs.msg import String
from vision.perception.camera_stream import CameraStream
from vision.perception.yolo_detector import YoloDetector
from vision.perception.calibration_utils import load_cam_from_yaml
from vision.perception.apriltag_tracker import AprilTagTracker
import yaml

class VisionNode(Node):
    def __init__(self):
        super().__init__("vision_node")
        self.pub_det = self.create_publisher(String, "/vision/detections", 10)
        self.pub_tag = self.create_publisher(String, "/vision/tag_poses", 10)
        cfg_cam, self.K, self.dist, _ = load_cam_from_yaml("vision/configs/cameras.yaml")
        cfg_yolo = yaml.safe_load(open("vision/configs/yolo.yaml","r",encoding="utf-8"))
        cfg_tag  = yaml.safe_load(open("vision/configs/apriltags.yaml","r",encoding="utf-8"))
        self.det = YoloDetector(cfg_yolo)
        self.tag = AprilTagTracker(cfg_tag["family"], cfg_tag["size_m"])
        self.stream = CameraStream(cfg_cam["source"], cfg_cam["width"], cfg_cam["height"])
        self.create_timer(0.05, self.tick)

    def tick(self):
        try:
            frame, ts = next(iter(self.stream))
        except StopIteration:
            return
        dets = self.det.infer(frame)
        self.pub_det.publish(String(data=json.dumps(dets)))
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        tags = self.tag.detect(gray, self.K, self.dist)
        # numpy -> list
        tags_s = []
        for t in tags:
            T = t["T_cam_tag"].tolist()
            tags_s.append({"id": t["id"], "T_cam_tag": T})
        self.pub_tag.publish(String(data=json.dumps(tags_s)))

def main():
    rclpy.init()
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
