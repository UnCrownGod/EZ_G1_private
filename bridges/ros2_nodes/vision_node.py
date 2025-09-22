# File: vision/bridges/ros2_nodes/vision_node.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 vision node with reusable perception pipeline.

This node wraps the perception stack (camera stream, YOLO detection, AprilTag
pose estimation) inside :class:`vision.perception.pipeline.PerceptionPipeline`
so heavy work runs on a background thread. The ROS executor only handles
publishing, keeping callbacks responsive even when inference stalls.

Parameters (declare via launch or CLI):
- config_dir (str): Base directory for YAML configs. Defaults to `<repo>/configs`.
- camera_config (str): Camera YAML file within `config_dir`. Default `cameras.yaml`.
- yolo_config (str): YOLO config YAML. Default `yolo.yaml`.
- apriltag_config (str): AprilTag config YAML. Default `apriltags.yaml`.
- detections_topic (str): JSON detections topic. Default `/vision/detections`.
- detections_typed_topic (str): Structured message topic. Default `/vision/detections_array`.
- tags_topic (str): JSON AprilTag topic. Default `/vision/tag_poses`.
- tags_typed_topic (str): Structured PoseArray topic. Default `/vision/tag_poses_array`.
- timer_period (float): Publish rate in seconds. Default `0.05` (20 Hz).
- frame_id (str): TF frame for published messages. Default `camera_optical_frame`.
- publish_json (bool): Whether to publish JSON String topics. Default `True`.
- publish_typed (bool): Whether to publish structured ROS messages. Default `True`.

Usage:
    ros2 run vision.bridges.ros2_nodes vision_node
    # or via launch file with overridden parameters
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any

import numpy as np
import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node
from std_msgs.msg import Header, String

try:  # Optional vision_msgs (ROS 2 perception interfaces)
    from vision_msgs.msg import (
        BoundingBox2D,
        Detection2D,
        Detection2DArray,
        ObjectHypothesisWithPose,
    )

    HAVE_VISION_MSGS = True
except Exception:  # pragma: no cover - executed when vision_msgs unavailable
    HAVE_VISION_MSGS = False

import yaml

from vision.perception.apriltag_tracker import AprilTagPoseCV2
from vision.perception.calibration_utils import load_cam_from_yaml
from vision.perception.camera_stream import CameraStream
from vision.perception.pipeline import PerceptionPipeline
from vision.perception.yolo_detector import YoloDetector


def _to_builtin(obj: Any) -> Any:
    """Recursively convert numpy/torch objects to JSON-serializable builtins."""

    if hasattr(obj, "tolist"):
        return obj.tolist()
    if hasattr(obj, "item"):
        try:
            return obj.item()
        except Exception:  # pragma: no cover - extremely rare
            pass
    if isinstance(obj, dict):
        return {k: _to_builtin(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_to_builtin(v) for v in obj]
    return obj


def _matrix_to_pose(T: np.ndarray) -> Pose:
    """Convert a 4x4 homogeneous matrix to :class:`geometry_msgs.msg.Pose`."""

    pose = Pose()
    pose.position.x = float(T[0, 3])
    pose.position.y = float(T[1, 3])
    pose.position.z = float(T[2, 3])

    R = np.asarray(T[:3, :3], dtype=float)
    trace = float(R[0, 0] + R[1, 1] + R[2, 2])
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        pose.orientation.w = 0.25 / s
        pose.orientation.x = (R[2, 1] - R[1, 2]) * s
        pose.orientation.y = (R[0, 2] - R[2, 0]) * s
        pose.orientation.z = (R[1, 0] - R[0, 1]) * s
    else:
        idx = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if idx == 0:
            s = 2.0 * np.sqrt(max(R[0, 0] - R[1, 1] - R[2, 2] + 1.0, 1e-9))
            pose.orientation.w = (R[2, 1] - R[1, 2]) / s
            pose.orientation.x = 0.25 * s
            pose.orientation.y = (R[0, 1] + R[1, 0]) / s
            pose.orientation.z = (R[0, 2] + R[2, 0]) / s
        elif idx == 1:
            s = 2.0 * np.sqrt(max(R[1, 1] - R[0, 0] - R[2, 2] + 1.0, 1e-9))
            pose.orientation.w = (R[0, 2] - R[2, 0]) / s
            pose.orientation.x = (R[0, 1] + R[1, 0]) / s
            pose.orientation.y = 0.25 * s
            pose.orientation.z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(max(R[2, 2] - R[0, 0] - R[1, 1] + 1.0, 1e-9))
            pose.orientation.w = (R[1, 0] - R[0, 1]) / s
            pose.orientation.x = (R[0, 2] + R[2, 0]) / s
            pose.orientation.y = (R[1, 2] + R[2, 1]) / s
            pose.orientation.z = 0.25 * s
    return pose


class VisionNode(Node):
    """ROS 2 node that publishes perception outputs for downstream planning."""

    def __init__(self, name: str = "vision_node") -> None:
        super().__init__(name)

        repo_root = Path(__file__).resolve().parents[2]
        default_cfg_dir = repo_root / "configs"

        self.declare_parameter("config_dir", str(default_cfg_dir))
        self.declare_parameter("camera_config", "cameras.yaml")
        self.declare_parameter("yolo_config", "yolo.yaml")
        self.declare_parameter("apriltag_config", "apriltags.yaml")
        self.declare_parameter("detections_topic", "/vision/detections")
        self.declare_parameter("detections_typed_topic", "/vision/detections_array")
        self.declare_parameter("tags_topic", "/vision/tag_poses")
        self.declare_parameter("tags_typed_topic", "/vision/tag_poses_array")
        self.declare_parameter("timer_period", 0.05)
        self.declare_parameter("frame_id", "camera_optical_frame")
        self.declare_parameter("publish_json", True)
        self.declare_parameter("publish_typed", True)

        cfg_dir = Path(self.get_parameter("config_dir").value)
        cam_cfg_name = self.get_parameter("camera_config").value
        yolo_cfg_name = self.get_parameter("yolo_config").value
        tag_cfg_name = self.get_parameter("apriltag_config").value

        cfg_cam, self.K, self.dist, _ = load_cam_from_yaml(str(cfg_dir / cam_cfg_name))
        with open(cfg_dir / yolo_cfg_name, "r", encoding="utf-8") as f_yolo:
            cfg_yolo = yaml.safe_load(f_yolo)
        with open(cfg_dir / tag_cfg_name, "r", encoding="utf-8") as f_tag:
            cfg_tag = yaml.safe_load(f_tag)

        self.frame_id = self.get_parameter("frame_id").value
        self.publish_json = bool(self.get_parameter("publish_json").value)
        wants_typed = bool(self.get_parameter("publish_typed").value)

        detections_topic = self.get_parameter("detections_topic").value
        detections_typed_topic = self.get_parameter("detections_typed_topic").value
        tags_topic = self.get_parameter("tags_topic").value
        tags_typed_topic = self.get_parameter("tags_typed_topic").value

        self.pub_det_json = None
        self.pub_tag_json = None
        if self.publish_json:
            self.pub_det_json = self.create_publisher(String, detections_topic, 10)
            self.pub_tag_json = self.create_publisher(String, tags_topic, 10)

        self.pub_det_typed = None
        self.pub_tag_typed = None
        if wants_typed and HAVE_VISION_MSGS:
            self.pub_det_typed = self.create_publisher(Detection2DArray, detections_typed_topic, 10)
            self.pub_tag_typed = self.create_publisher(PoseArray, tags_typed_topic, 10)
        elif wants_typed and not HAVE_VISION_MSGS:
            self.get_logger().warn(
                "vision_msgs not available - structured topics disabled. Install ros-<distro>-vision-msgs."
            )

        self.detector = YoloDetector(cfg_yolo)
        self.tag_detector = AprilTagPoseCV2(tag_size_m=float(cfg_tag.get("size_m", 0.05)))

        width = int(cfg_cam.get("width", 1280))
        height = int(cfg_cam.get("height", 720))
        source = cfg_cam.get("source", "0")
        self.stream = CameraStream(source, width, height)

        self.pipeline = PerceptionPipeline(
            frame_source=self.stream,
            detector=self.detector,
            tag_detector=self.tag_detector,
            camera_matrix=self.K,
            dist_coeffs=self.dist,
            error_handler=self._on_pipeline_error,
        )
        self.pipeline.start()
        self._last_seq = -1

        timer_period = float(self.get_parameter("timer_period").value)
        self.create_timer(timer_period, self._on_timer)

        self.get_logger().info(
            "VisionNode initialised with camera=%s, YOLO model=%s", source, cfg_yolo.get("model")
        )

    def _on_pipeline_error(self, exc: Exception) -> None:
        self.get_logger().error(f"Perception pipeline error: {exc!r}")

    def _on_timer(self) -> None:
        sample = self.pipeline.latest()
        if sample is None or sample.seq == self._last_seq:
            return
        self._last_seq = sample.seq

        stamp = self.get_clock().now().to_msg()
        stamp_float = stamp.sec + stamp.nanosec * 1e-9

        if self.publish_json:
            dets_payload = {"stamp": stamp_float, "detections": _to_builtin(sample.detections)}
            tags_payload = {
                "stamp": stamp_float,
                "tags": [
                    {"id": int(t.get("id", -1)), "T_cam_tag": _to_builtin(t.get("T_cam_tag"))}
                    for t in sample.tags
                ],
            }
            if self.pub_det_json is not None:
                self.pub_det_json.publish(String(data=json.dumps(dets_payload, ensure_ascii=False)))
            if self.pub_tag_json is not None:
                self.pub_tag_json.publish(String(data=json.dumps(tags_payload, ensure_ascii=False)))

        if self.pub_det_typed is not None:
            header = Header()
            header.stamp = stamp
            header.frame_id = self.frame_id
            array_msg = Detection2DArray()
            array_msg.header = header
            for detection in sample.detections:
                bbox = detection.get("bbox", [0.0, 0.0, 0.0, 0.0])
                x1, y1, x2, y2 = map(float, bbox)
                det_msg = Detection2D()
                det_msg.header = header
                bb = BoundingBox2D()
                bb.center.x = (x1 + x2) / 2.0
                bb.center.y = (y1 + y2) / 2.0
                bb.size_x = max(0.0, x2 - x1)
                bb.size_y = max(0.0, y2 - y1)
                det_msg.bbox = bb
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(detection.get("class_id", ""))
                hyp.hypothesis.score = float(detection.get("score", detection.get("conf", 0.0)))
                det_msg.results.append(hyp)
                array_msg.detections.append(det_msg)
            self.pub_det_typed.publish(array_msg)

        if self.pub_tag_typed is not None:
            pose_array = PoseArray()
            pose_array.header.stamp = stamp
            pose_array.header.frame_id = self.frame_id
            for tag in sample.tags:
                T = np.asarray(tag.get("T_cam_tag"))
                if T.shape != (4, 4):
                    continue
                pose_array.poses.append(_matrix_to_pose(T))
            self.pub_tag_typed.publish(pose_array)

    def destroy_node(self) -> bool:
        try:
            if getattr(self, "pipeline", None):
                self.pipeline.stop()
            if getattr(self, "stream", None):
                try:
                    self.stream.release()
                except Exception:
                    pass
        finally:
            return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
