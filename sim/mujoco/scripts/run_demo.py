# File: vision/sim/mujoco/scripts/run_demo.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Run MuJoCo G1 grasp demo and bridge sensor data to ROS 2."""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import numpy as np

try:
    import mujoco
    import mujoco.viewer
except Exception as exc:  # pragma: no cover
    raise RuntimeError("MuJoCo >=3.1 required. Install via `pip install mujoco`.") from exc

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from cv_bridge import CvBridge
except Exception:
    rclpy = None

import cv2


class CameraPublisher(Node):
    def __init__(self, topic: str = "/sim/camera/front") -> None:
        super().__init__("mujoco_camera_publisher")
        self.publisher = self.create_publisher(Image, topic, 10)
        self.bridge = CvBridge()

    def publish_frame(self, frame: np.ndarray, ts: float) -> None:
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher.publish(msg)


def render_camera(model: mujoco.MjModel, data: mujoco.MjData, width: int, height: int) -> np.ndarray:
    renderer = mujoco.Renderer(model, width, height)
    renderer.update_scene(data, camera="front_cam")
    pixels = renderer.render()
    renderer.free()
    return cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)


def main() -> None:
    parser = argparse.ArgumentParser(description="Run MuJoCo demo scene")
    parser.add_argument("--scene", default="vision/sim/mujoco/scenes/g1_demo.xml")
    parser.add_argument("--publish", action="store_true", help="Publish camera frames over ROS 2")
    args = parser.parse_args()

    xml = Path(args.scene).read_text(encoding="utf-8")
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)

    ros_node = None
    if args.publish:
        if rclpy is None:
            raise RuntimeError("ROS 2 Python packages required for --publish option")
        rclpy.init()
        ros_node = CameraPublisher()

    with mujoco.viewer.launch_passive(model, data) as viewer:
        last = time.time()
        while viewer.is_running():
            step_start = time.time()
            mujoco.mj_step(model, data)
            viewer.sync()

            if args.publish and ros_node is not None:
                frame = render_camera(model, data, 640, 480)
                ros_node.publish_frame(frame, time.time())
                rclpy.spin_once(ros_node, timeout_sec=0.0)

            # maintain real-time
            time.sleep(max(0.0, model.opt.timestep - (time.time() - step_start)))

    if ros_node is not None:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
