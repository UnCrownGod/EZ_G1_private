# File: vision/bridges/ros2_nodes/vision_node.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 视觉节点

发布两类主题：
- /vision/detections: YOLO 检测结果（JSON）
- /vision/tag_poses:  AprilTag 位姿（JSON）

依赖：
    pip install rclpy opencv-contrib-python pyyamlK
    # 若在 ROS 2 Humble: sudo apt install ros-humble-vision-opencv

约定：
- 配置目录位于仓库根下 vision/configs/
- 相机配置通过 `calibration_utils.load_cam_from_yaml` 读取
"""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict, Iterable, List, Tuple

import cv2
import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String

from vision.perception.apriltag_tracker import AprilTagPoseCV2
from vision.perception.calibration_utils import load_cam_from_yaml
from vision.perception.camera_stream import CameraStream
from vision.perception.yolo_detector import YoloDetector


def _to_builtin(obj: Any) -> Any:
    """将 numpy、torch 等对象递归转换为内置类型，便于 JSON 序列化。

    Args:
        obj: 任意对象，可能包含 numpy.ndarray、numpy.generic、dict、list、tuple 等。

    Returns:
        转换后的 Python 内置类型对象（int、float、list、dict 等）。

    Raises:
        TypeError: 当对象无法被识别或转换时抛出（极少见，通常不会触发）。
    """
    # numpy.ndarray / torch.Tensor 等
    if hasattr(obj, "tolist"):
        return obj.tolist()
    # numpy scalar
    if hasattr(obj, "item"):
        try:
            return obj.item()
        except Exception:
            pass
    if isinstance(obj, dict):
        return {k: _to_builtin(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [_to_builtin(v) for v in obj]
    return obj


class VisionNode(Node):
    """视觉发布节点。

    周期性从相机读取帧，运行 YOLO 与 AprilTag 检测，并以 JSON 文本发布到 ROS 2 主题。

    Args:
        name: 节点名。默认 "vision_node"。

    Attributes:
        pub_det: /vision/detections 发布者。
        pub_tag: /vision/tag_poses 发布者。
        det: YOLO 检测器实例。
        tag: AprilTag 求位姿实例。
        stream: 相机数据流。
        K: 相机内参矩阵 (3x3)。
        dist: 畸变参数向量。
        _iter: 相机帧迭代器。
        cfg_dir: 配置目录路径。
    """

    def __init__(self, name: str = "vision_node") -> None:
        super().__init__(name)

        # 统一配置路径，避免工作目录变动带来的相对路径问题
        # vision/bridges/ros2_nodes/ -> parents[2] == vision/
        root: Path = Path(__file__).resolve().parents[2]
        self.cfg_dir: Path = root / "configs"

        # ROS 2 发布者
        self.pub_det = self.create_publisher(String, "/vision/detections", 10)
        self.pub_tag = self.create_publisher(String, "/vision/tag_poses", 10)

        # 读取配置
        cfg_cam, self.K, self.dist, _ = load_cam_from_yaml(str(self.cfg_dir / "cameras.yaml"))
        with open(self.cfg_dir / "yolo.yaml", "r", encoding="utf-8") as f:
            cfg_yolo = yaml.safe_load(f)
        with open(self.cfg_dir / "apriltags.yaml", "r", encoding="utf-8") as f:
            cfg_tag = yaml.safe_load(f)

        # 初始化感知模块
        self.det = YoloDetector(cfg_yolo)
        self.tag = AprilTagPoseCV2(tag_size_m=cfg_tag["size_m"])

        # 相机流与迭代器
        self.stream = CameraStream(cfg_cam["source"], cfg_cam["width"], cfg_cam["height"])
        self._iter: Iterable[Tuple[Any, float]] = iter(self.stream)

        # 定时器：50ms -> 20Hz
        self.create_timer(0.05, self.tick)

    def tick(self) -> None:
        """单次循环：取帧 -> YOLO -> AprilTag -> 发布两条消息。

        Args:
            None

        Returns:
            None

        Raises:
            RuntimeError: 当底层读取或推理出现严重异常时可能抛出。默认捕获并记录日志，不中断节点。
        """
        try:
            frame, ts = next(self._iter, (None, None))
            if frame is None:
                # 断流时重建迭代器，等待下一帧
                self._iter = iter(self.stream)
                return

            # 统一时间戳（纳秒）
            stamp_ns: int = self.get_clock().now().nanoseconds

            # YOLO 检测并发布
            dets_raw: List[Dict[str, Any]] = self.det.infer(frame)  # 约定输出为可序列化字典列表
            dets: List[Dict[str, Any]] = _to_builtin(dets_raw)
            msg_det = {"t": stamp_ns, "dets": dets}
            self.pub_det.publish(String(data=json.dumps(msg_det)))

            # AprilTag 位姿并发布
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            tags_raw = self.tag.detect(gray, self.K, self.dist)  # 列表[{"id":int,"T_cam_tag":np.ndarray(4x4)},...]
            tags_serialized: List[Dict[str, Any]] = []
            for t in tags_raw:
                tags_serialized.append(
                    {"id": int(t["id"]), "T_cam_tag": _to_builtin(t["T_cam_tag"])}
                )
            msg_tag = {"t": stamp_ns, "tags": tags_serialized}
            self.pub_tag.publish(String(data=json.dumps(msg_tag)))

        except StopIteration:
            # 流结束，重建迭代器
            self._iter = iter(self.stream)
        except Exception as e:
            # 记录错误，避免节点退出
            self.get_logger().error(f"tick error: {e!r}")

    def destroy_node(self) -> bool:
        """释放资源并销毁节点。

        Returns:
            bool: True 表示销毁成功。

        Raises:
            None
        """
        try:
            # 如 CameraStream 支持上下文清理，可在此补充
            return super().destroy_node()
        except Exception:
            return False


def main() -> None:
    """入口函数。初始化 ROS 2 并运行节点。

    Args:
        None

    Returns:
        None

    Raises:
        RuntimeError: 当 rclpy 初始化或 spin 过程出现不可恢复错误时。
    """
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
