# File: vision/bridges/ros2_nodes/robot_node.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 bridge for Unitree G1 velocity and gripper control.

This node subscribes to a configurable `cmd_vel` topic, clamps velocities to
safe limits, forwards them to the Unitree SDK2 client, and provides a watchdog
that issues a stop command when velocity commands time out.

Parameters:
- endpoint (str): Optional SDK2 endpoint URI or connection hint.
- cmd_vel_topic (str): Topic to subscribe for Twist commands (default `/cmd_vel`).
- command_timeout (float): Seconds before automatic stop (default 0.5).
- send_duration (float): Duration forwarded to SDK `go` command (default 0.2).
- max_vx, max_vy, max_wz (float): Velocity limits for safety.
- auto_stop (bool): Whether to use the watchdog timer (default True).
"""

from __future__ import annotations




import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

from vision.control.unitree_sdk2_client import UnitreeSDK2Client


def _clamp(value: float, limit: float) -> float:
    limit = abs(limit)
    if limit <= 0.0:
        return 0.0
    return max(-limit, min(limit, value))


class RobotNode(Node):
    """Thin ROS 2 wrapper to expose Unitree SDK2 velocity interface."""

    def __init__(self) -> None:
        super().__init__("robot_node")

        self.declare_parameter("endpoint", "")
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter("command_timeout", 0.5)
        self.declare_parameter("send_duration", 0.2)
        self.declare_parameter("max_vx", 0.6)
        self.declare_parameter("max_vy", 0.3)
        self.declare_parameter("max_wz", 1.0)
        self.declare_parameter("auto_stop", True)

        endpoint = self.get_parameter("endpoint").value
        self.client = UnitreeSDK2Client(endpoint=endpoint)

        self.max_vx = float(self.get_parameter("max_vx").value)
        self.max_vy = float(self.get_parameter("max_vy").value)
        self.max_wz = float(self.get_parameter("max_wz").value)
        self.command_duration = float(self.get_parameter("send_duration").value)
        self.command_timeout = float(self.get_parameter("command_timeout").value)
        self.auto_stop = bool(self.get_parameter("auto_stop").value)

        topic = self.get_parameter("cmd_vel_topic").value
        self.sub_cmd = self.create_subscription(Twist, topic, self.on_cmd, 10)

        self.status_pub = self.create_publisher(Bool, "/robot/connected", 1)
        self.status_pub.publish(Bool(data=not self.client.mock))

        self._last_command_time = self.get_clock().now()
        self._watchdog = None
        if self.auto_stop:
            self._watchdog = self.create_timer(0.1, self._on_watchdog)

        self.get_logger().info(
            "RobotNode initialised (endpoint=%s, cmd_vel_topic=%s, auto_stop=%s)",
            endpoint or "mock",
            topic,
            self.auto_stop,
        )

    def on_cmd(self, msg: Twist) -> None:
        vx = _clamp(float(msg.linear.x), self.max_vx)
        vy = _clamp(float(msg.linear.y), self.max_vy)
        wz = _clamp(float(msg.angular.z), self.max_wz)

        if vx == vy == wz == 0.0 and not self.auto_stop:
            self.client.stop()
        else:
            self.client.go(vx, vy, wz, duration=self.command_duration)

        self._last_command_time = self.get_clock().now()

    def _on_watchdog(self) -> None:
        now = self.get_clock().now()
        if (now - self._last_command_time) > Duration(seconds=self.command_timeout):
            self.client.stop()
            self._last_command_time = now

    def destroy_node(self) -> bool:
        try:
            if hasattr(self, "_watchdog") and self._watchdog:
                self._watchdog.cancel()
        finally:
            return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = RobotNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
