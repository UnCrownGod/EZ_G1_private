# File: vision/bridges/ros2_nodes/robot_node.py
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ROS2 node bridging Unitree SDK2.
ROS2 机器人桥：订阅 /cmd_vel 与抓手指令，将其转发到 UnitreeSDK2Client；回传状态到 /robot/status。

Requires:
    pip install rclpy
"""

# 把 /cmd_vel 转成 SDK2 客户端调用（若官方仿真插件已支持 /cmd_vel，可不需要本节点）
from __future__ import annotations
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from vision.control.unitree_sdk2_client import UnitreeSDK2Client

class RobotNode(Node):
    def __init__(self):
        super().__init__("robot_node")
        self.client = UnitreeSDK2Client()
        self.sub = self.create_subscription(Twist, "/cmd_vel", self.on_cmd, 10)

    def on_cmd(self, msg: Twist):
        vx = float(msg.linear.x); vy=float(msg.linear.y); wz=float(msg.angular.z)
        self.client.go(vx, vy, wz, duration=0.1)

def main():
    rclpy.init()
    node = RobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
