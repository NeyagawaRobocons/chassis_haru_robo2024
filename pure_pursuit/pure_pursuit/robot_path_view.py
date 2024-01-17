#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
from nav_msgs.msg import Path

class PathView(Node):
    def __init__(self):
        super().__init__('path_view')
        self.subscription = self.create_subscription(
            PoseStamped, '/robot_pose', self.pose_callback, 10)
        # 動作制御のサブスクリプション
        self.subscription_control = self.create_subscription(
            Bool, '/robot_control', self.control_callback, 10)
        self.path_publisher = self.create_publisher(Path, '/robot_path', 10)

        self.is_active = False  # 初期状態では停止
        self.path_data = Path() # 経路データを格納するリスト
        self.path_data = [] # 経路データを格納するリスト

        self.get_logger().info("path_view_node has been started")

    def control_callback(self, msg: Bool):
        # 動作制御のメッセージに基づいて状態を更新
        # self.get_logger().info("control_callback")
        self.is_active = msg.data
        if self.is_active:
            self.path_data = [] # reset path data

    def pose_callback(self, msg: PoseStamped):
        if self.is_active:
            self.path_data.append(msg)
            self.path_publisher.publish(self.path_data)
        else:
            self.get_logger().info("waiting for command")

def main(args=None):
    rclpy.init(args=args)
    path_view = PathView()
    rclpy.spin(path_view)
    path_view.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
