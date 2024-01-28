#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mecha_control.msg import PointAndMechaStateArray, MechaState
import numpy as np

class MechaStatePublisher(Node):
    def __init__(self):
        super().__init__('mecha_state_publisher')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('distance_threshold', 0.5),  # デフォルト値は 0.5
                ('angle_threshold', 0.1),     # デフォルト値は 0.1
            ])
        self.distance_threshold = self.get_parameter('distance_threshold').value
        self.angle_threshold = self.get_parameter('angle_threshold').value

        self.subscription_pose = self.create_subscription(
            PoseStamped,
            'robot_pose',
            self.robot_pose_callback,
            10)
        self.subscription_command = self.create_subscription(
            PointAndMechaStateArray,
            'point_command',
            self.point_command_callback,
            10)
        self.publisher_ = self.create_publisher(MechaState, 'mecha_state', 10)

        self.command_points = []
        self.processed_points = []  # 処理済みのポイントを保存するリスト
        self.current_pose = None

        self.get_logger().info('mecha_state_publisher has been started')

    def robot_pose_callback(self, msg):
        self.current_pose = msg
        self.check_and_publish()

    def point_command_callback(self, msg):
        # 新しいコマンドを受け取ったとき、まだ処理されていないポイントのみを保持する
        self.command_points = [p for p in msg.points if p not in self.processed_points]

    def check_and_publish(self):
        if self.current_pose is None or not self.command_points:
            return

        for point_command in self.command_points:
            if self.is_nearby(self.current_pose, point_command):
                self.publisher_.publish(point_command.command)
                self.processed_points.append(point_command)  # 処理済みリストに追加
                self.command_points.remove(point_command)  # 現在のリストから削除
                break  # ポイントを処理したらループを抜ける

    def get_yaw(self, pose_stamped):
        # 回転（クォータニオン）からYaw角を取得する関数
        orientation = pose_stamped.pose.orientation
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        # 四元数からヨー角を計算
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return yaw

    def is_nearby(self, pose, point_command):
        pose_error = self.calculate_distance(pose.pose.position, point_command)
        yaw = self.get_yaw(pose)
        angle_error = abs(yaw - point_command.angle)
        return pose_error < self.distance_threshold and angle_error < self.angle_threshold

    def calculate_distance(self, pose_position, point_command):
        dx = pose_position.x - point_command.x
        dy = pose_position.y - point_command.y
        return (dx**2 + dy**2)**0.5

def main(args=None):
    rclpy.init(args=args)
    node = MechaStatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
