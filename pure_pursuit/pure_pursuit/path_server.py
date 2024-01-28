#!/usr/bin/env python3
"""
- description: パスを生成するノード
- input: 経路の点の座標と角度：/point_commandトピック(mecha_control/msg/PointAndMechaStateArray型)
- output: 経路の点の座標と角度：/robot_pathトピック(pure_pursuit/msg/Path2DWithAngles型)
"""
import rclpy
from rclpy.node import Node
from mecha_control.msg import PointAndMechaStateArray
from pure_pursuit.msg import PointAndAngle, Path2DWithAngles
import numpy as np

class PathServerNode(Node):
    def __init__(self):
        super().__init__('path_server')
        self.subscription = self.create_subscription(
            PointAndMechaStateArray, '/point_command', self.path_points_callback, 10)
        self.publisher = self.create_publisher(Path2DWithAngles, '/robot_path', 10)
        self.get_logger().info("path_server_node has been started")

    def path_points_callback(self, msg: PointAndMechaStateArray):
        # パスの点の座標と角度を取得
        path_points = []
        path_angles = []
        for point in msg.points:
            path_points.append([point.x, point.y])
            path_angles.append(point.angle)
        # パスの点の座標と角度をパブリッシュ
        path_msg = Path2DWithAngles()
        path_msg = self.generate_path_data(path_points, path_angles)
        self.publisher.publish(path_msg)
        # self.get_logger().info("path_server_node has published path")

    def generate_path_data(self, path_points, path_angles) -> Path2DWithAngles:
        path_density = 20
        path_msg = Path2DWithAngles()

        for i in range(len(path_points) - 1):
            start_point = path_points[i]
            end_point = path_points[i + 1]

            # ベクトル化された操作で補間値を生成
            x_values = np.linspace(start_point[0], end_point[0], num=path_density)
            y_values = np.linspace(start_point[1], end_point[1], num=path_density)
            angle_values = np.linspace(path_angles[i], path_angles[i + 1], num=path_density)

            # リスト内包表記で補間された点をPath2DWithAnglesメッセージに追加
            path_msg.path_with_angles.extend([
                PointAndAngle(x=x, y=y, theta=theta) for x, y, theta in zip(x_values, y_values, angle_values)
            ])

        return path_msg
    
def main(args=None):
    rclpy.init(args=args)
    path_server_node = PathServerNode()
    rclpy.spin(path_server_node)
    # シャットダウン処理
    path_server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
