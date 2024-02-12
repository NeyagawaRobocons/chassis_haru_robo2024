#!/usr/bin/env python3
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from pure_pursuit.msg import Path2DWithAngles

class PurePursuitNode(Node):
    def __init__(self):
        super().__init__('pure_pursuit_node')
        self.subscription = self.create_subscription(
            PoseStamped, '/robot_pose', self.pose_callback, 10)
        self.publisher = self.create_publisher(Twist, '/robot_vel', 10)
        # Pathメッセージをパブリッシュするためのパブリッシャーの初期化
        self.path_publisher = self.create_publisher(Path, '/robot_path_for_view', 10)

        # 動作制御のサブスクリプション
        self.subscription_control = self.create_subscription(
            Bool, '/robot_control', self.control_callback, 10)
        self.is_active = False  # 初期状態では停止

        self.path_subscriber = self.create_subscription(
            Path2DWithAngles, '/robot_path', self.path_callback, 10)

        # パラメータの宣言
        # この行を変更
        self.declare_parameter('speed', 2.0)
        self.declare_parameter('lookahead_distance', 0.3)

        # パラメータの取得
        # self.on_way_points: list[float] = [[0.0, 0.0], [0.0, 1.0], [1.2, 1.0], [1.2, 2.0]]
        self.speed: float = self.get_parameter('speed').value
        self.lookahead_distance: float = self.get_parameter('lookahead_distance').value

        # メンバ変数
        self.path_points: np.ndarray = np.array([])
        self.tangents: np.ndarray = np.array([])
        self.current_path_index: int = -1 # 現在の経路のインデックス

        # 経路データと接ベクトルの生成
        # self.path_points, self.tangents = self.generate_path_data(self.on_way_points)
        # self.publish_path(self.path_points)

        self.get_logger().info("pure_pursuit_node has been started")

    def control_callback(self, msg: Bool):
        # 動作制御のメッセージに基づいて状態を更新
        # self.get_logger().info("control_callback")
        self.is_active = msg.data

    def path_callback(self, msg: Path2DWithAngles):
        if msg.path_with_angles == []:
            self.get_logger().warn("path_callback: path_with_angles is empty")
            return

        self.path_points = np.array([[point.x, point.y] for point in msg.path_with_angles])
        self.path_angles = np.array([point.theta for point in msg.path_with_angles])

        # 接ベクトルの計算
        tangents = [
            (self.path_points[i + 1] - self.path_points[i]) / np.linalg.norm(self.path_points[i + 1] - self.path_points[i])
            if np.linalg.norm(self.path_points[i + 1] - self.path_points[i]) != 0 else np.zeros(2)
            for i in range(len(self.path_points) - 1)
        ]

        # 最後の点のために最後の接ベクトルを追加
        if len(self.path_points) > 1:
            tangents.append(tangents[-1])

        self.tangents = np.array(tangents)
        
        #　タンジェントの配列長を出力
        self.get_logger().info("path_callback: len(tangents) = %s" % len(self.tangents))

    def pose_callback(self, msg: PoseStamped):
        if self.is_active:
            # 自己位置の取得
            current_point = np.array([msg.pose.position.x, msg.pose.position.y])
            theta = self.get_yaw(msg)

            # 先行点の計算
            self.lookahead_point = self.calculate_lookahead_point(self.path_points, self.tangents, current_point)

            # 速度ベクトルの計算
            velocity_vector = self.calculate_velocity_vector(self.lookahead_point, current_point)

            # velocity_vectorがNoneでないことを確認してからpublish
            if velocity_vector is not None:
                # 速度ベクトルのpublish
                twist_msg = Twist()
                # 速度ベクトルを回転させて格納
                twist_msg.linear.x = velocity_vector[0] * np.cos(theta) + velocity_vector[1] * np.sin(theta)
                twist_msg.linear.y = -velocity_vector[0] * np.sin(theta) + velocity_vector[1] * np.cos(theta)
                self.publisher.publish(twist_msg)
            else:
                # 速度ベクトルがNoneの場合は、ログに警告を出力し、何もしない
                self.get_logger().warn("Velocity vector is None, cannot publish Twist message.")
                # publish Twist message with zero velocity
                twist_msg = Twist()
                twist_msg.linear.x = 0.0
                twist_msg.linear.y = 0.0
                self.publisher.publish(twist_msg)
        else:
            self.get_logger().info("waiting for command")
            # publish Twist message with zero velocity
            twist_msg = Twist()
            twist_msg.linear.x = 0.0
            twist_msg.linear.y = 0.0
            self.publisher.publish(twist_msg)
        self.publish_path(self.path_points)

    def publish_path(self, path_points):
        # 経路データをPathメッセージに変換
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = 'map'  # 経路データがこのフレーム内にあると想定
        
        for point in path_points:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.w = 1.0  # クォータニオンのw値は単位クォータニオン
            path_msg.poses.append(pose)

        self.path_publisher.publish(path_msg)

    def calculate_lookahead_point(
            self, 
            path_points: np.ndarray, 
            tangents: np.ndarray, 
            current_point: np.ndarray) -> np.ndarray:
        """
        経路上の現在地における前方参照点を計算するメソッド
        """
        closest_point = None
        closest_tangent = None
        min_distance = float('inf')

        for i in range(self.current_path_index, len(path_points)):
            path_point = path_points[i]
            tangent = tangents[i]
            to_path_vector = path_point - current_point
            distance_to_path = np.linalg.norm(to_path_vector)

            if distance_to_path < self.lookahead_distance:
                continue

            # ロボットの現在位置からの距離がルックアヘッド距離より短い場合、スキップ
            dot_product = np.dot(to_path_vector, tangent)

            # 内積が正で、かつ最短距離でかつ一つ前の先行点よりもインデックスが大きい点であれば更新
            if dot_product > 0 and distance_to_path < min_distance:
                min_distance = distance_to_path
                closest_point = path_point
                closest_tangent = tangent
                self.current_path_index = i

        # まだ先行点が見つかっていない場合、最も近い点を使用
        if closest_point is None:
            return None

        # 接ベクトルとルックアヘッド距離を用いて先行点を補間
        t = (self.lookahead_distance - min_distance) / np.linalg.norm(closest_tangent)
        lookahead_point = closest_point + t * closest_tangent

        return lookahead_point

    def calculate_velocity_vector(self, lookahead_point: np.ndarray, current_point: np.ndarray) -> np.ndarray:
        # 速度ベクトルの計算
        if lookahead_point is None:
            return None  # 先行点が見つからない場合

        # 速度ベクトルを計算する（先行点への単位ベクトルに速度を乗じる）
        direction_vector = lookahead_point - current_point
        unit_direction_vector = direction_vector / np.linalg.norm(direction_vector)
        velocity_vector = unit_direction_vector * self.speed
        return velocity_vector
    
    def get_yaw(self, pose_stamped):
        # 回転（クォータニオン）からYaw角を取得する関数
        orientation = pose_stamped.pose.orientation
        # 四元数の値
        x = orientation.x
        y = orientation.y
        z = orientation.z
        w = orientation.w

        # 四元数からヨー角を計算
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return yaw

def main(args=None):
    rclpy.init(args=args)
    velocity_calculator = PurePursuitNode()
    rclpy.spin(velocity_calculator)
    velocity_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
