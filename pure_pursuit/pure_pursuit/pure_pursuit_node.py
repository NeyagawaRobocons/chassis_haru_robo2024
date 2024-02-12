#!/usr/bin/env python3
import numpy as np
from numpy.typing import NDArray
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ServerGoalHandle
from tf2_ros import Buffer, TransformListener
from tf2_ros.transform import TransformStamped
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from pure_pursuit.msg import Path2DWithAngles
from pure_pursuit.action import PathAndFeedback

class PurePursuitNode(Node):
    def __init__(self) -> None:
        super().__init__('pure_pursuit_node')
        # トピック，TF，アクションの初期化
        self.odom_sub = self.create_subscription(PoseStamped, '/odometry_pose', self.odom_callback, 10)
        self.mcl_sub = self.create_subscription(PoseWithCovarianceStamped, '/mcl_pose', self.mcl_callback, 10)
        self.action_server = ActionServer(
            self,
            PathAndFeedback,
            'path_and_feedback',
            self.execute_callback
        )
        self.vel_pub = self.create_publisher(Twist, '/robot_vel', 10)
        # パラメータの宣言
        self.declare_parameters(
            namespace='',
            parameters=[
                ('speed', 1.0), # [m/s]
                ('lookahead_distance', 0.5),
                ('path_p_gain', 0.5),
                ('angle_p_gain', 0.1),
                ('angle_i_gain', 0.01),
                ('distance_threshold', 0.2),
            ]
        )
        # パラメータの取得
        self.speed = self.get_parameter('speed').value
        self.lookahead_distance = self.get_parameter('lookahead_distance').value
        self.path_p_gain = self.get_parameter('path_p_gain').value
        self.angle_p_gain = self.get_parameter('angle_p_gain').value
        self.angle_i_gain = self.get_parameter('angle_i_gain').value
        self.distance_threshold = self.get_parameter('distance_threshold').value
        # 動的変数
        self.path_data: NDArray[np.float32] # np.array([[x, y, theta], [x, y, theta]])の形
        self.indices: NDArray[np.int8] # np.array([1, 2, 3])の形
        self.tangents: NDArray[np.float32] # np.array([[0.0, 0.0], [0.0, 0.0]])の形
        self.normals: NDArray[np.float32] # np.array([[0.0, 0.0], [0.0, 0.0]])の形
        self.robot_pose: NDArray[np.float32] # np.array([x, y, theta])の形

    def execute_callback (self, goal_handle: ServerGoalHandle[PathAndFeedback.Goal]) -> None:
        # 経路データの受け取りと格納
        self.path_data = np.array(goal_handle.request.path)
        # 特定の点のインデックスの受け取りと格納
        self.indices = np.array(goal_handle.request.index)
        self.tangents = self.接ベクトルの計算 (self.path_data) # 接ベクトルの計算と格納
        self.normals = self.法線ベクトルの計算 (self.tangents) # 法線ベクトルの計算と格納

    def odometry_callback (self) -> None:
        self.get_logger().debug('odometry_callback')
        self.pose_tf_callback()

    def mcl_callback (self) -> None:
        self.get_logger().debug('mcl_callback')
        self.pose_tf_callback()

    def pose_tf_callback (self) -> None:
        # ToDo: 位置の格納
        lookahead_point: NDArray[np.float32] = self.先行点の計算 (self.robot_pose, self.path_data, self.lookahead_distance)
        closest_point: NDArray[np.float32] = None
        closest_point, closest_index = self.最も近い経路上の点の計算 (self.robot_pose, self.path_data)
        vel_msg = self.速度入力の計算 (
            self.robot_pose, lookahead_point, closest_point, self.lookahead_distance, self.speed)
        # 速度入力パブリッシュ
        self.vel_pub.publish(vel_msg)
        if closest_index in self.indices:
            # フィードバックにインデックスを返す
            pass
        elif self.距離(self.robot_pose, 経路の終点) < self.distance_threshold:
            # 完了処理を行う
            pass

    def 速度入力の計算 (self,
            robot_pose: NDArray[np.float32], 
            lookahead_point: NDArray[np.float32],
            closest_point: NDArray[np.float32],
            lookahead_distance: float,
            speed: float
        ) -> Twist:
        pure_pursuit_vel: NDArray[np.float32] = self.pure_pursuit速度入力の計算 (robot_pose, lookahead_point, lookahead_distance, speed)
        p_control_vel: NDArray[np.float32] = self.経路法線方向のP制御入力の計算 (robot_pose, closest_point)
        omega: float = self.角度PI制御入力の計算 (robot_pose, closest_point)
        vel_msg = Twist()
        vel_msg.linear.x = pure_pursuit_vel[0] + p_control_vel[0]
        vel_msg.linear.x = pure_pursuit_vel[1] + p_control_vel[1]
        vel_msg.angular.z = omega
        return vel_msg

    def 先行点の計算 (self,
            robot_pose: NDArray[np.float32], 
            path_data: NDArray[np.float32], 
            先行点までの距離: float
        ) -> NDArray[np.float32]:
        pass

    def 最も近い経路上の点の計算 (robot_pose: NDArray[np.float32], path_data: NDArray[np.float32]) -> int:
        pass

    def pure_pursuit速度入力の計算 (
            robot_pose: NDArray[np.float32], 
            先行点: NDArray[np.float32], 
            先行点までの距離: float, 
            ロボットの速さ: float
        ) -> NDArray[np.float32]:
        pass

    def 経路法線方向のP制御入力の計算 (robot_pose: NDArray[np.float32], 最も近い経路上の点: int) -> NDArray[np.float32]:
        pass

    def 角度PI制御入力の計算 (robot_pose: NDArray[np.float32], 最も近い経路上の点: int) -> NDArray[np.float32]:
        pass

    def 接ベクトルの計算 (path_data: NDArray[np.float32]) -> NDArray[np.float32]:
        # 接ベクトルの計算
        pass

    def 法線ベクトルの計算 (tangents: NDArray[np.float32]) -> NDArray[np.float32]:
        # 法線ベクトルの計算
        pass

    def 距離 (p1: NDArray[np.float32], p2: NDArray[np.float32]) -> float:
        # 2点間の距離を計算
        return np.linalg.norm(p1 - p2)

def main () -> None:
    rclpy.init()
    node = PurePursuitNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()