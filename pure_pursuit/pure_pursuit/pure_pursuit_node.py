#!/usr/bin/env python3
import numpy as np
from numpy.typing import NDArray
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import PoseStamped, Twist
from pure_pursuit.msg import Path2DWithAngles
from pure_pursuit.action import PathAndFeedback

class PurePursuitNode(Node):
    def __init__(self) -> None:
        super().__init__('pure_pursuit_node')
        # トピック, アクションの初期化
        self.pose_sub = self.create_subscription(PoseStamped, '/robot_pose', self.pose_callback, 10)
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
        self.path_data: NDArray[np.float64] = None # np.array([[x, y, theta], [x, y, theta]])の形
        self.indices: NDArray[np.int8] = np.array([]) # np.array([1, 2, 3])の形
        self.tangents: NDArray[np.float64] = None # np.array([[0.0, 0.0], [0.0, 0.0]])の形
        self.max_angle: float = 0.0
        self.robot_pose: NDArray[np.float64] = np.array([0.0, 0.0, 0.0]) # np.array([x, y, theta])の形
        self.pure_pursuit_vel: NDArray[np.float64] = np.array([0.0, 0.0]) # np.array([v_x, v_y])の形
        self.current_path_index: int = -1
        self.closest_index: int = -1
        self.current_speed = self.speed
        self.current_lookahead_distance = self.lookahead_distance
        self.goal_handle = None # GoalHandleの初期化
        self.result_msg = None # Resultの初期化
        self.start_pure_pursuit = False # pure pursuitの開始フラグの初期化
        self.completed = False # 完了フラグの初期化
        self.goal_event = threading.Event()

        self.get_logger().info("pure_pursuit_node has been initialized")

    async def execute_callback (self, goal_handle) -> None:
        self.goal_event.clear()  # イベントフラグをリセット
        self.completed = False # 完了フラグをリセット
        self.result_msg = None # Resultをリセット
        self.start_pure_pursuit = True # pure pursuitの開始フラグをセット
        self.goal_handle = goal_handle
        # 経路データの受け取りと格納
        path_msgs = goal_handle.request.path.path
        # 特定の点のインデックスの受け取りと格納
        self.indices = np.array(goal_handle.request.feedback_indices)
        self.get_logger().info("path data has been received")
        # 経路データをnumpy配列に変換
        self.path_data = np.array([[msg.x, msg.y, msg.theta] for msg in path_msgs])
        self.tangents = self.compute_tangents (self.path_data) # 接ベクトルの計算と格納
        self.angles, self.max_angle = self.compute_angles (self.tangents) # 角度の計算と格納

        self.get_logger().info("path data has been initialized")
        self.get_logger().debug(f"path_data: {self.path_data}")
        self.get_logger().debug(f"indices: {self.indices}")
        # 新しいスレッドで条件が満たされるのを待つ
        threading.Thread(target=self.wait_for_condition).start()
        self.goal_event.wait()  # 条件が満たされるまで待つ
        goal_handle.succeed()
        return self.result_msg

    def wait_for_condition(self):
        while rclpy.ok() and not self.completed:
            time.sleep(0.1)  # 0.1秒待つ
        # 条件が満たされたらイベントフラグをセット
        self.goal_event.set()

    def pose_callback (self, msg: PoseStamped) -> None:
        self.get_logger().info(f"start_pure_pursuit: {self.start_pure_pursuit}")
        # pure pursuitの開始フラグが立っていない場合は何もしない
        if not self.start_pure_pursuit:
            self.get_logger().warn("pure pursuit has not started yet")
            return
        elif (self.goal_handle is not None) and (not self.goal_handle.is_active):
            self.get_logger().warn("no active goal")
            return
        # 位置の受け取りと格納
        self.robot_pose = self.pose_to_array(msg) # 位置の格納
        self.get_logger().debug(f"x: {self.robot_pose[0]}, y: {self.robot_pose[1]}, theta: {self.robot_pose[2]}")
        # 先行点の計算
        lookahead_point: NDArray[np.float64] = None
        lookahead_point, self.current_path_index = self.find_lookahead_point (self.robot_pose, self.path_data, self.lookahead_distance, self.distance_threshold, self.robot_pose[:2], self.current_path_index)
        self.get_logger().info(f"lookahead_point: {lookahead_point}")
        # 最も近い点の計算
        closest_point: NDArray[np.float64] = None
        closest_point, self.closest_index = self.find_closest_point (self.robot_pose, self.path_data, self.robot_pose, self.closest_index)
        # 速度入力の計算
        vel_msg = self.速度入力の計算 (self.robot_pose, lookahead_point, closest_point, self.lookahead_distance, self.speed)
        self.vel_pub.publish(vel_msg) # 速度入力パブリッシュ
        self.get_logger().debug(f"v_x: {vel_msg.linear.x}, v_y: {vel_msg.linear.y}, omega: {vel_msg.angular.z}")
        self.current_speed, self.current_lookahead_distance = self.change_speed_lookahead_distance (
            self.path_data, self.current_lookahead_distance, self.current_speed, self.angles, self.max_angle, closest_point, self.closest_index
        )

        # 完了処理
        if self.closest_index in self.indices:
            # フィードバックにインデックスを返す
            feedback_msg = PathAndFeedback.Feedback()
            feedback_msg.current_index = self.closest_index
            self.goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f"feedback: {self.closest_index}")
        elif self.distance(self.robot_pose[:2], self.path_data[-1][:2]) < self.distance_threshold:
            # 完了処理を行う
            self.result_msg = PathAndFeedback.Result()
            self.result_msg.final_index = self.closest_index
            self.completed = True

    def 速度入力の計算 (self,
            robot_pose: NDArray[np.float64], 
            lookahead_point: NDArray[np.float64],
            closest_point: NDArray[np.float64],
            lookahead_distance: float,
            speed: float
        ) -> Twist:
        self.pure_pursuit_vel = self.calc_pure_pursuit_vel (robot_pose, lookahead_point, lookahead_distance, speed, self.pure_pursuit_vel)
        p_control_vel: NDArray[np.float64] = self.compute_path_P_input (robot_pose[:2], closest_point, self.closest_index, self.angles, np.pi, self.angle_p_gain)
        omega: float = self.角度PI制御入力の計算 (robot_pose, closest_point)
        vel_msg = Twist()
        vel_msg.linear.x = self.pure_pursuit_vel[0] + p_control_vel[0]
        vel_msg.linear.x = self.pure_pursuit_vel[1] + p_control_vel[1]
        vel_msg.angular.z = omega
        return vel_msg

    def find_lookahead_point (self,
            robot_pose: NDArray[np.float64], 
            path_data: NDArray[np.float64], 
            lookahead_distance: float,
            distance_threshold: float,
            previous_point: NDArray[np.float64],
            previous_index: int
        ) -> tuple[NDArray[np.float64], int]:
        lookahead_point: NDArray[np.float64] = previous_point
        index: int = previous_index

        for i in range(previous_index + 1, len(path_data)):
            point = path_data[i][:2]
            if abs(self.distance(robot_pose[:2], point) - lookahead_distance) < distance_threshold:
                lookahead_point = point
                index = i
                break

        return lookahead_point, index

    def find_closest_point (self,
            robot_pose: NDArray[np.float64], 
            path_data: NDArray[np.float64],
            previous_point: NDArray[np.float64],
            previous_index: int
        ) -> tuple[NDArray[np.float64], int]:
        closest_point: NDArray[np.float64] = previous_point
        closest_index: int = -1
        min_distance: float = float('inf')

        for i in range(previous_index + 1, len(path_data)):
            point = path_data[i][:2]
            d = self.distance(robot_pose[:2], point)
            if d < min_distance:
                min_distance = d
                closest_point = point
                closest_index = i

        return closest_point, closest_index

    def calc_pure_pursuit_vel (self,
            robot_pose: NDArray[np.float64], 
            lookahead_point: NDArray[np.float64], 
            speed: float, 
        ) -> NDArray[np.float64]:
        # 速度入力の計算
        vel: NDArray[np.float64] = np.array([0.0, 0.0])
        direction: NDArray[np.float64] = lookahead_point - robot_pose[:2]
        if np.linalg.norm(direction) > 0.0:
            direction /= np.linalg.norm(direction) # 方向ベクトルの正規化
        vel = direction * speed
        return vel

    def compute_path_P_input (self, 
            robot_position: NDArray[np.float64], 
            closest_point: NDArray[np.float64],
            closest_index: int,
            angles: NDArray[np.float64],
            max_angle: float,
            path_p_gain: float
        ) -> NDArray[np.float64]:
        # p_input_vel = path_p_gain * (closest_point - robot_position)
        p_input_vel = path_p_gain * (1.0 + (1.0 / 10.0 - 1.0) * angles[closest_index] / max_angle) * (closest_point - robot_position)
        return p_input_vel

    def 角度PI制御入力の計算 (self, robot_pose: NDArray[np.float64], closest_point: NDArray[np.float64]) -> float:
        return 0.0

    def compute_tangents (self, path_data: NDArray[np.float64]) -> NDArray[np.float64]:
        # 接ベクトルの計算
        path_x = path_data[:, 0]
        path_y = path_data[:, 1]
        tangents = np.array([np.gradient(path_x), np.gradient(path_y)]).T  # 各点における接ベクトル
        return tangents
    
    def compute_angles (self, tangents: NDArray[np.float64]) -> tuple[NDArray[np.float64], float]:
        angles = []

        # 隣接する接ベクトル間の角度を計算する
        for i in range(len(tangents) - 1):
            dot_product = np.dot(tangents[i], tangents[i+1])
            norms_product = np.linalg.norm(tangents[i]) * np.linalg.norm(tangents[i+1])
            # 内積の値を-1と1の間に制限する
            cos_angle = np.clip(dot_product / norms_product, -1.0, 1.0)
            angle = np.arccos(cos_angle)
            angles.append(angle)
        angles.append(angle) # close

        max_angle = np.max(angles)

        return angles, max_angle
    
    def change_speed_lookahead_distance (self, 
            path_data: NDArray[np.float64],
            lookahead_distance: float,
            current_LA_dist: float,
            speed: float,
            current_speed: float,
            angles: NDArray[np.float64],
            max_angle: float,
            closest_point: NDArray[np.float64],
            closest_index: int,
        ):
        path_x = path_data[:, 0]
        path_y = path_data[:, 1]
        dist = np.sqrt((path_x[-1] - closest_point[0])**2 + (path_y[-1] - closest_point[1])**2)
        # if 最終点と現在の先行点の距離がしきい値以下なら:
        if dist < lookahead_distance * 2.0:
            # change lookahead_distance and speed as propotion to distance between lookahead_point and final path point
            # current_LA_dist = lookahead_distance * (1.0 - np.cos(np.pi * dist / lookahead_distance)) / 2.0
            current_LA_dist = lookahead_distance * (dist / 2.0 / lookahead_distance)
            current_speed = speed * dist / lookahead_distance / 2.0
            # current_speed = speed * (1.0 - np.cos(np.pi * dist / lookahead_distance / 2.0)) / 2.0
        else : # 曲率に応じて変化させる
            # current_LA_dist = lookahead_distance / 2 * (1 - curvatures[closest_index] / max_curvature) + lookahead_distance * curvatures[closest_index] / max_curvature
            target_speed = speed * (1.0 + (1.0 / 3.0 - 1.0) * angles[closest_index] / max_angle)
            current_speed = self.first_order_vel(current_speed, target_speed, 1.0, 0.05, 0.5)
        return current_speed, current_LA_dist

    def pose_to_array (self, msg: PoseStamped) -> NDArray[np.float64]:
        robot_pose: NDArray[np.float64] = np.array([0.0, 0.0, 0.0])
        robot_pose[0] = msg.pose.position.x
        robot_pose[1] = msg.pose.position.y
        q = np.array([
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ])
        robot_pose[2] = self.quaternion_to_yaw(q)
        return robot_pose

    def distance (self, p1: NDArray[np.float64], p2: NDArray[np.float64]) -> float:
        # 2点間の距離を計算
        return np.linalg.norm(p1 - p2)
        
    def quaternion_to_yaw(self, q: NDArray[np.float64]) -> float:
        """
        description: クォータニオンからヨー角(ズ軸回りの回転)を計算する。
        q: クォータニオンを [x, y, z, w] の形で表したnumpy配列
        """
        # クォータニオンの要素を抽出
        x, y, z, w = q
        
        # ヨー (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y**2 + z**2)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return yaw
    
    def first_order_vel(self, previous_vel, input_vel, K, dt, tau):
        new_vel = (tau * previous_vel + K * dt * input_vel) / (tau + dt)
        return new_vel

def main () -> None:
    rclpy.init()
    node = PurePursuitNode()
    executor = MultiThreadedExecutor() # threadingを使うためのexecutor
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()