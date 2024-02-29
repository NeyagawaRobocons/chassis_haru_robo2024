#!/usr/bin/env python3
import numpy as np
from numpy.typing import NDArray
from scipy.ndimage import gaussian_filter1d
import threading
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Quaternion, PoseStamped, Twist, TwistStamped, PoseWithCovarianceStamped
from visualization_msgs.msg import Marker
from pure_pursuit.action import PathAndFeedback
from PI_controller_class import PIController

class PurePursuitNode(Node):
    def __init__(self) -> None:
        super().__init__('pure_pursuit_node')
        # トピック, アクションの初期化
        self.pose_sub = self.create_subscription(PoseStamped, '/robot_pose', self.pose_callback, 10)
        self.action_server = ActionServer(self, PathAndFeedback, 'path_and_feedback', self.execute_callback)
        self.vel_pub = self.create_publisher(Twist, '/robot_vel', 10)
        # 可視化用トピックの初期化
        self.pure_pursuit_vel_pub = self.create_publisher(TwistStamped, '/pure_pursuit_vel', 10)
        self.p_control_vel_pub = self.create_publisher(TwistStamped, '/p_control_vel', 10)
        self.lookahead_pub = self.create_publisher(PoseStamped, 'lookahead_pose', 10)
        self.closest_pub = self.create_publisher(PoseStamped, 'closest_pose', 10)
        self.circle_pub = self.create_publisher(Marker, 'circle_marker', 10)
        # パラメータの宣言
        self.declare_parameters(
            namespace='',
            parameters=[
                ('lookahead_distance', 0.5),
                ('path_p_gain', 0.05),
                ('angle_p_gain', 0.5),
                ('angle_i_gain', 0.0),
                ('distance_threshold', 0.03), # [m]
                ('angle_threshold', 0.1), # [rad]
                ('initial_pose', [0.0, 0.0, 0.0]),
                ('path_p_magnification', 10.0),
                ('dt', 0.03), # [s]
                ('publish_rate', 10.0), # [Hz]
            ])
        # パラメータの取得
        self.lookahead_distance: float = self.get_parameter('lookahead_distance').value
        self.path_p_gain: float = self.get_parameter('path_p_gain').value
        self.angle_p_gain: float = self.get_parameter('angle_p_gain').value
        self.angle_i_gain: float = self.get_parameter('angle_i_gain').value
        self.distance_threshold: float = self.get_parameter('distance_threshold').value
        self.angle_threshold: float = self.get_parameter('angle_threshold').value
        self.initial_pose: NDArray[np.float64] = self.get_parameter('initial_pose').value
        self.path_p_magnification: float = self.get_parameter('path_p_magnification').value
        self.dt: float = self.get_parameter('dt').value
        self.publish_rate: float = self.get_parameter('publish_rate').value
        self.get_logger().info("parameters have been initialized")
        self.get_logger().info(f"lookahead_distance: {self.lookahead_distance}")
        self.get_logger().info(f"path_p_gain: {self.path_p_gain}")
        self.get_logger().info(f"angle_p_gain: {self.angle_p_gain}")
        self.get_logger().info(f"angle_i_gain: {self.angle_i_gain}")
        self.get_logger().info(f"distance_threshold: {self.distance_threshold}")
        self.get_logger().info(f"angle_threshold: {self.angle_threshold}")
        self.get_logger().info(f"initial_pose: {self.initial_pose}")
        self.get_logger().info(f"path_p_magnification: {self.path_p_magnification}")
        self.get_logger().info(f"dt: {self.dt}")
        self.get_logger().info(f"publish_rate: {self.publish_rate}")
        # PI制御器の初期化
        self.angle_controller = PIController(self.angle_p_gain, self.angle_i_gain, max_input=1.0)
        # 準静的な変数の初期化
        self.path_data: NDArray[np.float64] = None # np.array([[x1, y1, theta1, speed1], [x2, y2, theta2, speed2], ...])の形
        self.indices: NDArray[np.int8] = np.array([]) # np.array([1, 2, 3])の形
        self.tangents: NDArray[np.float64] = None # np.array([[0.0, 0.0], [0.0, 0.0]])の形
        self.angles: NDArray[np.float64] = None # np.array([0.0, 0.0, 0.0, ...])の形
        self.max_angle: float = 0.0
        self.max_speed: float = 0.0
        self.max_lookahead_distance: float = 0.0
        self.set_pose_index: int = 0
        self.pure_pursuit_timer = self.create_timer(1.0 / self.publish_rate, self.pure_pursuit_timer_callback) # タイマーの初期化
        # 動的な変数の初期化
        self.robot_pose: NDArray[np.float64] = np.array(self.initial_pose) # np.array([x, y, theta])の形
        self.previous_pose: NDArray[np.float64] = self.robot_pose # np.array([x, y, theta])の形
        self.pure_pursuit_vel: NDArray[np.float64] = np.array([0.0, 0.0]) # np.array([v_x, v_y])の形
        self.p_control_vel: NDArray[np.float64] = np.array([0.0, 0.0]) # np.array([v_x, v_y])の形
        self.lookahead_point: NDArray[np.float64] = np.array([0.0, 0.0, 0.0, 0.0]) # np.array([x, y, theta, speed])の形
        self.closest_point: NDArray[np.float64] = np.array([0.0, 0.0, 0.0, 0.0]) # np.array([x, y, theta, speed])の形
        self.closest_index: int = 0 # -1にすると最後の点が選ばれてしまうエラーがある
        self.current_speed: float = 0.0
        self.current_lookahead_distance = self.lookahead_distance
        self.pre_passed_index: int = 0
        # action serverのための変数の初期化
        self.goal_handle = None # GoalHandleの初期化
        self.result_msg = None # Resultの初期化
        self.start_pure_pursuit = False # pure pursuitの開始フラグの初期化
        self.completed = False # 完了フラグの初期化
        self.goal_event = threading.Event()

        self.get_logger().info("pure_pursuit_node has been initialized")

    async def execute_callback (self, goal_handle) -> None:
        self.goal_event.clear() # イベントフラグをリセット
        path_msgs = goal_handle.request.path.path # 経路データの受け取りと格納
        self.indices = np.array(goal_handle.request.feedback_indices) # 特定の点のインデックスの受け取りと格納
        self.set_pose_index = goal_handle.request.set_pose_index # 特定の点のインデックスの受け取りと格納
        self.get_logger().info("path data has been received")
        self.path_data = np.array([[msg.x, msg.y, msg.theta, msg.speed, msg.lookahead_distance] for msg in path_msgs]) # 経路データをnumpy配列に変換
        self.previous_pose = self.path_data[0, :3]
        self.tangents = self.compute_tangents (self.path_data) # 接ベクトルの計算と格納
        self.angles, self.max_angle = self.compute_angles (self.tangents) # 角度の計算と格納
        self.angles = gaussian_filter1d(self.angles, sigma=3) # 角度の平滑化
        # self.max_speed = np.max(self.path_data[:, 3]) # 最大速度の計算
        self.lookahead_distances = self.path_data[:, 4] # 先行距離の計算
        # self.max_lookahead_distance = np.max(self.lookahead_distances) # 最大先行距離の計算
        self.get_logger().info("path data has been initialized")
        self.get_logger().debug(f"path_data: {self.path_data}")
        self.get_logger().debug(f"indices: {self.indices}")
        # action serverのための変数の初期化
        self.completed = False # 完了フラグをリセット
        self.result_msg = None # Resultをリセット
        self.start_pure_pursuit = True # pure pursuitの開始フラグをセット
        self.goal_handle = goal_handle
        # 動的な変数の初期化
        self.pure_pursuit_vel = np.array([0.0, 0.0]) # np.array([v_x, v_y])の形
        self.lookahead_point = np.array([0.0, 0.0, 0.0, 0.0]) # np.array([x, y, theta, speed])の形
        self.closest_point = np.array([0.0, 0.0, 0.0, 0.0]) # np.array([x, y, theta, speed])の形
        self.closest_index = 0 # -1にすると最後の点が選ばれてしまうエラーがある
        self.current_speed = 0.0
        self.current_lookahead_distance = self.lookahead_distance
        self.vel: NDArray[np.float64] = np.array([0.0, 0.0, 0.0]) # np.array([v_x, v_y, omega])の形
        # 新しいスレッドで条件が満たされるのを待つ
        threading.Thread(target=self.wait_for_condition).start()
        self.goal_event.wait()  # 条件が満たされるまで待つ
        self.goal_handle.succeed()
        self.get_logger().info("Goal has been reached! result: {0}".format(self.result_msg.final_index))
        time.sleep(0.2)  # 0.1秒待つ
        return self.result_msg

    def wait_for_condition(self):
        while rclpy.ok() and not self.completed:
            time.sleep(0.05)  # 0.1秒待つ
        # 条件が満たされたらイベントフラグをセット
        self.goal_event.set()

    def pose_callback (self, msg: PoseStamped) -> None:
        self.robot_pose = self.pose_to_array(msg) # 位置の格納

    def pure_pursuit_timer_callback (self) -> None:
        self.get_logger().debug(f"start_pure_pursuit: {self.start_pure_pursuit}")
        # pure pursuitの開始フラグが立っていない場合は何もしない
        if not self.start_pure_pursuit:
            self.get_logger().warn("pure pursuit has not started yet")
            return
        elif (self.goal_handle is not None) and (not self.goal_handle.is_active):
            self.get_logger().warn("no active goal")
            return
        robot_pose = self.robot_pose # 位置の取得
        self.get_logger().debug(f"x: {robot_pose[0]}, y: {robot_pose[1]}, theta: {robot_pose[2]}")
        # 先行点の計算
        self.lookahead_point, self.closest_point, self.closest_index = self.find_lookahead_point (
            robot_pose, self.path_data, self.tangents, self.closest_index, self.lookahead_point, self.current_lookahead_distance)
        self.get_logger().debug(f"closest_point: {self.closest_point}")
        self.get_logger().debug(f"lookahead_point: {self.lookahead_point}")
        # 先行点, 最も近い点の可視化
        lookahead_pose = self.ndarray_to_PoseStamped(self.lookahead_point)
        closest_pose = self.ndarray_to_PoseStamped(self.closest_point)
        self.show_circle(self.current_lookahead_distance)
        self.lookahead_pub.publish(lookahead_pose) # 先行点パブリッシュ
        self.closest_pub.publish(closest_pose) # 最も近い点パブリッシュ
        if lookahead_pose is None or closest_pose is None:
            self.get_logger().warn("lookahead_pose or closest_pose is None")
        # 速度入力の計算
        self.vel = self.compute_velocity (
            robot_pose, self.lookahead_point, self.closest_point, self.current_speed, self.vel)
        # 完了処理
        for index in self.indices:
            index = int(index) # np.int32をintに変換
            if self.closest_index >= index and self.closest_index < index + 1:
                # フィードバックにインデックスを返す
                feedback_msg = PathAndFeedback.Feedback()
                feedback_msg.current_index = index
                self.goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(f"feedback: {index}, type: {type(index)}")
        if self.distance(robot_pose[:2], self.path_data[-1][:2]) < self.distance_threshold:
            self.get_logger().info(f"x, y distance is less than threshold, robot angle: {robot_pose[2]}, goal angle: {self.path_data[-1][2]}")
            # self.vel[:2] = self.path_p_gain * (self.path_data[-1][:2] - robot_pose[:2]) @ self.rotation_matrix(-robot_pose[2]).T
            self.vel[:2] = np.array([0.0, 0.0])
            if abs(robot_pose[2] - self.path_data[-1][2]) < self.angle_threshold:
                self.vel[2] = 0.0
                # 完了処理を行う
                self.result_msg = PathAndFeedback.Result()
                self.result_msg.final_index = self.closest_index
                # self.vel_pub.publish(Twist()) # 速度のパブリッシュ(停止)
                self.completed = True
                self.start_pure_pursuit = False
                self.get_logger().info("Goal has been completed")
                time.sleep(0.2)  # 0.1秒待つ
        self.publish_vels(self.vel, self.pure_pursuit_vel, self.p_control_vel) # 速度のパブリッシュ
        self.current_speed, self.current_lookahead_distance = self.change_speed_lookahead_distance (self.path_data, self.closest_index)
        self.previous_pose = robot_pose # 位置の更新

    def find_lookahead_point (self,
            robot_pose: NDArray[np.float64], 
            path_data: NDArray[np.float64], 
            tangents: NDArray[np.float64],
            pre_closest_index: int,
            pre_lookahead_point: NDArray[np.float64],
            lookahead_distance: float,
        ) -> tuple[NDArray[np.float64], NDArray[np.float64], int]:
        closest_index: int = 0
        serch_range = 100
        start_index = max(0, pre_closest_index - serch_range)
        end_index = min(len(path_data), pre_closest_index + serch_range)
        # 最も近い点の検索
        distances: NDArray[np.float64] = np.linalg.norm(path_data[:, :2] - robot_pose[:2], axis=1)
        diff_point = path_data[start_index:end_index, :2] - path_data[pre_closest_index, :2]
        euclidean_dist = np.linalg.norm(diff_point, axis=1)
        differences = distances[start_index:end_index]**2
        # + 0.1 * euclidean_dist**2 # LQRっぽい感じ
        
        closest_index = np.argmin(differences) + start_index
        closest_index = int(closest_index)
        closest_point: NDArray[np.float64] = path_data[closest_index]

        self.get_logger().info(f"closest_index: {closest_index}, closest_point: {closest_point}")

        # diff_LA_dist = distances[start_index:end_index] - lookahead_distance
        # diff_LA_point = path_data[start_index:end_index, :2] - pre_lookahead_point[:2]
        # euclidean_LA_dist = np.linalg.norm(diff_LA_point, axis=1)
        # differences_LA = euclidean_LA_dist**2 + diff_LA_dist**2 # LQRっぽい感じ
        lookahead_index: int = closest_index + np.argmin(np.abs(distances[closest_index:] - lookahead_distance))
        # lookahead_index: int = start_index + np.argmin(differences_LA)
        lookahead_point: NDArray[np.float64] = path_data[lookahead_index]

        self.get_logger().info(f"lookahead_index: {lookahead_index}, lookahead_point: {lookahead_point}")

        return lookahead_point, closest_point, closest_index

    def compute_velocity (self,
            robot_pose: NDArray[np.float64], 
            lookahead_point: NDArray[np.float64],
            closest_point: NDArray[np.float64],
            speed: float, 
            previous_vel: NDArray[np.float64] = np.array([0.0, 0.0, 0.0]),
        ) -> NDArray[np.float64]:
        self.pure_pursuit_vel = self.rotate_vel(
            self.calc_pure_pursuit_vel(robot_pose, lookahead_point, speed), 
            -robot_pose[2])
        self.p_control_vel = self.rotate_vel(
            self.compute_path_P_input(robot_pose, closest_point, self.closest_index, self.angles, self.max_angle, self.path_p_gain),
            -robot_pose[2])
        omega: float = self.compute_angle_PI (closest_point, robot_pose, self.dt)
        vel: NDArray[np.float64] = np.array([0.0, 0.0, 0.0])
        vel[:2] = self.pure_pursuit_vel + self.p_control_vel
        vel[0] = self.first_order_vel(previous_vel[0], vel[0], 1.0, self.dt, 0.1)
        vel[1] = self.first_order_vel(previous_vel[1], vel[1], 1.0, self.dt, 0.1)
        vel[2] = omega

        return vel

    def vel_to_Twist (self, vel: NDArray[np.float64]) -> Twist:
        vel_msg = Twist()
        vel_msg.linear.x = vel[0]
        vel_msg.linear.y = vel[1]
        vel_msg.angular.z = vel[2] # omega

        return vel_msg

    def vel_to_TwistStamped (self, vel: NDArray[np.float64]) -> TwistStamped:
        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.header.frame_id = "base_link"
        vel_msg.twist.linear.x = vel[0]
        vel_msg.twist.linear.y = vel[1]
        # vel_msg.twist.angular.z = vel[2] # omega (not used)

        return vel_msg

    def rotate_vel (self, vel: NDArray[np.float64], angle: float) -> NDArray[np.float64]:
        vel[:2] = vel[:2] @ self.rotation_matrix(angle).T # ロボット座標系に変換
        return vel

    def publish_vels (self, 
            vel: NDArray[np.float64],
            pure_pursuit_vel: NDArray[np.float64],
            p_control_vel: NDArray[np.float64],
        ) -> None:
        vel_msg = self.vel_to_Twist(vel)
        pure_pursuit_vel_msg = self.vel_to_TwistStamped(pure_pursuit_vel)
        p_control_vel_msg = self.vel_to_TwistStamped(p_control_vel)
        self.vel_pub.publish(vel_msg)
        self.pure_pursuit_vel_pub.publish(pure_pursuit_vel_msg)
        self.p_control_vel_pub.publish(p_control_vel_msg)
        self.get_logger().debug(f"v_x: {vel_msg.linear.x}, v_y: {vel_msg.linear.y}, omega: {vel_msg.angular.z}")

    def ndarray_to_PoseStamped (self, array: NDArray[np.float64]) -> PoseStamped:
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = array[0]
        pose_msg.pose.position.y = array[1]
        pose_msg.pose.orientation = self.yaw_to_quaternion(array[2] + np.pi / 2.0) # ロボット前方向に向ける

        return pose_msg

    def calc_pure_pursuit_vel (self,
            robot_pose: NDArray[np.float64], 
            lookahead_point: NDArray[np.float64], 
            speed: float, 
        ) -> NDArray[np.float64]:
        # 速度入力の計算
        vel: NDArray[np.float64] = np.array([0.0, 0.0])
        direction: NDArray[np.float64] = lookahead_point[:2] - robot_pose[:2]
        if np.linalg.norm(direction) > 0.0:
            direction /= np.linalg.norm(direction) # 方向ベクトルの正規化
        vel = direction * speed

        return vel

    def compute_path_P_input (self, 
            robot_pose: NDArray[np.float64], 
            closest_point: NDArray[np.float64],
            closest_index: int,
            angles: NDArray[np.float64],
            max_angle: float,
            path_p_gain: float
        ) -> NDArray[np.float64]:
        self.get_logger().debug(f"path_p_gain: {path_p_gain}")
        p_input_vel = path_p_gain * (1.0 + (1.0 / self.path_p_magnification - 1.0) * angles[closest_index] / max_angle) * (closest_point[:2] - robot_pose[:2])

        return p_input_vel

    def compute_angle_PI (self, robot_pose: NDArray[np.float64], closest_point: NDArray[np.float64], dt: float = 0.1) -> float:
        return -self.angle_controller.update(closest_point[2], robot_pose[2], dt) # 逆なのでマイナス

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
            closest_index: int,
        ):
        current_speed = path_data[closest_index][3]
        # current_LA_dist = lookahead_distance * (current_speed / max_speed)
        # current_LA_dist = lookahead_distance * np.sqrt(current_speed / max_speed)
        current_LA_dist = path_data[closest_index][4]

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

    def yaw_to_quaternion(self, yaw: float) -> Quaternion:        
        # 四元数を計算
        qx = 0.0
        qy = 0.0
        qz = np.sin(yaw / 2.0)
        qw = np.cos(yaw / 2.0)
        
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

    def rotation_matrix(self, angle: float) -> NDArray[np.float64]:
        """
        description: 2次元の回転行列を計算する。
        angle: 回転角 (ラジアン)
        """
        return np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle), np.cos(angle)]
        ])
    
    def first_order_vel(self, previous_vel, input_vel, K, dt, tau):
        if tau == 0.0:
            return input_vel
        new_vel = (tau * previous_vel + K * dt * input_vel) / (tau + dt)
        return new_vel

    def show_circle (self, radius: float) -> None:
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.type = marker.CYLINDER
        marker.action = marker.ADD
        marker.scale.x = 2.0 * radius  # 直径を半径の2倍に設定
        marker.scale.y = 2.0 * radius
        marker.scale.z = 0.01  # 高さを非常に小さく設定して円に見せる
        marker.color.a = 1.0  # 不透明度
        marker.color.r = 1.0  # 赤色
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.circle_pub.publish(marker)

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