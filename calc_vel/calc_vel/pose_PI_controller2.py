"""
- description: 位置制御を行うノード
- input1: 目標位置：/goal_poseトピック(geometry_msgs/msg/PoseStamped型)
- input2: 現在位置：/robot_poseトピック(geometry_msgs/msg/PoseStamped型)
- output: 速度指令：/input_velトピック(std_msgs/msg/Float64MultiArray型)
- 計算: -> PI_controller_class.pyのPI_controllerクラスを使用
- パラメータ: -> yamlファイル(../yaml/pi_params.yaml)から読み込み
  - p_gain_x: xのPゲイン
  - i_gain_x: xのIゲイン
  - p_gain_y: yのPゲイン
  - i_gain_y: yのIゲイン
  - p_gain_theta: thetaのPゲイン
  - i_gain_theta: thetaのIゲイン
  - max_input: 入力の最大値
  - radius: タイヤ半径
  - length: タイヤの設置半径
- 起動コマンド: 
  - cd ~/ros2_ws/src/chassis_haru_robo2024/calc_vel/calc_vel
  - ros2 run calc_vel calc_vel --ros-args --params-file ../yaml/pi_params.yaml
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from .PI_controller_class import PIController  # PI制御クラスのインポート
import numpy as np

class VelocityController(Node):
    def __init__(self):
        super().__init__('calc_vel')

        # パラメータの宣言
        self.declare_parameters(
            namespace='',
            parameters=[
                ('p_gain_x', 1.0),
                ('i_gain_x', 0.1),
                ('p_gain_y', 1.0),
                ('i_gain_y', 0.1),
                ('p_gain_theta', 0.1),
                ('i_gain_theta', 0.01),
                ('max_input', 30.0),
                ('radius', 0.050), # タイヤ半径
                ('length', 0.400), # タイヤの設置半径
            ]
        )
        # パラメータの取得
        self.p_gain_x = self.get_parameter('p_gain_x').value
        self.i_gain_x = self.get_parameter('i_gain_x').value
        self.p_gain_y = self.get_parameter('p_gain_y').value
        self.i_gain_y = self.get_parameter('i_gain_y').value
        self.p_gain_theta = self.get_parameter('p_gain_theta').value
        self.i_gain_theta = self.get_parameter('i_gain_theta').value
        max_input = self.get_parameter('max_input').value
        self.radius = self.get_parameter('radius').value
        self.length = self.get_parameter('length').value
        # メッセージの初期化
        self.goal_pose = None
        self.current_pose = None
        # 前回のコールバック時間
        self.last_time = self.get_clock().now()

        # 入力の最大値から各軸の最大速度を計算
        # self.v_x_max = self.radius / 3.0 * (-max_input - max_input + 2 * self.radius)
        # self.v_y_max = self.radius / 1.7320508075688772 * (max_input - max_input)
        self.v_x_max = max_input * self.radius
        self.v_y_max = max_input * self.radius
        self.omega_max = self.radius * (max_input) / self.length
        self.get_logger().info('v_x_max: %s' % self.v_x_max)
        self.get_logger().info('v_y_max: %s' % self.v_y_max)
        self.get_logger().info('omega_max: %s' % self.omega_max)

        # PIコントローラのインスタンス化
        self.controller_x = PIController(self.p_gain_x, self.i_gain_x, self.v_x_max)
        self.controller_y = PIController(self.p_gain_y, self.i_gain_y, self.v_y_max)
        self.controller_theta = PIController(self.p_gain_theta, self.i_gain_theta, self.omega_max)

        # サブスクライバーの設定
        self.subscription_goal = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.subscription_current = self.create_subscription(PoseStamped, '/robot_pose', self.current_pose_callback, 10)
        # パブリッシャーの設定
        self.publisher_ = self.create_publisher(Float64MultiArray, '/input_vel', 10)

    def goal_pose_callback(self, msg):
        self.goal_pose = msg
        # 目標位置が更新されたら積分項をリセット
        self.controller_x.reset_integral()
        self.controller_y.reset_integral()
        self.controller_theta.reset_integral()

    def current_pose_callback(self, msg):
        self.current_pose = msg
        self.p_gain_x = self.get_parameter('p_gain_x').value
        self.i_gain_x = self.get_parameter('i_gain_x').value
        self.p_gain_y = self.get_parameter('p_gain_y').value
        self.i_gain_y = self.get_parameter('i_gain_y').value
        self.p_gain_theta = self.get_parameter('p_gain_theta').value
        self.i_gain_theta = self.get_parameter('i_gain_theta').value
        self.controller_x.set_gains(self.p_gain_x, self.i_gain_x)
        self.controller_y.set_gains(self.p_gain_y, self.i_gain_y)
        self.controller_theta.set_gains(self.p_gain_theta, self.i_gain_theta)
        # self.get_logger().info('current_pose: %s' % self.current_pose)
        self.calculate_and_publish_velocity()
        self.get_logger().info('kp_x: %s, ki_x: %s, kp_y: %s, ki_y: %s, kp_theta: %s, ki_theta: %s' % (self.p_gain_x, self.i_gain_x, self.p_gain_y, self.i_gain_y, self.p_gain_theta, self.i_gain_theta))

    def calculate_and_publish_velocity(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time
        if self.goal_pose is None or self.current_pose is None:
            return

        # 目標位置と現在位置から制御入力を計算
        v_x = self.controller_x.update(self.goal_pose.pose.position.x, self.current_pose.pose.position.x, dt)
        v_y = self.controller_y.update(self.goal_pose.pose.position.y, self.current_pose.pose.position.y, dt)
        omega = self.controller_theta.update(self.get_yaw(self.goal_pose), self.get_yaw(self.current_pose), dt)

        # x, y方向の速度はロボットの向きに合わせて変換(値を角-thetaだけ回転させる)
        _theta = self.get_yaw(self.current_pose)
        rot_v_x = v_x * np.cos(_theta) + v_y * np.sin(_theta)
        rot_v_y = -v_x * np.sin(_theta) + v_y * np.cos(_theta)
        # 3輪オムニホイールの速度に変換
        omega_1, omega_2, omega_3 = self.calc_3wheel_vel(self.radius, self.length, rot_v_x, rot_v_y, omega)

        # 速度指令のパブリッシュ
        vel_msg = Float64MultiArray()
        vel_msg.data = [0.0, 0.0, 0.0] # data変数の初期化
        vel_msg.data = [omega_1, omega_2, omega_3]
        self.publisher_.publish(vel_msg)

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
    
    def calc_3wheel_vel(self, r, l, v_x, v_y, omega):
        # ロボットの速度，角速度から3輪オムニホイールの速度を計算する．
        omega_1 = (-0.5 * v_x + 0.8660254037844386 * v_y + l * omega) / r
        omega_2 = (-0.5 * v_x - 0.8660254037844386 * v_y + l * omega) / r
        omega_3 = (v_x + l * omega) / r
        return omega_1, omega_2, omega_3

def main(args=None):
    rclpy.init(args=args)
    velocity_controller = VelocityController()
    rclpy.spin(velocity_controller)

    # ノードのクリーンアップ
    velocity_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()