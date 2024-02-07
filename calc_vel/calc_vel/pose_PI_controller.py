# description: 位置制御を行うノード
# input1: 目標位置：/goal_poseトピック(geometry_msgs/msg/PoseStamped型)
# input2: 現在位置：/robot_poseトピック(geometry_msgs/msg/PoseStamped型)
# output: 速度指令：/input_velトピック(std_msgs/msg/Float64MultiArray型)
# 計算: PI_controller_class.pyのPI_controllerクラスを使用
# パラメータ: p_gain, i_gain -> yamlファイル(../yaml/pi_params.yaml)から読み込み
# 起動コマンド: 
# cd ~/ros2_ws/src/chassis_haru_robo2024/calc_vel/calc_vel
# ros2 run calc_vel calc_vel --ros-args --params-file ../yaml/pi_params.yaml

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import math
from std_msgs.msg import Float64MultiArray as Float64
from .PI_controller_class import PIController # .をつけることで同じパッケージ内のファイルを読み込む

PI_controller = [PIController(kp=1, ki=0.10, max_input=10), 
                 PIController(kp=1, ki=0.10, max_input=10), 
                 PIController(kp=1, ki=0.10, max_input=10)]

def get_yaw_from_pose(pose_stamped):
    orientation = pose_stamped.pose.orientation
    # 四元数の値
    x = orientation.x
    y = orientation.y
    z = orientation.z
    w = orientation.w

    # 四元数からヨー角を計算
    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return yaw

class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('calc_vel')
        self.vel_pub = self.create_publisher(Float64, 'input_vel', 10)
        self.goal_sub = self.create_subscription(
            PoseStamped,
            'goal_pose',
            self.goal_callback,
            10)
        self.goal_sub  # 警告を回避するために設置されているだけです。削除しても挙動はかわりません。
        self.robot_sub = self.create_subscription(
            PoseStamped,
            'robot_pose',
            self.robot_callback,
            10)
        self.robot_sub

        # パラメータを宣言
        self.declare_parameter('p_gain', 0.0)  # 0.0はデフォルト値
        self.declare_parameter('i_gain', 0.0)  # 同様にi_gainも宣言
        self.declare_parameter('max_vel', 0.0)
        # パラメータの現在値を取得(初期化)
        self.p_gain = self.get_parameter('p_gain').get_parameter_value().double_value
        self.i_gain = self.get_parameter('i_gain').get_parameter_value().double_value
        self.goal = PoseStamped() # 目標姿勢の格納
        self.pre_pose = PoseStamped() # 姿勢の格納
        self.dt = 0.0 # 制御周期
        self.max_vel = self.get_parameter('max_vel').get_parameter_value().double_value # 最大速度 [rad/s]

        self.previous_time = self.get_clock().now().nanoseconds # 前回の時間

        self.get_logger().info('complete initialize')

    def renewal_gain(self, dt):
        # パラメータの現在値を取得
        self.p_gain = self.get_parameter('p_gain').get_parameter_value().double_value
        self.i_gain = self.get_parameter('i_gain').get_parameter_value().double_value
        # self.get_logger().info('p_gain: %s' % self.p_gain)
        # self.get_logger().info('i_gain: %s' % self.i_gain)
        # ゲインを更新(Pythonでforを使いたくなかったのでこうなっています)
        PI_controller[0].set_params(self.p_gain, self.i_gain, dt)
        PI_controller[1].set_params(self.p_gain, self.i_gain, dt)
        PI_controller[2].set_params(self.p_gain, self.i_gain, dt)

    def goal_callback(self, msg: PoseStamped):
        # 目標位置を取得，格納
        self.goal = msg
        self.get_logger().info('goal_pose: %s' % self.goal.pose.position + 'goal_yaw: %s' % get_yaw_from_pose(self.goal))

    def robot_callback(self, msg: PoseStamped):
        current_time = self.get_clock().now().nanoseconds # 現在時刻を取得
        dt = (current_time - self.previous_time) * 1e-9 # 制御周期を計算
        self.renewal_gain(dt) # ゲインを更新
        vels = Float64() # 速度指令の格納
        vels.data = [0.0, 0.0, 0.0]
        if bool(msg.pose.position.x - self.pre_pose.pose.position.x) or bool(msg.pose.position.y - self.pre_pose.pose.position.y) or bool(get_yaw_from_pose(msg) - get_yaw_from_pose(self.pre_pose)):
            self.get_logger().info("目標値への移動を開始します")
            self.get_logger().info('robot_pose: %s' % msg.pose.position + 'robot_yaw: %s' % get_yaw_from_pose(msg))
            self.get_logger().info('goal_pose: %s' % self.goal.pose.position + 'goal_yaw: %s' % get_yaw_from_pose(self.goal))
            # ここにPI制御入力計算を記述
            vels.data[0] = PI_controller[0].calc_output(self.goal.pose.orientation.x - msg.pose.orientation.x)
            vels.data[1] = PI_controller[1].calc_output(self.goal.pose.orientation.y - msg.pose.orientation.y)
            vels.data[2] = PI_controller[2].calc_output(get_yaw_from_pose(self.goal) - get_yaw_from_pose(msg))
            self.vel_pub.publish(vels)
            self.pre_pose = msg # 姿勢の更新
            self.get_logger().info('publish vels: %s' % vels.data)
        self.previous_time = current_time # 前回の時間を更新

def main(args=None):
    rclpy.init(args=args)
    minimal_subscriber = MinimalSubscriber()
    rclpy.spin(minimal_subscriber)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()