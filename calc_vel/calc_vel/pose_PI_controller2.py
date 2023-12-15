import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
from .PI_controller_class import PIController  # PI制御クラスのインポート

class VelocityController(Node):
    def __init__(self):
        super().__init__('calc_vel')

        # パラメータの宣言
        self.declare_parameters(
            namespace='',
            parameters=[
                ('p_gain_x', 1.0),
                ('i_gain_x', 1.0),
                ('p_gain_y', 1.0),
                ('i_gain_y', 1.0),
                ('p_gain_theta', 1.0),
                ('i_gain_theta', 1.0),
                ('max_input', 120.0)
            ]
        )

        # PIコントローラのインスタンス化
        self.controller_x = PIController(self.get_parameter('p_gain_x').value, self.get_parameter('i_gain_x').value, self.get_parameter('max_input').value)
        self.controller_y = PIController(self.get_parameter('p_gain_y').value, self.get_parameter('i_gain_y').value, self.get_parameter('max_input').value)
        self.controller_theta = PIController(self.get_parameter('p_gain_theta').value, self.get_parameter('i_gain_theta').value, self.get_parameter('max_input').value)

        # サブスクライバーの設定
        self.subscription_goal = self.create_subscription(PoseStamped, '/goal_pose', self.goal_pose_callback, 10)
        self.subscription_current = self.create_subscription(PoseStamped, '/robot_pose', self.current_pose_callback, 10)

        # パブリッシャーの設定
        self.publisher_ = self.create_publisher(Float64MultiArray, '/input_vel', 10)

        # メッセージの初期化
        self.goal_pose = None
        self.current_pose = None

    def goal_pose_callback(self, msg):
        self.goal_pose = msg
        # 目標位置が更新されたら積分項をリセット
        self.controller_x.reset_integral()
        self.controller_y.reset_integral()
        self.controller_theta.reset_integral()

    def current_pose_callback(self, msg):
        self.current_pose = msg
        self.calculate_and_publish_velocity()

    def calculate_and_publish_velocity(self):
        if self.goal_pose is None or self.current_pose is None:
            return

        # 目標位置と現在位置から制御入力を計算
        v_x = self.controller_x.update(self.goal_pose.pose.position.x, self.current_pose.pose.position.x, dt)
        v_y = self.controller_y.update(self.goal_pose.pose.position.y, self.current_pose.pose.position.y, dt)
        omega = self.controller_theta.update(self.get_yaw(self.goal_pose.pose), self.get_yaw(self.current_pose.pose), dt)

        # ここでオムニホイールの速度に変換（ユーザーが実装）

        # 速度指令のパブリッシュ
        vel_msg = Float64MultiArray()
        vel_msg.data = [v_x, v_y, omega]
        self.publisher_.publish(vel_msg)

    def get_yaw(self, pose):
        # 回転（クォータニオン）からYaw角を取得する関数
        # 実装が必要

def main(args=None):
    rclpy.init(args=args)
    velocity_controller = VelocityController()
    rclpy.spin(velocity_controller)

    # ノードのクリーンアップ
    velocity_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
