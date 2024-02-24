import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Quaternion
from tf_transformations import euler_from_quaternion, quaternion_from_euler

class OdometryCalculator(Node):
    def __init__(self):
        super().__init__('pose_PI_simulator')
        # パラメータの宣言
        self.declare_parameters(
            namespace='',
            parameters=[
                ('radius', 0.0508), # タイヤ半径
                ('length', 0.400), # タイヤの設置半径
                ('output_name', '/odometry_pose'), # オドメトリー情報をパブリッシュするトピック名
                ('frame_id', 'odom'), # オドメトリー情報のフレームID
            ]
        )
        self.radius = self.get_parameter('radius').value
        self.length = self.get_parameter('length').value
        self.output_name = self.get_parameter('output_name').value
        self.frame_id = self.get_parameter('frame_id').value
        # サブスクライバーの設定
        self.subscription = self.create_subscription(Float64MultiArray, '/input_vel', self.velocity_callback, 10)
        self.initial_subscription = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.initial_callback, 10)
        # パブリッシャーの設定
        self.publisher_ = self.create_publisher(PoseStamped, self.output_name, 10)
        # 現在の位置と姿勢の初期化
        self.current_pose = PoseStamped()
        self.current_pose.pose.position.x = 0.0
        self.current_pose.pose.position.y = 0.0
        self.current_pose.pose.position.z = 0.0
        # 四元数の初期化
        q = quaternion_from_euler(0, 0, 0)
        self.current_pose.pose.orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        self.current_pose.header.frame_id = self.frame_id
        # 前回のコールバック時間
        self.last_time = self.get_clock().now()
        # 定期的にオドメトリー情報をpublish
        self.dt = 0.1 # 0.1秒ごとにオドメトリー情報をpublish -> 10Hz
        self.timer = self.create_timer(self.dt, self.timer_callback)
        self.get_logger().info('pose_PI_simulator has been started')
        self.get_logger().info('radius: %f' % self.radius)
        self.get_logger().info('length: %f' % self.length)
        self.get_logger().info('output_name: %s' % self.output_name)
        self.get_logger().info('frame_id: %s' % self.frame_id)

    def timer_callback(self):
        self.publisher_.publish(self.current_pose)

    def velocity_callback(self, msg):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt > 10 * self.dt:
            self.get_logger().warn('dt is too large: %f' % dt)
            dt = self.dt
        self.last_time = current_time

        # 速度指令を受け取る
        omega_1, omega_2, omega_3 = msg.data
        v_x, v_y, omega = self.transform_vel_from_3wheel(self.radius, self.length, omega_1, omega_2, omega_3)

        # 速度指令に基づいて位置を更新
        self.current_pose.pose.position.x += v_x * dt
        self.current_pose.pose.position.y += v_y * dt

        # 回転角度を更新
        _, _, yaw = euler_from_quaternion([
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w
        ])
        yaw += omega * dt
        q = quaternion_from_euler(0, 0, yaw)
        self.current_pose.pose.orientation.x = q[0]
        self.current_pose.pose.orientation.y = q[1]
        self.current_pose.pose.orientation.z = q[2]
        self.current_pose.pose.orientation.w = q[3]

        # オドメトリー情報をpublish
        self.publisher_.publish(self.current_pose)

    def initial_callback(self, msg: PoseWithCovarianceStamped):
        # 初期位置を受け取る
        self.current_pose.pose = msg.pose.pose
        self.current_pose.header = msg.header
        self.get_logger().info('initialpose: %s' % self.current_pose)
        # オドメトリー情報をpublish
        self.publisher_.publish(self.current_pose)
    
    def transform_vel_from_3wheel(self, r, l, omega_1, omega_2, omega_3):
        v_x = r / 3.0 * (-omega_1 - omega_2 + 2 * r)
        v_y = r / 1.7320508075688772 * (omega_1 - omega_2)
        omega = r * (omega_1 + omega_2 + omega_3) / (3.0 * l)
        return v_x, v_y, omega

def main(args=None):
    rclpy.init(args=args)
    odometry_calculator = OdometryCalculator()
    rclpy.spin(odometry_calculator)

    # ノードのクリーンアップ
    odometry_calculator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
