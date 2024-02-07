#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class CalcVelNode(Node):
    def __init__(self):
        super().__init__('debug_vel')
        # ホイールの角速度をパブリッシュ
        self.publisher_ = self.create_publisher(
            Float64MultiArray, '/input_vel', 10)

    def send_vel_always(self):
        # ホイールの角速度メッセージを作成してパブリッシュ
        vel_msg = Float64MultiArray()
        vel_msg.data = [1.0, 1.0, 1.0]
        self.publisher_.publish(vel_msg)

    def calc_3wheel_vel(self, r, l, v_x, v_y, omega):
        # 3輪のホイール速度を計算
        omega_1 = (-0.5 * v_x + 0.8660254037844386 * v_y + l * omega) / r
        omega_2 = (-0.5 * v_x - 0.8660254037844386 * v_y + l * omega) / r
        omega_3 = (v_x + l * omega) / r
        return omega_1, omega_2, omega_3

def main(args=None):
    rclpy.init(args=args)
    calc_vel_node = CalcVelNode()
    rclpy.spin(calc_vel_node)
    # シャットダウン処理
    calc_vel_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()