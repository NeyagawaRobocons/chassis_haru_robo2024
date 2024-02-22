#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA
from geometry_msgs.msg import Vector3, Point
import math

class TwistVisualizer(Node):
    def __init__(self):
        super().__init__('twist_visualizer')
        self.subscription = self.create_subscription(
            Twist,
            '/robot_vel',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(
            Marker,
            '/visualization_marker',
            10)
        
        self.subscription  # prevent unused variable warning
        self.get_logger().info('twist_visualizer has started')

    def listener_callback(self, msg):
        marker = Marker()
        marker.header = Header(frame_id="base_link", stamp=self.get_clock().now().to_msg())
        marker.ns = "twist"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        marker.scale = Vector3(x=0.1, y=0.2, z=0.0)  # 矢印の大きさ
        marker.color = ColorRGBA(a=1.0, r=0.0, g=1.0, b=0.0)  # 矢印の色
        
        # Twistから速度ベクトルを取得し、矢印の向きと長さを設定
        magnitude = math.sqrt(msg.linear.x ** 2 + msg.linear.y ** 2)
        angle = math.atan2(msg.linear.y, msg.linear.x)
        
        # 矢印の位置と向き
        marker.pose.position = Point(x=0.0, y=0.0, z=0.0)
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = math.sin(angle / 2.0)
        marker.pose.orientation.w = math.cos(angle / 2.0)
        
        # 矢印の長さを速度の大きさに応じて設定
        # marker.scale.x = magnitude
        marker.scale.x = 1.0
        marker.scale.y = 0.2
        marker.scale.z = 0.2

        self.publisher.publish(marker)
        self.get_logger().info(f"{magnitude}, {angle}")

def main(args=None):
    rclpy.init(args=args)
    twist_visualizer = TwistVisualizer()
    rclpy.spin(twist_visualizer)
    twist_visualizer.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
