#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistToTwistStamped(Node):
    def __init__(self):
        super().__init__('twist_to_twist_stamped')
        self.subscription = self.create_subscription(
            Twist,
            '/robot_vel',
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(
            TwistStamped,
            '/robot_vel_stamped',
            10)
        
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        twist_stamped_msg = TwistStamped()
        twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
        twist_stamped_msg.header.frame_id = "base_link"  # 適切なフレームIDに変更してください
        twist_stamped_msg.twist = msg

        self.publisher.publish(twist_stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToTwistStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
