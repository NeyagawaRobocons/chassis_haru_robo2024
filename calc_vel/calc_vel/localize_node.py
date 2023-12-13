# description: 仮の自己位置計算ノード
# input: odometry_poseトピック(geometry_msgs/msg/Pose型)
# output: robot_poseトピック(geometry_msgs/msg/Pose型)

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose

class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('localize_node')
        self.subscription = self.create_subscription(
            Pose,
            'odometry_pose',
            self.listener_callback,
            10)
        self.subscription

        self.publisher_ = self.create_publisher(
            Pose, 
            'robot_pose',
            10)
        self.odom_pose = Pose()
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def listener_callback(self, msg: Pose):
        self.odom_pose = msg

    def timer_callback(self):
        self.publisher_.publish(self.odom_pose)

def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)
    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()