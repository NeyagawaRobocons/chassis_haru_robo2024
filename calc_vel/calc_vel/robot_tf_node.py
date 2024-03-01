import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
from geometry_msgs.msg import TransformStamped

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('robot_tf_node')
        self.declare_parameter('header_frame_id', 'map')
        self.declare_parameter('child_frame_id', 'base_link')
        self.declare_parameter('topic_name', '/robot_pose')
        self.subscription = self.create_subscription(
            PoseStamped,
            self.get_parameter('topic_name').get_parameter_value().string_value,
            self.handle_robot_pose,
            10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info('complete initialize')
        self.get_logger().info('topic_name: ' + self.get_parameter('topic_name').get_parameter_value().string_value)
        self.get_logger().info('header_frame_id: ' + self.get_parameter('header_frame_id').get_parameter_value().string_value)
        self.get_logger().info('child_frame_id: ' + self.get_parameter('child_frame_id').get_parameter_value().string_value)

    def handle_robot_pose(self, msg):
        transform = TransformStamped()

        # ヘッダの設定
        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = self.get_parameter('header_frame_id').get_parameter_value().string_value
        transform.child_frame_id = self.get_parameter('child_frame_id').get_parameter_value().string_value

        # 位置の設定
        transform.transform.translation.x = msg.pose.position.x
        transform.transform.translation.y = msg.pose.position.y
        transform.transform.translation.z = msg.pose.position.z

        # 回転（姿勢）の設定
        transform.transform.rotation = msg.pose.orientation

        # トランスフォームの配信
        self.tf_broadcaster.sendTransform(transform)

def main(args=None):
    rclpy.init(args=args)
    node = PoseSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
