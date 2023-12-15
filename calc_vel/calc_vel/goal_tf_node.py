import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import tf2_ros
from geometry_msgs.msg import TransformStamped

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('goal_tf_node')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.handle_robot_pose,
            10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        self.get_logger().info('complete initialize')

    def handle_robot_pose(self, msg):
        transform = TransformStamped()

        # ヘッダの設定
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'map'
        transform.child_frame_id = 'goal_link'

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
