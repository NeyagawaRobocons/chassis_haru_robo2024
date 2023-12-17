import rclpy
from rclpy.node import Node
from path_server.msg import OrientedPath
from geometry_msgs.msg import Pose, Vector3

class PathPublisher(Node):
    def __init__(self):
        super().__init__('path_server_node')
        self.publisher = self.create_publisher(OrientedPath, 'path', 10)
        self.timer = self.create_timer(1.0, self.publish_path)

    def publish_path(self):
        # 直線経路を作成
        path = OrientedPath()
        path.header.frame_id = "map"

        # 経路上の点を定義（例：直線経路）
        for i in range(10):
            pose = Pose()
            pose.position.x = float(i)
            pose.position.y = 0.0
            pose.position.z = 0.0
            velocity = Vector3()
            velocity.x = 0.5
            path.poses.append({'pose': pose, 'velocity': velocity})

        # 経路をパブリッシュ
        self.publisher.publish(path)
        self.get_logger().info('Publishing path')

def main(args=None):
    rclpy.init(args=args)
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
