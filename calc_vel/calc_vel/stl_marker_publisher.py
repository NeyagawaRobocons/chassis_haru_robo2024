from visualization_msgs.msg import Marker
from geometry_msgs.msg import Pose
import rclpy
from rclpy.node import Node

class STLMarkerPublisher(Node):
    def __init__(self):
        super().__init__('stl_marker_publisher')
        self.declare_parameter('stl_file_path', '')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('topic_name')

        self.stl_file_path = self.get_parameter('stl_file_path').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value
        self.topic_name = self.get_parameter('topic_name').get_parameter_value().string_value

        self.publisher = self.create_publisher(Marker, self.topic_name, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.type = marker.MESH_RESOURCE
        marker.action = marker.ADD
        marker.pose = Pose()
        marker.scale.x = 1.0  # スケールの調整
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 1.0  # カラーの設定
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0  # アルファ値（透明度）
        marker.mesh_resource = "file://" + self.stl_file_path
        self.publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = STLMarkerPublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
