#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from pure_pursuit.action import PathAndFeedback
from pure_pursuit.msg import Path2DWithAngles, PointAndAngle
from mecha_control.msg import MechaState
import pandas as pd

class PathActionClient(Node):
    def __init__(self):
        super().__init__('path_action_client')
        self._action_client = ActionClient(self, PathAndFeedback, 'path_and_feedback')
        self._path_pub = self.create_publisher(Path, '/robot_path', 10)
        self.get_logger().info('Path action client has been initialized')
        path = self.get_path()
        indices = self.get_indices()
        self.send_goal(path, indices)
        self.get_logger().info('Goal has been sent')

    def send_goal(self, path: PointAndAngle, indices: list[int]) -> None:
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.poses = [PoseStamped(pose=Pose(position=Point(x=point.x, y=point.y))) for point in path]
        self._path_pub.publish(path_msg)
        self.get_logger().info('Path has been published')

        goal_msg = PathAndFeedback.Goal()
        goal_msg.path.path = path
        goal_msg.feedback_indices = [int(index) for index in indices]

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info('Goal has been sent')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.final_index))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_index))

    def get_path(self):
        path = []
        df = pd.read_csv('/home/nwrc/ros_ws/src/chassis_haru_robo2024/pure_pursuit/csv/path.csv')
        for i in range(len(df)):
            point = PointAndAngle()
            point.x = df.at[i, 'x']
            point.y = df.at[i, 'y']
            point.theta = df.at[i, 'theta']
            path.append(point)
        return path
    
    def get_indices(self):
        indices = []
        df = pd.read_csv('/home/nwrc/ros_ws/src/chassis_haru_robo2024/pure_pursuit/csv/indices_and_commands.csv')
        for i in range(len(df)):
            indices.append(df.at[i, 'index'])
        return indices

def main(args=None):
    rclpy.init(args=args)
    action_client = PathActionClient()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()