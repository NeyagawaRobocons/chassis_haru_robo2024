#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Pose, PoseStamped
from pure_pursuit.action import PathAndFeedback
from pure_pursuit.msg import PointAndAngle
from mecha_actions_class import DaizaCmdActionClient, HinaCmdActionClient
from std_srvs.srv import SetBool
import pandas as pd
import os
from ament_index_python.packages import get_package_share_directory

class PathActionClient(Node):
    def __init__(self):
        super().__init__('path_action_client')
        self._action_client = ActionClient(self, PathAndFeedback, 'path_and_feedback')
        self._daiza_cmd_action_client = DaizaCmdActionClient()
        self._hina_cmd_action_client = HinaCmdActionClient()
        self._bonbori_cmd_service = self.create_client(SetBool, '/bonbori_cmd')
        self._path_pub = self.create_publisher(Path, '/robot_path', 10)
        self.get_logger().info('Path action client has been initialized')
        self.path = self.get_path()
        self.indices, self.daiza_commands, self.hina_commands, self.bonbori_commands = self.get_indices_and_commands()
        self.get_logger().info('Path and indices have been loaded')
        # self.get_logger().info(f"Path: {self.path}")
        self.get_logger().info(f"Indices: {self.indices}")
        self.get_logger().info(f"Daiza commands: {self.daiza_commands}")
        self.get_logger().info(f"Hina commands: {self.hina_commands}")
        self.get_logger().info(f"Bonbori commands: {self.bonbori_commands}")
        self.send_goal(self.path, self.indices)
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
        feedback: int = feedback_msg.feedback.current_index
        self.get_logger().info('Received feedback: {0}'.format(feedback))
        if feedback in self.indices:
            index = self.indices.index(feedback)
            daiza_state: int = int(self.daiza_commands[index])
            hina_state: int = int(self.hina_commands[index])
            bonbori_state: bool = bool(self.bonbori_commands[index])
            self.send_mecha_commands(daiza_state, hina_state, bonbori_state)

    def send_mecha_commands(self, daiza_state: int, hina_state: int, bonbori_state: bool) -> None:
        self.get_logger().info(f"daiza_state: {daiza_state}, hina_state: {hina_state}, bonbori_state: {bonbori_state}")
        if daiza_state:
            self._daiza_cmd_action_client.send_goal(daiza_state)
        elif hina_state:
            self._hina_cmd_action_client.send_goal(hina_state)
        if bonbori_state:
            self._bonbori_cmd_service(bonbori_state)

    def get_path(self):
        path = []
        df = pd.read_csv(os.path.join(get_package_share_directory('pure_pursuit'), 'csv', 'path.csv'))
        for i in range(len(df)):
            point = PointAndAngle()
            point.x = df.at[i, 'x']
            point.y = df.at[i, 'y']
            point.theta = df.at[i, 'theta']
            path.append(point)
        return path
    
    def get_indices_and_commands(self):
        indices = []
        daiza_commands = []
        hina_commands = []
        bonbori_commands = []
        df = pd.read_csv(os.path.join(get_package_share_directory('pure_pursuit'), 'csv', 'indices_and_commands.csv'))
        for i in range(len(df)):
            indices.append(df.at[i, 'index'])
            daiza_commands.append(df.at[i, 'command1'])
            hina_commands.append(df.at[i, 'command2'])
            bonbori_commands.append(df.at[i, 'command3'])
        return indices, daiza_commands, hina_commands, bonbori_commands

def main(args=None):
    rclpy.init(args=args)
    action_client = PathActionClient()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()