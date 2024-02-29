#!/usr/bin/env python3
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, PoseWithCovarianceStamped
from pure_pursuit.action import PathAndFeedback
from pure_pursuit.msg import Pose2DWithSpeed
from mecha_actions_class import DaizaCmdActionClient, HinaCmdActionClient
from std_msgs.msg import Bool
from std_srvs.srv import SetBool
import numpy as np
from numpy.typing import NDArray
import pandas as pd
import time
import os
from ament_index_python.packages import get_package_share_directory
from tf_transformations import quaternion_from_euler

class PathActionClient(Node):
    def __init__(self):
        super().__init__('path_action_client')
        self._action_client = ActionClient(self, PathAndFeedback, 'path_and_feedback')
        self._daiza_cmd_action_client = DaizaCmdActionClient()
        self._hina_cmd_action_client = HinaCmdActionClient()
        self._bonbori_cmd_service = self.create_client(SetBool, '/set_bonbori')
        self._path_pub = self.create_publisher(Path, '/robot_path', 10)
        self._initialpose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self._get_mcl_pose_pub = self.create_publisher(Bool, '/get_mcl_pose', 10)

        self.declare_parameters(
            namespace='',
            parameters=[
                ('path_number', 1),
            ]
        )
        self.path_number: int = self.get_parameter('path_number').value
        self.path_file_name: str = f'path{self.path_number}.csv'
        self.indices_file_name: str = f'indices_and_commands{self.path_number}.csv'

        self.path = self.get_path()
        self.indices, self.daiza_commands, self.hina_commands, self.bonbori_commands, self.set_pose_index = self.get_indices_and_commands()
        self.pre_feedback = -1
        self.get_logger().info('Path and indices have been loaded')
        self.get_logger().info(f"path_number: {self.path_number}")
        self.get_logger().info(f"path_file_name: {self.path_file_name}")
        self.get_logger().info(f"indices_file_name: {self.indices_file_name}")
        # self.get_logger().info(f"Path: {self.path}")
        self.get_logger().info(f"first point of path: {self.path[0]}")
        self.get_logger().info(f"Indices: {self.indices}")
        self.get_logger().info(f"first commands: {self.daiza_commands[0]}, {self.hina_commands[0]}, {self.bonbori_commands[0]}")
        self.get_logger().info(f"Daiza commands: {self.daiza_commands}")
        self.get_logger().info(f"Hina commands: {self.hina_commands}")
        self.get_logger().info(f"Bonbori commands: {self.bonbori_commands}")
        self.get_logger().info(f"Set pose index: {self.set_pose_index}")
        self.send_goal(self.path, self.indices)
        self.get_logger().info('Goal has been sent')

        self.get_logger().info('Path action client has been initialized')

    def send_goal(self, path: Pose2DWithSpeed, indices: list[int]) -> None:
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.poses = [PoseStamped(pose=Pose(position=Point(x=point.x, y=point.y), orientation=self.yaw_to_quaternion(point.theta + np.pi/2))) for point in path]
        for _ in range(10):
            self._path_pub.publish(path_msg)
            time.sleep(0.1)
        self.get_logger().info('Path has been published')

        self.set_pose(path=self.path, index=0)

        goal_msg = PathAndFeedback.Goal()
        goal_msg.path.path = path
        goal_msg.feedback_indices = [int(index) for index in indices]
        goal_msg.set_pose_index = self.set_pose_index

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.get_logger().info('Goal has been sent')

    def yaw_to_quaternion (self, yaw: float) -> Quaternion:
        q_raw = quaternion_from_euler(0, 0, yaw)
        q = Quaternion(x=q_raw[0], y=q_raw[1], z=q_raw[2], w=q_raw[3])
        return q

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Path Goal rejected :(')
            return

        self.get_logger().info('Path Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result.final_index
        self.get_logger().info('Path Result: {0}'.format(result))
        if result in self.indices:
            index = self.indices.index(result)
            daiza_state: int = int(self.daiza_commands[index])
            hina_state: int = int(self.hina_commands[index])
            bonbori_state: bool = bool(self.bonbori_commands[index])
            self.send_mecha_commands(daiza_state, hina_state, bonbori_state)

    def feedback_callback(self, feedback_msg):
        feedback: int = feedback_msg.feedback.current_index
        self.get_logger().info('Path Received feedback: {0}'.format(feedback))
        if feedback == self.pre_feedback: # 連続して同じfeedbackが送られてくることがあるため
            return
        if feedback in self.indices:
            index = self.indices.index(feedback)
            daiza_state: int = int(self.daiza_commands[index])
            hina_state: int = int(self.hina_commands[index])
            bonbori_state: bool = bool(self.bonbori_commands[index])
            self.send_mecha_commands(daiza_state, hina_state, bonbori_state)
            # if self.path[feedback].speed / self.max_speed < 0.5:
            #     self.set_pose(path=self.path, index=feedback)
        self.pre_feedback = feedback

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
        df = pd.read_csv(os.path.join(get_package_share_directory('pure_pursuit'), 'csv', self.path_file_name))
        for i in range(len(df)):
            point = Pose2DWithSpeed()
            point.x = df.at[i, 'x']
            point.y = df.at[i, 'y']
            point.theta = df.at[i, 'theta']
            point.speed = df.at[i, 'speed']
            path.append(point)
        return path

    def get_indices_and_commands(self):
        indices = []
        daiza_commands = []
        hina_commands = []
        bonbori_commands = []
        set_pose_index: int = 0
        df = pd.read_csv(os.path.join(get_package_share_directory('pure_pursuit'), 'csv', self.indices_file_name))
        for i in range(len(df)):
            indices.append(df.at[i, 'index'])
            daiza_commands.append(df.at[i, 'command1'])
            hina_commands.append(df.at[i, 'command2'])
            bonbori_commands.append(df.at[i, 'command3'])
            if df.at[i, 'set_pose_flag']:
                set_pose_index = int(df.at[i, 'index'])

        return indices, daiza_commands, hina_commands, bonbori_commands, set_pose_index
    
    def set_pose(self, path: NDArray[np.float64], index: int) -> None:
        initialpose_msg = PoseWithCovarianceStamped()
        initialpose_msg.header.frame_id = 'map'
        initialpose_msg.pose.pose = Pose(position=Point(x=path[index].x, y=path[index].y), orientation=self.yaw_to_quaternion(path[index].theta))
        self._initialpose_pub.publish(initialpose_msg)

        for _ in range(5):
            self._get_mcl_pose_pub.publish(Bool(data=True))
            time.sleep(0.1)
        for _ in range(5):
            self._get_mcl_pose_pub.publish(Bool(data=False))
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    action_client = PathActionClient()
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()