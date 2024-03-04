#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from pure_pursuit.msg import Pose2DWithParams, Path2DWithParams
from pure_pursuit.srv import GetPath
import pandas as pd
import os

class PathServerNode(Node):
    def __init__(self):
        super().__init__('path_server_node')
        self.get_path_service = self.create_service(GetPath, 'get_path', self.get_path_callback)
        self.get_logger().info('Path server node has been initialized')

    def get_path_callback(self, request, response):
        path_file_name = f'path{request.path_number}.csv'
        indices_file_name = f'indices_and_commands{request.path_number}.csv'
        path, distance_threshold, angle_threshold = self.get_path(path_file_name)
        indices, daiza_commands, hina_commands, bonbori_commands = self.get_indices_and_commands(indices_file_name)
        response.path.path = path
        response.path.distance_threshold = distance_threshold
        response.path.angle_threshold = angle_threshold
        response.indices = indices
        response.daiza_commands = daiza_commands
        response.hina_commands = hina_commands
        response.bonbori_commands = bonbori_commands
        self.get_logger().info(f'response: {response}')
        return response

    def get_path(self, path_file_name='path1.csv'):
        path = []
        distance_threshold: float = 0.0
        angle_threshold: float = 0.0
        df = pd.read_csv(os.path.join(get_package_share_directory('pure_pursuit'), 'csv', path_file_name))
        for i in range(len(df)):
            point = Pose2DWithParams()
            point.x = df.at[i, 'x']
            point.y = df.at[i, 'y']
            point.theta = df.at[i, 'theta']
            point.speed = df.at[i, 'speed']
            point.lookahead_distance = df.at[i, 'lookahead_distance']
            point.angle_p_gain = df.at[i, 'p_gain']
            point.angle_i_gain = df.at[i, 'i_gain']
            point.path_p_gain = df.at[i, 'path_p_gain']
            point.path_i_gain = df.at[i, 'path_i_gain']
            path.append(point)
        distance_threshold = df.at[0, 'distance_threshold']
        angle_threshold = df.at[0, 'angle_threshold']
        return path, distance_threshold, angle_threshold

    def get_indices_and_commands(self, indices_file_name='indices1.csv'):
        indices = []
        daiza_commands = []
        hina_commands = []
        bonbori_commands = []
        df = pd.read_csv(os.path.join(get_package_share_directory('pure_pursuit'), 'csv', indices_file_name))
        for i in range(len(df)):
            indices.append(int(df.at[i, 'index']))
            daiza_commands.append(int(df.at[i, 'command1']))
            hina_commands.append(int(df.at[i, 'command2']))
            bonbori_commands.append(bool(df.at[i, 'command3']))

        return indices, daiza_commands, hina_commands, bonbori_commands

def main(args=None):
    rclpy.init(args=args)
    path_server_node = PathServerNode()
    rclpy.spin(path_server_node)
    path_server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()