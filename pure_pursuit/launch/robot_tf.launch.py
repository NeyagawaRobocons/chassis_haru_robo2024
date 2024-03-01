from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np

def generate_launch_description():

    return LaunchDescription([
        # static tfの起動
        Node(
            package='tf2_ros',           # static tfが属するパッケージ名
            executable='static_transform_publisher',     # static tfの実行可能ファイル名
            name='static_transform_publisher1',           # static tfのノード名
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
        ),
        Node(
            package='tf2_ros',           # static tfが属するパッケージ名
            executable='static_transform_publisher',     # static tfの実行可能ファイル名
            name='static_transform_publisher2',           # static tfのノード名
            arguments=['0.2699', '0.2699', '0', str(np.pi / 4.0), str(-np.pi), '0', 'base_link', 'laser_front'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_transform_publisher3',
            arguments=['-0.2699', '-0.2699', '0', str(-3.0 * np.pi / 4.0), str(np.pi), '0', 'base_link', 'laser_back'],
        ),
        # robot_tf_nodeの起動
        Node(
            package='calc_vel',           # robot_tf_nodeが属するパッケージ名
            executable='robot_tf_node',     # robot_tf_nodeの実行可能ファイル名
            name='robot_tf_node',           # robot_tf_nodeのノード名
            parameters=[
                {'header_frame_id': 'odom'},
                {'child_frame_id': 'base_footprint'},
                {'topic_name': 'odometry_pose'}
            ]
        ),
    ])
