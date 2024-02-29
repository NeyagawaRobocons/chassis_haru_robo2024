from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np

def generate_launch_description():

    return LaunchDescription([
        # Node(
        #     package='ldlidar',
        #     executable='ldlidar',
        #     name='lidar_front',
        #     parameters=[{
        #         'topic_name': '/lidar_front',
        #         'frame_id': 'laser_front',
        #         'serial_port_candidates': ['/dev/ttySerial564D005091']
        #     }]
        # ),
        # Node(
        #     package='ldlidar',
        #     executable='ldlidar',
        #     name='lidar_back',
        #     parameters=[{
        #         'topic_name': '/lidar_back',
        #         'frame_id': 'laser_back',
        #         'serial_port_candidates': ['/dev/ttySerial564D004832']
        #     }]
        # ),
        Node(
            package='nucleo_agent',
            executable='nucleo_agent_node',
            name='nucleo_agent_node',
        ),
        Node(
            package='nucleo_agent',
            executable='rp_encoder_agent_node',
            name='rp_encoder_agent_node',
        ),
        Node(
            package='mecha_control',
            executable='cmd_seq',
            name='cmd_seq',
        ),
        Node(
            package='localize',
            executable='pose_rate_change',
            name='pose_rate_change',
        ),
        Node(
            package='odometry_calculator',
            executable='odometry_node',
            name='odometry_node',
            output='log',
            parameters=[
                {'frame_id': 'odom'},
                {'output_topic': 'odometry_pose'},
            ]
        ),
        Node(
            package='odometry_calculator',
            executable='odometry_node',
            name='corrected_pose_node',
            output='log',
            parameters=[
                {'frame_id': 'map'},
                {'output_topic': 'corrected_pose'},
            ]
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
        Node(
            package='calc_wheel_vel',           # calc_wheel_vel_nodeが属するパッケージ名
            executable='calc_wheel_vel',     # calc_wheel_vel_nodeの実行可能ファイル名
            name='calc_wheel_vel',           # calc_wheel_vel_nodeのノード名
        ),
        # rviz2の起動
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 
                os.path.join(get_package_share_directory('pure_pursuit'), 'rviz', 'robot_tf.rviz')]  # rviz2の設定ファイルのパス
        ),
        Node(
            package='pure_pursuit',
            executable='twist_to_twiststamped.py',
            name='twist_to_twiststamped',
        ),
    ])
