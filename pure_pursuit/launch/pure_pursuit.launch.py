from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory
import numpy as np

def generate_launch_description():
    # # パラメータファイルのパスを取得
    # param_file = os.path.join(
    #     get_package_share_directory('calc_vel'),  # 'your_package_name'を実際のパッケージ名に置き換える
    #     'pi_params.yaml'  # 'your_param_file.yaml'を実際のパラメータファイル名に置き換える
    # )

    return LaunchDescription([
        # localize_nodeの起動
        Node(
            package='localize',     # localize_nodeが属するパッケージ名
            executable='localize_node',     # localize_nodeの実行可能ファイル名
            name='localize_node'            # localize_nodeのノード名
        ),
        Node(
            package='odometry_calculator',
            executable='odometry_node',
            name='odometry_node'
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
            name='static_transform_publisher',           # static tfのノード名
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link']
        ),
        Node(
            package='tf2_ros',           # static tfが属するパッケージ名
            executable='static_transform_publisher',     # static tfの実行可能ファイル名
            name='static_transform_publisher',           # static tfのノード名
            arguments=['0.2699', '0.2699', '0', '0', '0', str(-np.pi * 3.0 / 4.0), 'base_link', 'laser']
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
        Node(
            package='pure_pursuit',
            executable='pure_pursuit_node.py',
            name='pure_pursuit_node',
            parameters=[
                {'speed': 0.5},
                {'lookahead_distance': 0.3},
                {'path_p_gain': 0.05},
                {'angle_p_gain': 0.1},
                {'angle_i_gain': 0.0},
                {'initial_pose': [1.562, -3.112, 0.0]},
            ]
        ),
    ])

"""
other launch file
---
ros2 launch emcl2 emcl2.launch.py 
ros2 launch ldlidar ldlidar.launch.py 
ros2 launch laser_filters angular_filter_example.launch.py 

"""