from launch import LaunchDescription
from launch_ros.actions import Node
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
        Node(
            package='calc_vel',
            executable='PI_simulator',
            name='PI_simulator',
            parameters=[{
                'frame_id': 'map',
                'output_name': 'robot_pose',
            }]
        ),
        Node(
            package='calc_vel',
            executable='robot_tf_node',
            name='robot_tf_node',
        ),
        # Node(
        #     package='calc_wheel_vel',
        #     executable='calc_wheel_vel',
        #     name='calc_wheel_vel',
        #     parameters=[{'num_wheels': 3}]
        # ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', 
                os.path.join(get_package_share_directory('pure_pursuit'), 'rviz', 'robot_sim.rviz')]  # rviz2の設定ファイルのパス
        ),
        Node(
            package='pure_pursuit',
            executable='twist_to_twiststamped.py',
            name='twist_to_twiststamped',
        ),
    ])