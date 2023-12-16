from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # パラメータファイルのパスを取得
    param_file = os.path.join(
        get_package_share_directory('calc_vel'),  # 'your_package_name'を実際のパッケージ名に置き換える
        'pi_params.yaml'  # 'your_param_file.yaml'を実際のパラメータファイル名に置き換える
    )

    return LaunchDescription([
        # localize_nodeの起動
        Node(
            package='calc_vel',     # localize_nodeが属するパッケージ名
            executable='localize_node',     # localize_nodeの実行可能ファイル名
            name='localize_node'            # localize_nodeのノード名
        ),
        # robot_tf_nodeの起動
        Node(
            package='calc_vel',           # robot_tf_nodeが属するパッケージ名
            executable='robot_tf_node',     # robot_tf_nodeの実行可能ファイル名
            name='robot_tf_node'            # robot_tf_nodeのノード名
        ),
        # calc_velの起動
        Node(
            package='calc_vel',     # calc_velが属するパッケージ名
            executable='pose_PI_controller',          # calc_velの実行可能ファイル名
            name='calc_vel',                 # calc_velのノード名
            output='screen',
            parameters=[param_file]
        ),
        # pose_PI_simulatorの起動
        Node(
            package='calc_vel',  # pose_PI_simulatorが属するパッケージ名
            executable='PI_simulator', # pose_PI_simulatorの実行可能ファイル名
            name='pose_PI_simulator'        # pose_PI_simulatorのノード名
        ),
        Node(
            package='calc_vel',  # goal_tf_nodeが属するパッケージ名
            executable='goal_tf_node', # goal_tf_nodeの実行可能ファイル名
            name='goal_tf_node'        # goal_tf_nodeのノード名
        ), 
        Node(
            package='calc_vel',  # param_control_nodeが属するパッケージ名
            executable='param_control_node', # param_control_nodeの実行可能ファイル名
            name='param_control_node'        # param_control_nodeのノード名
        ),
    ])
