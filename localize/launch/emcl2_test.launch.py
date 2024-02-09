import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
# from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    declare_serial = DeclareLaunchArgument(
        name='serial_port_candidates', 
        default_value='/dev/ttyACM1',
        description='LD06 Serial Port Candidate'
    )

    declare_topic = DeclareLaunchArgument(
        name='topic_name', 
        default_value='scan',
        description='LD06 Topic Name'
    )
    
    declare_frame = DeclareLaunchArgument(
        name='lidar_frame', 
        default_value='laser',
        description='Lidar Frame ID'
    )
    
    declare_range = DeclareLaunchArgument(
        name='range_threshold', 
        default_value='0.005',
        description='Range Threshold'
    )
    
    ldlidar = Node(
        package='ldlidar',
        executable='ldlidar',
        name='ldlidar',
        output='screen',
        parameters=[
            # {'serial_port_candidates': LaunchConfiguration("serial_port_candidates")},
            {'topic_name': LaunchConfiguration("topic_name")},
            {'lidar_frame': LaunchConfiguration("lidar_frame")},
            {'range_threshold': LaunchConfiguration("range_threshold")}
        ]
    )
    
    # tf2_ros static_transform_publishers
    base_link_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        name='base_link_to_base_footprint'
    )
    base_link_to_laser = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.24718', '0', '0', '0', '0', '3.14159265358979', 'base_link', 'laser'],
        name='base_link_to_laser'
    )

    # localize_node
    localize_node = Node(
        package='localize',
        executable='localize_node',
        name='localize_node'
    )

    # robot_tf_node with renamed node and parameters
    robot_tf_node = Node(
        package='calc_vel',
        executable='robot_tf_node',
        name='odom_tf_node',
        parameters=[{'header_frame_id': 'odom', 'child_frame_id': 'base_link', 'topic_name': 'odometry_pose'}]
    )

    # odometry_calculator_node
    odometry_node = Node(
        package='odometry_calculator',
        executable='odometry_node',
        name='odometry_node'
    )

    # nucleo_agent_node
    nucleo_agent_node = Node(
        package='nucleo_agent',
        executable='nucleo_agent_node',
        name='nuceo_agent_node'
    )
    return LaunchDescription([
        declare_serial,
        declare_topic,
        declare_frame,
        declare_range,
        ldlidar,
        base_link_to_base_footprint,
        base_link_to_laser,
        localize_node,
        robot_tf_node,
        odometry_node,
        nucleo_agent_node,
    ])