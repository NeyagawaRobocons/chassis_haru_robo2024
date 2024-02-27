from launch import LaunchDescription
from launch_ros.actions import Node
import numpy as np

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_transformation',
            executable='laser_transformation',
            name='laser_transformation_front',
            parameters=[{'frame_id': 'laser_front', 'angle_min_deg': 90.0, 'angle_max_deg': 270.0, 'input_topic': 'scan_front', 'output_topic': 'scan_cutted_front'}],
        ),
        Node(
            package='lidar_transformation',
            executable='laser_transformation',
            name='laser_transformation_back',
            parameters=[{'frame_id': 'laser_back', 'angle_min_deg': 90.0, 'angle_max_deg': 270.0, 'input_topic': 'scan_back', 'output_topic': 'scan_cutted_back'}],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0.2699', '0.2699', '0', str(np.pi / 4.0), str(-np.pi), '0', 'base_link', 'laser_front'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['-0.2699', '-0.2699', '0', str(-3.0 * np.pi / 4.0), str(np.pi), '0', 'base_link', 'laser_back'],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
        ),
        Node(
            package='calc_vel',
            executable='robot_tf_node',
            parameters=[{'header_frame_id': 'odom', 'child_frame_id': 'base_footprint', 'topic_name': 'odometry_pose'}],
        ),
        Node(
            package='localize',
            executable='tf_to_posestamped',
            name='tf_to_posestamped',
        ),
        Node(
            package='pure_pursuit',
            executable='twist_to_twiststamped.py',
            name='twist_to_twiststamped',
        ),
        # Node(
        #     package='ldlidar',
        #     executable='ldlidar',
        #     name='ldlidar_front',
        #     parameters=[{
        #         'topic_name': 'scan_front', 
        #         'lidar_frame': 'laser_front',
        #         'serial_port_candidates': ['/dev/ttySerial564D004832']
        #     }],
        # ),
        # Node(
        #     package='ldlidar',
        #     executable='ldlidar',
        #     name='ldlidar_back',
        #     parameters=[{
        #         'topic_name': 'scan_back', 
        #         'lidar_frame': 'laser_back', 
        #         'serial_port_candidates': ['/dev/ttySerial564D004832']
        #     }],
        # ),
    ])
