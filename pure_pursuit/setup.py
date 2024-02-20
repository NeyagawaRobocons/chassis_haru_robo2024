from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'pure_pursuit'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('./yaml/*.yaml')),
        (os.path.join('share', package_name), glob('./rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tonto2423',
    maintainer_email='tnt.kosen2423@gmail.com',
    description='pure pursuit',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit_node = pure_pursuit.pure_pursuit_node:main',
            'pure_pursuit_PI = pure_pursuit.pure_pursuit_PI:main',
            'calc_wheel_vel_node = pure_pursuit.calc_wheel_vel:main',
            'path_view_node = pure_pursuit.robot_path_view:main', 
            'debug_vel = pure_pursuit.debug_vel:main', 
            'param_controller = pure_pursuit.param_controller:main', 
            'mecha_state_publisher = pure_pursuit.mecha_state_publisher:main',
            'path_server = pure_pursuit.path_server:main',
            'robot_manual_controller = pure_pursuit.robot_manual_controller:main',
            'action_client = pure_pursuit.action_client:main',
            'path_generator_node = pure_pursuit.path_generator_node',
            'twist_visualizer = pure_pursuit.twist_visualizer:main',
            'twist_to_twist_stamped = pure_pursuit.twist_to_twist_stamped:main',
        ],
    },
)
