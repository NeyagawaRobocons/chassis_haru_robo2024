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
        (os.path.join('share', package_name), glob('./csv/*.csv')),
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
            'action_client = pure_pursuit.action_client:main',
            'twist_to_twist_stamped = pure_pursuit.twist_to_twist_stamped:main',
            'action_test = pure_pursuit.mecha_actions_class:main',
            'robot_master_node = pure_pursuit.robot_master_node:main',
        ],
    },
)
