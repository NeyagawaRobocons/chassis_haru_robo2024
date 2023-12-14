from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'calc_vel'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('./launch/*.launch.xml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tonto2423',
    maintainer_email='tnt.kosen2423@gmail.com',
    description='package for calculate velocity of robot drive tires',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_PI_controller = calc_vel.pose_PI_controller:main', 
            'localize_node = calc_vel.localize_node:main',
            'robot_tf_node = calc_vel.robot_tf_node:main',
        ],
    },
)