"""Setuptools entry point for the care_robot ROS 2 package."""

from setuptools import find_packages
from setuptools import setup

package_name = 'care_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abeer',
    maintainer_email='abeerasif232@gmail.com',
    description=(
        'Runtime nodes for RGB-D human following and Gazebo Sim actor support.'),
    license='Proprietary',
    tests_require=['pytest'],
    extras_require={
        'perception': ['mediapipe>=0.10,<1.0'],
    },
    entry_points={
        'console_scripts': [
            'human_follower = care_robot.follower_node:main',
            'actor_proxy_follower = care_robot.actor_box_node:main',
            # Backward-compatible executable names.
            'actor_collision_follower = care_robot.actor_box_node:main',
            'follower_node = care_robot.follower_node:main',
            'actor_box_node = care_robot.actor_box_node:main',
        ],
    },
)
