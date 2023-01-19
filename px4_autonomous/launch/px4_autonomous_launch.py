import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='px4_autonomous',
            executable='offboard'
        ),
        Node(
            package='px4_autonomous',
            executable='transformation'
        ),
        Node(
            package='px4_autonomous',
            executable='visualization'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["0", "0", "0", "0", "0", "0", "map", "world"]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('px4_autonomous'), 'config/visualization.rviz')]
        )
    ])
