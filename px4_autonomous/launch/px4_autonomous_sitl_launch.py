import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    vehicle_model = 'x500'
    
    px4 = ExecuteProcess(
        cmd=[
            os.path.expanduser('~/PX4-Autopilot/build/px4_sitl_default/bin/px4'),
            os.path.expanduser('~/PX4-Autopilot/build/px4_sitl_default/etc')
        ],
        additional_env={'PX4_SIM_MODEL': 'gz_' + vehicle_model},
        output='screen'
    )

    micrortps = ExecuteProcess(
        cmd=[
            'micro-ros-agent',
            'udp4',
            '--port',
            '8888'
        ]
    )

    px4autonomous = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('px4_autonomous'), 'launch/px4_autonomous_launch.py')
        )
    )
    
    return LaunchDescription(
        [
            px4,
            micrortps,
            px4autonomous,
        ]
    )
