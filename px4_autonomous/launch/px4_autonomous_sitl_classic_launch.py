import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, AppendEnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    vehicle_model = 'iris'
    
    px4 = ExecuteProcess(
        cmd=[
            os.path.expanduser('~/PX4-Autopilot/build/px4_sitl_default/bin/px4'),
            os.path.expanduser('~/PX4-Autopilot/build/px4_sitl_default/etc')
        ],
        additional_env={
            'PX4_SIM_MODEL': 'gazebo-classic_' + vehicle_model,
        },
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

    model_path = AppendEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic/models')
    )

    plugin_path = AppendEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        os.path.expanduser('~/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic')
    )

    ld_path = AppendEnvironmentVariable(
        'LD_LIBRARY_PATH',
        os.path.expanduser('~/PX4-Autopilot/build/px4_sitl_default/build_gazebo-classic')
    )
    
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch/gzserver.launch.py')
        ),
        launch_arguments={
            'verbose': 'true'
        }.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch/gzclient.launch.py')
        ),
        launch_arguments={
            'verbose': 'true'
        }.items()
    )

    load_vehicle = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'uav', '-database', vehicle_model]
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
            model_path,
            plugin_path,
            ld_path,
            gzserver,
            gzclient,
            load_vehicle,
            px4autonomous,
        ]
    )
