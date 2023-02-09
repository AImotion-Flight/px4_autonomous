import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, AppendEnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    share_dir = get_package_share_directory('px4_autonomous')
    px4_build_dir = os.path.expanduser('~/PX4-Autopilot/build/px4_sitl_default')
    px4_gazebo_classic_dir = os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    
    vehicle_model = 'iris'
    world = 'custom'

    px4 = ExecuteProcess(
        cmd=[
            os.path.join(px4_build_dir, 'bin/px4'),
            os.path.join(px4_build_dir, 'etc')
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
        os.path.join(px4_gazebo_classic_dir, 'models')
    )

    plugin_path = AppendEnvironmentVariable(
        'GAZEBO_PLUGIN_PATH',
        os.path.join(px4_build_dir, 'build_gazebo-classic')
    )

    ld_path = AppendEnvironmentVariable(
        'LD_LIBRARY_PATH',
        os.path.join(px4_build_dir, 'build_gazebo-classic')
    )
    
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch/gzserver.launch.py')
        ),
        launch_arguments={
            'verbose': 'true',
            'world': os.path.join(share_dir, 'worlds', world + '.world')
        }.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_share, 'launch/gzclient.launch.py')
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
            os.path.join(share_dir, 'launch/px4_autonomous_launch.py')
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
            #px4autonomous,
        ]
    )
