import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, AppendEnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    px4_build_dir = os.path.expanduser('~/PX4-Autopilot/build/px4_sitl_default')
    px4_gazebo_classic_dir = os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic')
    gazebo_ros_share = get_package_share_directory('gazebo_ros')
    
    vehicle_model = 'typhoon_h480_thi'
    world = 'thi'
    
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
            'world': os.path.join(px4_gazebo_classic_dir, 'worlds', world + '.world')
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

    xrce_dds_agent = ExecuteProcess(
        cmd=[
            'micro-xrce-dds-agent',
            'udp4',
            '-p',
            '8888'
        ]
    )

    launch_entities = [
        model_path,
        plugin_path,
        ld_path,
        gzserver,
        gzclient,
        xrce_dds_agent,
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(get_package_share_directory('px4_autonomous'), 'config/visualization.rviz')]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=["0", "0", "0", "0.7071068", "0.7071068", "0", "0", "px4", "gazebo"]
        )
    ]
    initial_coords = [(0.5, 0.5, 1)] # (x, y, z)
    n = 1
    for k in range(1, n + 1):
        gen_sdf = ExecuteProcess(
            cmd=[
                'python3',
                os.path.join(px4_gazebo_classic_dir, 'scripts/jinja_gen.py'),
                os.path.join(px4_gazebo_classic_dir, 'models', vehicle_model, vehicle_model + '.sdf.jinja'),
                px4_gazebo_classic_dir,
                '--mavlink_tcp_port',
                str(4560 + k),
                '--mavlink_udp_port',
                str(14560 + k),
                '--mavlink_id',
                str(k),
                '--gst_udp_port',
                str(5600 + k),
                '--video_uri',
                str(5600 + k),
                '--mavlink_cam_udp_port',
                str(14530 + k),
                '--output-file',
                '/tmp/' + vehicle_model + '_' + str(k) + '.sdf'
            ]
        )
        x = initial_coords[k - 1][0]
        y = initial_coords[k - 1][1]
        z = initial_coords[k - 1][2]
        load_vehicle = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'uav_' + str(k),
                '-file', '/tmp/' + vehicle_model + '_' + str(k) + '.sdf',
                '-x', str(x),
                '-y', str(y),
                '-z', str(z)
            ]
        )
        px4 = ExecuteProcess(
            cmd=[
                os.path.join(px4_build_dir, 'bin/px4'),
                '-i',
                str(k),
                '-d',
                os.path.join(px4_build_dir, 'etc')
            ],
            additional_env={
                'PX4_SIM_MODEL': 'gazebo-classic_' + vehicle_model,
            },
            output='screen'
        )
        transform = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments=[str(x), str(y), "0", "0.7071068", "0.7071068", "0", "0", "gazebo", "px4_uav_" + str(k)]
        )
        launch_entities.append(gen_sdf)
        launch_entities.append(load_vehicle)
        launch_entities.append(px4)
        launch_entities.append(transform)
    
    return LaunchDescription(launch_entities)
