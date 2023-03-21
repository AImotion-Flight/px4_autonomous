import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, AppendEnvironmentVariable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    px4_build_dir = os.path.expanduser('~/PX4-Autopilot/build/px4_sitl_default')
    px4_gazebo_classic_dir = os.path.expanduser('~/PX4-Autopilot/Tools/simulation/gazebo-classic/sitl_gazebo-classic')
    
    n = 3
    vehicle_model = 'typhoon_h480_thi'

    launch_entities = []

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
        load_vehicle = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-entity', 'uav_' + str(k),
                '-file', '/tmp/' + vehicle_model + '_' + str(k) + '.sdf',
                '-y', str(k * 2)
            ]
        )
        launch_entities.append(gen_sdf)
        launch_entities.append(px4)
        launch_entities.append(load_vehicle)
    
    return LaunchDescription(
        launch_entities
    )
