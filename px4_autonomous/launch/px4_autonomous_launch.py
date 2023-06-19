from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    launch_entities = []

    # For multiple UAVs add tuples of the form (x, y, z, partner_id) to this array
    initial_args = [(0.5, 0.5, 1, 5), (-0.5, 0.5, 1, 3), (-1.5, 0.5, 1, 2)]
    n = len(initial_args)
    for k in range(1, n + 1):
        x = initial_args[k - 1][0]
        y = initial_args[k - 1][1]
        partner_id = initial_args[k - 1][3]
        launch_entities.append(
            Node(
                package='px4_autonomous',
                executable='offboard',
                namespace='uav_' + str(k),
                parameters=[{
                    'uav_id': k,
                    'system_id': 99 + k,
                    'partner_id': partner_id
                }],
                output='screen'
            )
        )
        launch_entities.append(
            Node(
                package='px4_autonomous',
                executable='transformation',
                namespace='uav_' + str(k),
                parameters=[{
                    'uav_id': k,
                    'initial_x': x,
                    'initial_y': y
                }]
            )
        )
        launch_entities.append(
            Node(
                package='px4_autonomous',
                executable='visualization',
                namespace='uav_' + str(k),
                parameters=[{
                    'uav_id': k,
                    'initial_x': x,
                    'initial_y': y
                }]
            )
        )

    return LaunchDescription(launch_entities)
