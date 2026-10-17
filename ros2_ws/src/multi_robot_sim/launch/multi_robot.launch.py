
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    TimerAction,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_description(context, *args, **kwargs):
    """Publish namespaced robot descriptions for all three robots."""
    from xacro import process_file

    actions = []

    urdf_path = os.path.join(
        get_package_share_directory('turtlebot3_description'),
        'urdf',
        'turtlebot3_burger.urdf',
    )

    for name in ('r0', 'r1', 'r2'):
        robot_description = process_file(
            urdf_path,
            mappings={'namespace': f'{name}/'},
        ).toxml()

        actions.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                namespace=name,
                name='robot_state_publisher',
                parameters=[
                    {
                        'robot_description': robot_description,
                        'use_sim_time': True,
                    }
                ],
                remappings=[
                    ('/tf', '/tf'),
                    ('/tf_static', '/tf_static'),
                ],
                output='screen',
            )
        )

    return actions


def generate_launch_description():
    """Launch Gazebo and three namespaced TurtleBot3 robots."""
    package_share = get_package_share_directory('multi_robot_sim')
    gazebo_share = get_package_share_directory('gazebo_ros')

    world = os.path.join(
        package_share,
        'worlds',
        'inspection_world.world',
    )

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    gazebo_share,
                    'launch',
                    'gzserver.launch.py',
                )
            ),
            launch_arguments={'world': world}.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    gazebo_share,
                    'launch',
                    'gzclient.launch.py',
                )
            ),
        ),
        OpaqueFunction(function=generate_description),
    ]

    # Publish the fixed base_link -> camera_link transform for each robot.
    for name in ('r0', 'r1', 'r2'):
        actions.append(
            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name=f'{name}_camera_tf',
                arguments=[
                    '0.08',
                    '0',
                    '0.12',
                    '0',
                    '0',
                    '0',
                    f'{name}/base_link',
                    f'{name}/camera_link',
                ],
                output='screen',
            )
        )

    # Spawn robots at separate positions, with delays to avoid startup races.
    robots = [
        ('r0', 0.0, 0.0),
        ('r1', 2.0, 0.0),
        ('r2', 0.0, 2.0),
    ]

    for index, (name, x, y) in enumerate(robots):
        model = os.path.join(
            package_share,
            'models',
            'turtlebot3_burger',
            f'model_{name}.sdf',
        )

        spawn = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name=f'spawn_{name}',
            arguments=[
                '-entity',
                name,
                '-file',
                model,
                '-x',
                str(x),
                '-y',
                str(y),
                '-z',
                '0.01',
            ],
            output='screen',
        )

        actions.append(
            TimerAction(
                period=5.0 + index * 5.0,
                actions=[spawn],
            )
        )

    return LaunchDescription(actions)
