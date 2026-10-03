from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from waypoint_tools.paths import (
    LaunchParams, declare_overrides, source_path)


def launch_setup(context, *args, **kwargs):
    params = LaunchParams(
        context, LaunchConfiguration('params_file').perform(context))

    # 編集する YAML ファイルを 1 つ指定する。
    edit_target = params.path('edit_waypoint_path')
    rviz_config = params.path('rviz_config_path')
    frame_id = params.str('frame_id')
    use_sim_time = params.bool('use_sim_time')
    start_map = params.bool('edit_start_map')

    nodes = []

    if start_map:
        map_yaml = params.path('map_yaml_path')
        nodes.extend([
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                parameters=[{
                    'yaml_filename': map_yaml,
                    'use_sim_time': use_sim_time,
                }],
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_waypoint_tools_map',
                output='screen',
                parameters=[{
                    'autostart': True,
                    'node_names': ['map_server'],
                    'use_sim_time': use_sim_time,
                }],
            ),
        ])

    nodes.append(
        Node(
            package='waypoint_tools',
            executable='waypoint_editor_node',
            name='waypoint_editor_node',
            output='screen',
            parameters=[{
                'yaml_path': edit_target,
                'frame_id': frame_id,
                'use_sim_time': use_sim_time,
            }],
        )
    )

    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_waypoint_tools',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen',
        )
    )

    return nodes


def generate_launch_description():
    default_params_file = source_path(
        'config', 'params', 'waypoint_tools_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Waypoint tools parameter file.'),
        *declare_overrides([
            'edit_waypoint_path',
            'map_yaml_path',
            'rviz_config_path',
            'frame_id',
            'use_sim_time',
            'edit_start_map',
        ]),
        OpaqueFunction(function=launch_setup),
    ])
