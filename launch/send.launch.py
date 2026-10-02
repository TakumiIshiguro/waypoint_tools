from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from waypoint_tools.paths import (
    LaunchParams, declare_overrides, source_path)


def launch_setup(context, *args, **kwargs):
    params = LaunchParams(
        context, LaunchConfiguration('params_file').perform(context))

    # ファイル or フォルダを 1 つ指定する（フォルダならファイル送りモード）。
    send_target = params.path('send_waypoint_path')
    frame_id = params.str('frame_id')
    use_sim_time = params.bool('use_sim_time')
    send_on_start = params.bool('send_on_start')

    return [
        Node(
            package='waypoint_tools',
            executable='waypoint_sender_node',
            name='waypoint_sender_node',
            output='screen',
            parameters=[{
                'yaml_path': send_target,
                'frame_id': frame_id,
                'send_on_start': send_on_start,
                'use_sim_time': use_sim_time,
            }],
        )
    ]


def generate_launch_description():
    default_params_file = source_path(
        'config', 'params', 'waypoint_tools_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Waypoint tools parameter file.'),
        *declare_overrides([
            'send_waypoint_path',
            'frame_id',
            'use_sim_time',
            'send_on_start',
        ]),
        OpaqueFunction(function=launch_setup),
    ])
