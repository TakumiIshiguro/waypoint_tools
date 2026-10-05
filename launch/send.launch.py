from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from waypoint_tools.paths import (
    LaunchParams, declare_overrides, source_path)


def launch_setup(context, *args, **kwargs):
    params = LaunchParams(
        context, LaunchConfiguration('params_file').perform(context))

    # 送信する YAML ファイルを 1 つ指定する。
    send_target = params.path('send_waypoint_path')
    frame_id = params.str('frame_id')
    use_sim_time = params.bool('use_sim_time')
    robot_frame = params.str('robot_frame')
    switch_radius = params.float('switch_radius')
    max_retries = params.int('max_retries')
    skip_on_failure = params.bool('skip_on_failure')

    return [
        Node(
            package='waypoint_tools',
            executable='waypoint_sender_node',
            name='waypoint_sender_node',
            output='screen',
            parameters=[{
                'yaml_path': send_target,
                'frame_id': frame_id,
                'robot_frame': robot_frame,
                'switch_radius': switch_radius,
                'max_retries': max_retries,
                'skip_on_failure': skip_on_failure,
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
            'robot_frame',
            'switch_radius',
            'max_retries',
            'skip_on_failure',
        ]),
        OpaqueFunction(function=launch_setup),
    ])
