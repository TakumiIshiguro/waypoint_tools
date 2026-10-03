import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from waypoint_tools.paths import (
    LaunchParams, declare_overrides, source_path)


def launch_setup(context, *args, **kwargs):
    params = LaunchParams(
        context, LaunchConfiguration('params_file').perform(context))

    output_path = params.path('record_waypoint_path')
    rviz_config = params.path('rviz_config_path')
    frame_id = params.str('frame_id')
    robot_frame = params.str('robot_frame')
    distance_interval = params.float('distance_interval')
    yaw_interval_deg = params.float('yaw_interval_deg')
    min_move = params.float('min_move')
    use_sim_time = params.bool('use_sim_time')
    # true: 既存地図上で emcl2 により自己位置推定しながら記録する
    #       （map_server + emcl2 をこの launch で起動）。
    #       /initialpose を受けるまでは打点しない。
    # false: SLAM などが別途 map -> base_link を出している前提で記録する。
    localization = params.bool('localization')

    nodes = []

    if localization:
        map_yaml = params.path('map_yaml_path')
        if not os.path.isfile(map_yaml):
            raise RuntimeError(
                f'localization:=true requires an existing map: {map_yaml}')
        # emcl2.launch.py が map_server と lifecycle_manager_localization も起動する。
        emcl2_args = {
            'map': map_yaml,
            'params_file': params.path('emcl2_params_path'),
            'use_sim_time': str(use_sim_time).lower(),
        }
        nodes.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('emcl2'),
                'launch', 'emcl2.launch.py')),
            launch_arguments=emcl2_args.items(),
        ))

    nodes.append(
        Node(
            package='waypoint_tools',
            executable='waypoint_recorder_node',
            name='waypoint_recorder_node',
            output='screen',
            parameters=[{
                'output_path': output_path,
                'frame_id': frame_id,
                'robot_frame': robot_frame,
                'distance_interval': distance_interval,
                'yaw_interval_deg': yaw_interval_deg,
                'min_move': min_move,
                'wait_for_initialpose': localization,
                'use_sim_time': use_sim_time,
            }],
        )
    )

    nodes.append(
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2_waypoint_record',
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
            'record_waypoint_path',
            'map_yaml_path',
            'rviz_config_path',
            'emcl2_params_path',
            'frame_id',
            'robot_frame',
            'distance_interval',
            'yaw_interval_deg',
            'min_move',
            'use_sim_time',
            'localization',
        ]),
        OpaqueFunction(function=launch_setup),
    ])
