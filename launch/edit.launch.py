import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from waypoint_tools.paths import resolve_path

DEFAULT_MAP = (
    'pkg://orne_box_navigation_executor/config/maps/'
    'tsudanuma/tsudanuma_keepout.yaml')


def load_params(params_file):
    with open(params_file, 'r') as yaml_file:
        config = yaml.safe_load(yaml_file) or {}

    if 'waypoint_tools' in config:
        return config['waypoint_tools'].get('ros__parameters', {})
    return config.get('ros__parameters', config)


def as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).lower() in ('1', 'true', 'yes', 'on')


def launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file').perform(context)
    params = load_params(params_file)

    def value(names, param_name=None, default=''):
        if isinstance(names, str):
            names = [names]
        for name in names:
            override = LaunchConfiguration(name).perform(context)
            if override:
                return override
        return str(params.get(param_name or names[0], default))

    # ファイル or フォルダを 1 つ指定する（フォルダならファイル送りモード）。
    edit_target = resolve_path(value(
        ['edit_waypoint_path', 'yaml_path', 'waypoint_yaml_path'],
        'edit_waypoint_path', 'config/waypoints/sample.yaml'))
    map_yaml = resolve_path(value(
        ['map', 'map_yaml_path'], 'map_yaml_path', DEFAULT_MAP))
    rviz_config = resolve_path(value(
        ['rviz_config', 'rviz_config_path'], 'rviz_config_path',
        'config/rviz/waypoint_tools.rviz'))
    frame_id = value('frame_id', default='map')
    use_sim_time = as_bool(value('use_sim_time', default='false'))
    # 既定 ON: 編集時は既存マップの上で waypoint を並べる。params では制御しない。
    # マップ不要なら start_map:=false。
    start_map_arg = LaunchConfiguration('start_map').perform(context)
    start_map = as_bool(start_map_arg) if start_map_arg else True

    nodes = []

    if start_map:
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
    package_share = get_package_share_directory('waypoint_tools')
    default_params_file = os.path.join(
        package_share, 'config', 'params', 'waypoint_tools_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Waypoint tools parameter file.'),
        DeclareLaunchArgument('edit_waypoint_path', default_value=''),
        DeclareLaunchArgument('yaml_path', default_value=''),
        DeclareLaunchArgument('waypoint_yaml_path', default_value=''),
        DeclareLaunchArgument('map', default_value=''),
        DeclareLaunchArgument('map_yaml_path', default_value=''),
        DeclareLaunchArgument('rviz_config', default_value=''),
        DeclareLaunchArgument('rviz_config_path', default_value=''),
        DeclareLaunchArgument('frame_id', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value=''),
        DeclareLaunchArgument('start_map', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
