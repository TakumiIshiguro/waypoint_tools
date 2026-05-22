import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


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

    waypoint_yaml = value(
        ['send_waypoint_yaml_path', 'yaml_path', 'waypoint_yaml_path'],
        'send_waypoint_yaml_path',
        os.path.join(get_package_share_directory('waypoint_tools'),
                     'config', 'waypoints', 'sample.yaml'))
    waypoint_yamls = params.get(
        'send_waypoint_yaml_paths',
        params.get('waypoint_yaml_paths', []))
    if not isinstance(waypoint_yamls, list):
        waypoint_yamls = []
    frame_id = value('frame_id', default='map')
    action_name = value('action_name', default='/follow_waypoints')
    use_sim_time = as_bool(value('use_sim_time', default='false'))
    send_on_start = as_bool(value('send_on_start', default='true'))

    return [
        Node(
            package='waypoint_tools',
            executable='waypoint_sender_node',
            name='waypoint_sender_node',
            output='screen',
            parameters=[{
                'yaml_path': waypoint_yaml,
                'yaml_paths': waypoint_yamls,
                'frame_id': frame_id,
                'action_name': action_name,
                'send_on_start': send_on_start,
                'use_sim_time': use_sim_time,
            }],
        )
    ]


def generate_launch_description():
    package_share = get_package_share_directory('waypoint_tools')
    default_params_file = os.path.join(
        package_share, 'config', 'params', 'waypoint_tools_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Waypoint tools parameter file.'),
        DeclareLaunchArgument('yaml_path', default_value=''),
        DeclareLaunchArgument('waypoint_yaml_path', default_value=''),
        DeclareLaunchArgument('send_waypoint_yaml_path', default_value=''),
        DeclareLaunchArgument('frame_id', default_value=''),
        DeclareLaunchArgument('use_sim_time', default_value=''),
        DeclareLaunchArgument('action_name', default_value=''),
        DeclareLaunchArgument('send_on_start', default_value=''),
        OpaqueFunction(function=launch_setup),
    ])
