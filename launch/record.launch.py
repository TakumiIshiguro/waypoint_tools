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

from waypoint_tools.costmap import inflation_costmap_nodes
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
    # true: 既存地図上で自己位置推定しながら記録する
    #       （map_server + localization_method の推定器をこの launch で起動）。
    #       /initialpose を受けるまでは打点しない。
    # false: SLAM などが別途 map -> base_link を出している前提で記録する。
    localization = params.bool('localization')

    nodes = []

    if localization:
        map_yaml = params.path('map_yaml_path')
        if not os.path.isfile(map_yaml):
            raise RuntimeError(
                f'localization:=true requires an existing map: {map_yaml}')
        method = params.str('localization_method')
        if method == 'emcl2':
            # emcl2.launch.py が map_server と lifecycle_manager_localization も起動する。
            emcl2_args = {
                'map': map_yaml,
                'params_file': params.path('localization_params_path'),
                'use_sim_time': str(use_sim_time).lower(),
            }
            nodes.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('emcl2'),
                    'launch', 'emcl2.launch.py')),
                launch_arguments=emcl2_args.items(),
            ))
        elif method == 'ndt':
            ndt_map = params.path('ndt_map_path')
            if not os.path.isfile(ndt_map):
                raise RuntimeError(
                    f'localization_method:=ndt requires an existing PCD map: {ndt_map}')
            # ndt_localization.launch.py は map_server を起動しないので、
            # RViz 表示と costmap 用の /map はここで出す。
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
            # 1 つの params ファイルを全ノードの *_param_path に渡す
            # （ファイル内はノードごとの節に分ける）。
            ndt_params = params.path('localization_params_path')
            ndt_args = {
                f'{node}_param_path': ndt_params for node in (
                    'vehicle_velocity_converter', 'imu_corrector',
                    'gyro_odometer', 'odometry_integrator',
                    'ndt_scan_matcher', 'ekf_localizer', 'pose_initializer')
            }
            ndt_args.update({
                'map_path': ndt_map,
                'use_sim_time': str(use_sim_time).lower(),
                'rviz': 'false',
            })
            nodes.append(IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('ndt_localization'),
                    'launch', 'ndt_localization.launch.py')),
                launch_arguments=ndt_args.items(),
            ))
        else:
            raise RuntimeError(
                f'localization_method must be emcl2 or ndt: {method!r}')

    # 地図にインフレーションをかけた costmap（RViz 表示用）。
    nodes.extend(inflation_costmap_nodes(params, frame_id, use_sim_time))

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
            'localization_method',
            'localization_params_path',
            'ndt_map_path',
            'frame_id',
            'robot_frame',
            'distance_interval',
            'yaw_interval_deg',
            'min_move',
            'use_sim_time',
            'localization',
            'inflation_radius',
            'cost_scaling_factor',
        ]),
        OpaqueFunction(function=launch_setup),
    ])
