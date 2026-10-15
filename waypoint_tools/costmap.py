"""地図にインフレーションをかけた costmap を表示する node（edit / record 共通）."""
from launch_ros.actions import Node


def inflation_costmap_nodes(params, frame_id, use_sim_time):
    """/map に inflation_layer をかけた /costmap/costmap を publish する node を返す.

    nav2_costmap_2d の standalone node は名前が /costmap/costmap で固定。
    robot_base_frame も frame_id にして、ロボットの TF が無くても起動できるようにする。
    """
    return [
        Node(
            package='nav2_costmap_2d',
            executable='nav2_costmap_2d',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'global_frame': frame_id,
                'robot_base_frame': frame_id,
                'rolling_window': False,
                'update_frequency': 1.0,
                'publish_frequency': 1.0,
                'plugins': ['static_layer', 'inflation_layer'],
                'static_layer.plugin': 'nav2_costmap_2d::StaticLayer',
                'static_layer.map_subscribe_transient_local': True,
                'inflation_layer.plugin': 'nav2_costmap_2d::InflationLayer',
                'inflation_layer.inflation_radius': params.float('inflation_radius'),
                'inflation_layer.cost_scaling_factor': params.float('cost_scaling_factor'),
            }],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_waypoint_tools_costmap',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['costmap/costmap'],
                # 大きな地図では activate 中のインフレーション計算で bond が間に合わないので使わない。
                'bond_timeout': 0.0,
                'use_sim_time': use_sim_time,
            }],
        ),
    ]
