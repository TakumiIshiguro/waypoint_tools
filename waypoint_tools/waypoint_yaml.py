from pathlib import Path

import yaml


def load_config(yaml_path):
    path = Path(yaml_path).expanduser()
    with path.open('r') as yaml_file:
        config = yaml.safe_load(yaml_file)

    if not isinstance(config, dict):
        raise ValueError('YAML root must be a map.')

    if 'waypoint_server' in config:
        waypoints = config.get('waypoint_server', {}).get('waypoints')
        if not isinstance(waypoints, list):
            raise ValueError('waypoint_server.waypoints must be a list.')
        return config, 'waypoint_manager2'

    if 'waypoints' in config:
        waypoints = config.get('waypoints')
        if not isinstance(waypoints, list):
            raise ValueError('waypoints must be a list.')
        return config, 'waypoint_follower'

    raise ValueError('YAML must contain waypoint_server.waypoints or waypoints.')


def save_config(yaml_path, config):
    path = Path(yaml_path).expanduser()
    with path.open('w') as yaml_file:
        yaml.safe_dump(config, yaml_file, sort_keys=False)


def get_waypoints(config):
    if 'waypoint_server' in config:
        return config['waypoint_server']['waypoints']
    return config['waypoints']


def get_xyz_yaw(waypoint):
    if 'position' in waypoint:
        position = waypoint.get('position', {})
        euler = waypoint.get('euler_angles', {})
        return (
            float(position.get('x', 0.0)),
            float(position.get('y', 0.0)),
            float(position.get('z', 0.0)),
            float(euler.get('z', 0.0)),
        )

    return (
        float(waypoint.get('x', 0.0)),
        float(waypoint.get('y', 0.0)),
        float(waypoint.get('z', 0.0)),
        float(waypoint.get('yaw', 0.0)),
    )


def set_xyz_yaw(waypoint, x, y, z, yaw):
    if 'position' in waypoint:
        waypoint.setdefault('position', {})
        waypoint.setdefault('euler_angles', {})
        waypoint['position']['x'] = float(x)
        waypoint['position']['y'] = float(y)
        waypoint['position']['z'] = float(z)
        waypoint['euler_angles']['x'] = float(
            waypoint['euler_angles'].get('x', 0.0))
        waypoint['euler_angles']['y'] = float(
            waypoint['euler_angles'].get('y', 0.0))
        waypoint['euler_angles']['z'] = float(yaw)
        return

    waypoint['x'] = float(x)
    waypoint['y'] = float(y)
    waypoint['z'] = float(z)
    waypoint['yaw'] = float(yaw)


def make_waypoint(index, x=0.0, y=0.0, z=0.0, yaw=0.0, yaml_format='simple'):
    if yaml_format == 'waypoint_manager2':
        return {
            'id': index,
            'position': {'x': float(x), 'y': float(y), 'z': float(z)},
            'euler_angles': {'x': 0.0, 'y': 0.0, 'z': float(yaw)},
            'properties': {'goal_radius': 1.0},
        }

    return {
        'x': float(x),
        'y': float(y),
        'z': float(z),
        'yaw': float(yaw),
    }


def normalize_format_name(yaml_format):
    if yaml_format in ('simple', 'waypoint_follower'):
        return 'waypoint_follower'
    if yaml_format == 'waypoint_manager2':
        return yaml_format
    raise ValueError(
        'format must be auto, waypoint_manager2, or waypoint_follower.')


def convert_config(config, target_format):
    target_format = normalize_format_name(target_format)
    source_waypoints = get_waypoints(config)
    converted_waypoints = []

    for index, waypoint in enumerate(source_waypoints):
        x, y, z, yaw = get_xyz_yaw(waypoint)
        converted_waypoint = make_waypoint(index, x, y, z, yaw, target_format)

        if target_format == 'waypoint_manager2':
            properties = waypoint.get('properties')
            if isinstance(properties, dict):
                converted_waypoint['properties'] = dict(properties)
            if 'connections' in waypoint:
                converted_waypoint['connections'] = list(waypoint['connections'])
        converted_waypoints.append(converted_waypoint)

    if target_format == 'waypoint_manager2':
        return {
            'waypoint_server': {
                'waypoints': converted_waypoints,
            },
        }

    return {
        'waypoints': converted_waypoints,
    }
