import re
from pathlib import Path

import yaml


def _natural_key(name):
    return [int(part) if part.isdigit() else part
            for part in re.split(r'(\d+)', name)]


def list_waypoint_yamls(directory):
    """directory 内の *.yaml を数値順（1, 2, ..., 10）でソートして返す."""
    directory = Path(directory).expanduser()
    paths = [p for p in directory.glob('*.yaml') if p.is_file()]
    paths.sort(key=lambda p: _natural_key(p.name))
    return [str(p) for p in paths]


def load_config(yaml_path):
    path = Path(yaml_path).expanduser()
    with path.open('r') as yaml_file:
        config = yaml.safe_load(yaml_file)

    if not isinstance(config, dict):
        raise ValueError('YAML root must be a map.')

    waypoints = config.get('waypoints')
    if not isinstance(waypoints, list):
        raise ValueError('YAML must contain a top-level "waypoints" list.')

    return config


def save_config(yaml_path, config):
    path = Path(yaml_path).expanduser()
    with path.open('w') as yaml_file:
        yaml.safe_dump(config, yaml_file, sort_keys=False)


def empty_config():
    return {'waypoints': []}


def get_waypoints(config):
    return config['waypoints']


def get_xyz_yaw(waypoint):
    return (
        float(waypoint.get('x', 0.0)),
        float(waypoint.get('y', 0.0)),
        float(waypoint.get('z', 0.0)),
        float(waypoint.get('yaw', 0.0)),
    )


def set_xyz_yaw(waypoint, x, y, z, yaw):
    waypoint['x'] = float(x)
    waypoint['y'] = float(y)
    waypoint['z'] = float(z)
    waypoint['yaw'] = float(yaw)


def make_waypoint(x=0.0, y=0.0, z=0.0, yaw=0.0):
    return {
        'x': float(x),
        'y': float(y),
        'z': float(z),
        'yaw': float(yaw),
    }
