from glob import glob

from setuptools import setup


package_name = 'waypoint_tools'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
            glob('launch/*.launch.py')),
        ('share/' + package_name + '/config/params',
            glob('config/params/*.yaml')),
        ('share/' + package_name + '/config/rviz',
            glob('config/rviz/*.rviz')),
        ('share/' + package_name + '/config/waypoints',
            glob('config/waypoints/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='takumi',
    maintainer_email='takumi@example.com',
    description='GUI waypoint editor and FollowWaypoints sender.',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_editor_node = waypoint_tools.waypoint_editor_node:main',
            'waypoint_follower_sender_node = waypoint_tools.waypoint_follower_sender_node:main',
        ],
    },
)
