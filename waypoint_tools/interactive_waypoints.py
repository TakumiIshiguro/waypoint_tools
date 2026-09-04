"""RViz interactive marker の組み立て（editor / recorder 共通）."""
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    Marker,
)

from waypoint_tools.action_sender import yaw_to_quaternion


def make_disc_marker(scale):
    marker = Marker()
    marker.type = Marker.CYLINDER
    marker.scale.x = scale
    marker.scale.y = scale
    marker.scale.z = 0.04
    marker.color.r = 0.1
    marker.color.g = 0.7
    marker.color.b = 1.0
    marker.color.a = 0.45
    return marker


def make_arrow_marker(scale):
    marker = Marker()
    marker.type = Marker.ARROW
    marker.scale.x = max(scale * 0.6, 0.35)
    marker.scale.y = 0.08
    marker.scale.z = 0.08
    marker.color.r = 1.0
    marker.color.g = 0.2
    marker.color.b = 0.1
    marker.color.a = 1.0
    return marker


def build_waypoint_marker(name, frame_id, x, y, yaw, scale, description=''):
    """位置移動(平面) + yaw 回転 + メニュー用ボタンを持つ InteractiveMarker."""
    marker = InteractiveMarker()
    marker.header.frame_id = frame_id
    marker.name = name
    marker.description = description
    marker.scale = float(scale)
    marker.pose.position.x = float(x)
    marker.pose.position.y = float(y)
    marker.pose.position.z = 0.0
    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    marker.pose.orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

    move_control = InteractiveMarkerControl()
    move_control.orientation.w = 1.0
    move_control.orientation.x = 0.0
    move_control.orientation.y = 1.0
    move_control.orientation.z = 0.0
    move_control.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
    move_control.orientation_mode = InteractiveMarkerControl.INHERIT
    move_control.always_visible = True
    move_control.markers.append(make_disc_marker(scale))
    marker.controls.append(move_control)

    rotate_control = InteractiveMarkerControl()
    rotate_control.orientation.w = 1.0
    rotate_control.orientation.x = 0.0
    rotate_control.orientation.y = 1.0
    rotate_control.orientation.z = 0.0
    rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
    rotate_control.orientation_mode = InteractiveMarkerControl.INHERIT
    rotate_control.always_visible = True
    rotate_control.markers.append(make_arrow_marker(scale))
    marker.controls.append(rotate_control)

    menu_control = InteractiveMarkerControl()
    menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
    marker.controls.append(menu_control)

    return marker
