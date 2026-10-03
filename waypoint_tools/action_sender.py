import math

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

from waypoint_tools.waypoint_yaml import get_xyz_yaw


def yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def quaternion_to_yaw(quaternion):
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return math.atan2(siny_cosp, cosy_cosp)


def make_navigate_to_pose_goal(waypoint, frame_id, stamp):
    goal = NavigateToPose.Goal()
    x, y, z, yaw = get_xyz_yaw(waypoint)
    pose = PoseStamped()
    pose.header.frame_id = frame_id
    pose.header.stamp = stamp
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    qx, qy, qz, qw = yaw_to_quaternion(yaw)
    pose.pose.orientation.x = qx
    pose.pose.orientation.y = qy
    pose.pose.orientation.z = qz
    pose.pose.orientation.w = qw
    goal.pose = pose
    return goal
