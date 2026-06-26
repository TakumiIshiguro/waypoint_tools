#!/usr/bin/env python3
import copy

# 追加
import glob
import os
from rcl_interfaces.msg import SetParametersResult

import rclpy
from geometry_msgs.msg import Point, Quaternion
from interactive_markers import InteractiveMarkerServer, MenuHandler
from rclpy.node import Node
from std_srvs.srv import Trigger
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
    MarkerArray,
)

from waypoint_tools.action_sender import (
    quaternion_to_yaw,
    yaw_to_quaternion,
)
from waypoint_tools.waypoint_yaml import (
    convert_config,
    get_waypoints,
    get_xyz_yaw,
    load_config,
    normalize_format_name,
    save_config,
    set_xyz_yaw,
)


DEFAULT_WAYPOINT_YAML_PATH = (
    '/home/takumi/ros2_ws/src/orne-box/orne_box_navigation_executor/'
    'config/waypoints/tsudanuma2-3.yaml'
)


class WaypointEditorNode(Node):
    def __init__(self):
        super().__init__('waypoint_editor_node')

        self.declare_parameter('yaml_path', DEFAULT_WAYPOINT_YAML_PATH)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('route_topic', '~/routes')
        self.declare_parameter('edit_format', 'auto')
        # フォルダ指定パラメータ（空文字 = 単体ファイルモード)
        self.declare_parameter('yaml_dir', '')

        self.yaml_path = self.get_parameter(
            'yaml_path').get_parameter_value().string_value
        self.frame_id = self.get_parameter(
            'frame_id').get_parameter_value().string_value
        route_topic = self.get_parameter(
            'route_topic').get_parameter_value().string_value
        edit_format = self.get_parameter(
            'edit_format').get_parameter_value().string_value
        yaml_dir = self.get_parameter(
            'yaml_dir').get_parameter_value().string_value

        # -------------------------------------------------------
        # フォルダモードの初期化
        self.yaml_files = []    # フォルダ内 yaml ファイル一覧
        self.file_index = 0     # 現在のインデックス
        if yaml_dir:
            self._scan_yaml_dir(yaml_dir)
            if self.yaml_files:
                self.yaml_path = self.yaml_files[0]
        # -------------------------------------------------------

        self.config, self.yaml_format = load_config(self.yaml_path)
        if edit_format != 'auto':
            self.set_format(edit_format)

        self.server = InteractiveMarkerServer(self, 'waypoint_editor')
        self.menu_handler = MenuHandler()
        self.format_menu_handles = {}

        self.route_pub = self.create_publisher(MarkerArray, route_topic, 10)
        self.save_service = self.create_service(
            Trigger, '~/save', self.save_callback)
        self.reload_service = self.create_service(
            Trigger, '~/reload', self.reload_callback)

        # -------------------------------------------------------
        # next / prev サービス
        self.next_service = self.create_service(
            Trigger, '~/next_file', self.next_file_callback)
        self.prev_service = self.create_service(
            Trigger, '~/prev_file', self.prev_file_callback)
        # -------------------------------------------------------
        # yaml_dir パラメータの動的変更を監視
        self.add_on_set_parameters_callback(self.on_params_changed)
        # -----------------------------------------------------

        self.init_menu()
        self.rebuild_markers()
        self.timer = self.create_timer(0.5, self.publish_routes)

        self.get_logger().info(f'Loaded waypoints: {self.yaml_path}')
        self.get_logger().info(f'Edit format: {self.yaml_format}')

    # -----------------------------------------------------------
    # フォルダスキャン
    # -----------------------------------------------------------
    def _scan_yaml_dir(self, yaml_dir):
        """yaml_dir 内の .yaml ファイルをソートして self.yaml_files に格納する"""
        pattern = os.path.join(yaml_dir, '*.yaml')
        files = sorted(glob.glob(pattern))
        if not files:
            self.get_logger().warn(f'No yaml files found in: {yaml_dir}')
        self.yaml_files = files
        self.file_index = 0
        self.get_logger().info(
            f'Found {len(self.yaml_files)} yaml files in {yaml_dir}')

    # -----------------------------------------------------------
    # パラメータ動的変更コールバック（yaml_dir の変更に対応）
    # -----------------------------------------------------------
    def on_params_changed(self, params):
        for p in params:
            if p.name == 'yaml_dir' and p.value:
                self._scan_yaml_dir(p.value)
                if self.yaml_files:
                    self._load_file(self.yaml_files[0])
        return SetParametersResult(successful=True)

    # -----------------------------------------------------------
    # ファイル切り替えの共通処理
    # -----------------------------------------------------------
    def _load_file(self, path):
        self.yaml_path = path
        self.config, self.yaml_format = load_config(self.yaml_path)
        self.update_format_menu_checks()
        self.rebuild_markers()
        self.get_logger().info(
            f'[{self.file_index + 1}/{len(self.yaml_files)}] '
            f'Loaded: {os.path.basename(self.yaml_path)}')

    # -----------------------------------------------------------
    # next / prev サービスコールバック
    # -----------------------------------------------------------
    def next_file_callback(self, request, response):
        if not self.yaml_files:
            response.success = False
            response.message = 'yaml_dir is not set or no yaml files found.'
            return response
        if self.file_index >= len(self.yaml_files) - 1:
            response.success = False
            response.message = (
                f'Already at the last file '
                f'({self.file_index + 1}/{len(self.yaml_files)}): '
                f'{os.path.basename(self.yaml_path)}')
            return response

        self.file_index += 1
        self._load_file(self.yaml_files[self.file_index])
        response.success = True
        response.message = (
            f'[{self.file_index + 1}/{len(self.yaml_files)}] '
            f'{os.path.basename(self.yaml_path)}')
        return response

    def prev_file_callback(self, request, response):
        if not self.yaml_files:
            response.success = False
            response.message = 'yaml_dir is not set or no yaml files found.'
            return response
        if self.file_index <= 0:
            response.success = False
            response.message = (
                f'Already at the first file '
                f'({self.file_index + 1}/{len(self.yaml_files)}): '
                f'{os.path.basename(self.yaml_path)}')
            return response

        self.file_index -= 1
        self._load_file(self.yaml_files[self.file_index])
        response.success = True
        response.message = (
            f'[{self.file_index + 1}/{len(self.yaml_files)}] '
            f'{os.path.basename(self.yaml_path)}')
        return response


    def init_menu(self):
        self.menu_handler.insert('insert after', callback=self.insert_callback)
        self.menu_handler.insert('delete', callback=self.delete_callback)
        self.menu_handler.insert('save', callback=self.menu_save_callback)
        format_menu = self.menu_handler.insert('format')
        self.format_menu_handles['waypoint_manager2'] = self.menu_handler.insert(
            'waypoint_manager2', parent=format_menu,
            callback=self.format_callback)
        self.format_menu_handles['waypoint_follower'] = self.menu_handler.insert(
            'waypoint_follower', parent=format_menu,
            callback=self.format_callback)
        self.update_format_menu_checks()

    def update_format_menu_checks(self):
        for yaml_format, handle in self.format_menu_handles.items():
            state = MenuHandler.CHECKED
            if yaml_format != self.yaml_format:
                state = MenuHandler.UNCHECKED
            self.menu_handler.setCheckState(handle, state)

    def rebuild_markers(self):
        self.server.clear()
        for index, waypoint in enumerate(get_waypoints(self.config)):
            x, y, z, yaw = get_xyz_yaw(waypoint)
            self.make_marker(index, x, y, z, yaw)
        self.server.applyChanges()
        self.publish_routes()

    def make_marker(self, index, x, y, z, yaw):
        marker = InteractiveMarker()
        marker.header.frame_id = self.frame_id
        marker.name = str(index)
        marker.description = f'waypoint {index}'
        marker.scale = self.get_marker_scale(index)
        marker.pose.position.x = x
        marker.pose.position.y = y
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
        move_control.markers.append(self.make_disc_marker(marker.scale))
        marker.controls.append(move_control)

        rotate_control = InteractiveMarkerControl()
        rotate_control.orientation.w = 1.0
        rotate_control.orientation.x = 0.0
        rotate_control.orientation.y = 1.0
        rotate_control.orientation.z = 0.0
        rotate_control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        rotate_control.orientation_mode = InteractiveMarkerControl.INHERIT
        rotate_control.always_visible = True
        rotate_control.markers.append(self.make_arrow_marker(marker.scale))
        marker.controls.append(rotate_control)

        menu_control = InteractiveMarkerControl()
        menu_control.interaction_mode = InteractiveMarkerControl.BUTTON
        marker.controls.append(menu_control)

        self.server.insert(marker, feedback_callback=self.feedback_callback)
        self.server.setCallback(
            marker.name, self.pose_update_callback,
            InteractiveMarkerFeedback.POSE_UPDATE)
        self.menu_handler.apply(self.server, marker.name)

    def get_marker_scale(self, index):
        waypoint = get_waypoints(self.config)[index]
        properties = waypoint.get('properties', {})
        radius = float(properties.get('goal_radius', 1.0))
        return max(radius, 0.3)

    def make_disc_marker(self, scale):
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

    def make_arrow_marker(self, scale):
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

    def feedback_callback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
            return

    def pose_update_callback(self, feedback):
        index = int(feedback.marker_name)
        waypoints = get_waypoints(self.config)
        if index >= len(waypoints):
            return

        pose = feedback.pose
        yaw = quaternion_to_yaw(pose.orientation)
        _, _, old_z, _ = get_xyz_yaw(waypoints[index])
        set_xyz_yaw(waypoints[index], pose.position.x, pose.position.y,
                    old_z, yaw)
        pose.position.z = 0.0
        self.server.setPose(feedback.marker_name, pose)
        self.server.applyChanges()

    def insert_callback(self, feedback):
        index = int(feedback.marker_name)
        waypoints = get_waypoints(self.config)
        x, y, z, yaw = get_xyz_yaw(waypoints[index])
        new_waypoint = copy.deepcopy(waypoints[index])
        set_xyz_yaw(new_waypoint, x + 0.5, y, z, yaw)
        if 'id' in new_waypoint:
            new_waypoint['id'] = index + 1
        waypoints.insert(index + 1, new_waypoint)
        self.rebuild_markers()

    def delete_callback(self, feedback):
        waypoints = get_waypoints(self.config)
        if len(waypoints) <= 1:
            self.get_logger().warn('Cannot delete the last waypoint.')
            return
        del waypoints[int(feedback.marker_name)]
        self.rebuild_markers()

    def menu_save_callback(self, feedback):
        self.save_waypoints()

    def format_callback(self, feedback):
        for yaml_format, handle in self.format_menu_handles.items():
            if feedback.menu_entry_id == handle:
                self.set_format(yaml_format)
                break
        self.update_format_menu_checks()
        self.menu_handler.reApply(self.server)
        self.rebuild_markers()

    def save_callback(self, request, response):
        try:
            self.save_waypoints()
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            return response
        response.success = True
        response.message = f'Saved: {self.yaml_path}'
        return response

    def reload_callback(self, request, response):
        try:
            self.config, self.yaml_format = load_config(self.yaml_path)
            self.update_format_menu_checks()
            self.rebuild_markers()
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            return response
        response.success = True
        response.message = f'Reloaded: {self.yaml_path}'
        return response

    def save_waypoints(self):
        save_config(self.yaml_path, self.config)
        self.get_logger().info(
            f'Saved waypoints as {self.yaml_format}: {self.yaml_path}')

    def set_format(self, yaml_format):
        yaml_format = normalize_format_name(yaml_format)
        self.config = convert_config(self.config, yaml_format)
        self.yaml_format = yaml_format

    def publish_routes(self):
        marker_array = MarkerArray()
        waypoints = get_waypoints(self.config)
        for index in range(len(waypoints) - 1):
            x1, y1, _, _ = get_xyz_yaw(waypoints[index])
            x2, y2, _, _ = get_xyz_yaw(waypoints[index + 1])

            route = Marker()
            route.header.frame_id = self.frame_id
            route.header.stamp = self.get_clock().now().to_msg()
            route.ns = 'waypoint_routes'
            route.id = index
            route.type = Marker.LINE_STRIP
            route.action = Marker.ADD
            route.scale.x = 0.04
            route.color.r = 1.0
            route.color.g = 0.9
            route.color.b = 0.0
            route.color.a = 1.0
            route.points.append(Point(x=x1, y=y1, z=0.0))
            route.points.append(Point(x=x2, y=y2, z=0.0))
            marker_array.markers.append(route)

        self.route_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)
    node = WaypointEditorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
