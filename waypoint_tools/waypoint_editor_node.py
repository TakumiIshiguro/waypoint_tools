#!/usr/bin/env python3
import copy
import os

from rcl_interfaces.msg import SetParametersResult

import rclpy
from geometry_msgs.msg import Point
from interactive_markers import InteractiveMarkerServer, MenuHandler
from rclpy.node import Node
from std_srvs.srv import Trigger
from visualization_msgs.msg import (
    InteractiveMarkerFeedback,
    Marker,
    MarkerArray,
)

from waypoint_tools.action_sender import quaternion_to_yaw
from waypoint_tools.interactive_waypoints import (
    DISC_COLOR, ROUTE_TOPIC, STOP_DISC_COLOR, build_waypoint_marker)
from waypoint_tools.node_params import STRING, require_parameters
from waypoint_tools.waypoint_yaml import (
    get_waypoints,
    get_xyz_yaw,
    is_stop,
    load_config,
    save_config,
    set_xyz_yaw,
)


class WaypointEditorNode(Node):
    def __init__(self):
        super().__init__('waypoint_editor_node')

        params = require_parameters(self, {
            'yaml_path': STRING,
            'frame_id': STRING,
        })

        self.frame_id = params['frame_id']
        self.yaml_path = self._check_file(os.path.expanduser(params['yaml_path']))

        self.config = load_config(self.yaml_path)

        self.server = InteractiveMarkerServer(self, 'waypoint_tools')
        self.menu_handler = MenuHandler()

        self.route_pub = self.create_publisher(MarkerArray, ROUTE_TOPIC, 10)
        # 直近で publish した route セグメント数。delete や reload で
        # waypoint が減ったとき、余った古い marker を DELETE するのに使う。
        self._published_route_count = 0
        self.save_service = self.create_service(
            Trigger, '~/save', self.save_callback)
        self.reload_service = self.create_service(
            Trigger, '~/reload', self.reload_callback)

        # yaml_path パラメータの動的変更を監視
        self.add_on_set_parameters_callback(self.on_params_changed)
        # -----------------------------------------------------

        self.init_menu()
        self.rebuild_markers()
        self.timer = self.create_timer(0.5, self.publish_routes)

        self.get_logger().info(f'Loaded waypoints: {self.yaml_path}')

    @staticmethod
    def _check_file(path):
        if not os.path.isfile(path):
            raise RuntimeError(f'yaml_path must be a file: {path}')
        return path

    # -----------------------------------------------------------
    # パラメータ動的変更コールバック（yaml_path の変更に対応）
    # -----------------------------------------------------------
    def on_params_changed(self, params):
        for p in params:
            if p.name == 'yaml_path' and p.value:
                try:
                    self._load_file(
                        self._check_file(os.path.expanduser(p.value)))
                except Exception as exc:  # noqa: BLE001
                    return SetParametersResult(
                        successful=False, reason=str(exc))
        return SetParametersResult(successful=True)

    def _load_file(self, path):
        config = load_config(path)
        self.yaml_path = path
        self.config = config
        self.rebuild_markers()
        self.get_logger().info(f'Loaded: {self.yaml_path}')

    def init_menu(self):
        self.menu_handler.insert('insert after', callback=self.insert_callback)
        self.menu_handler.insert('delete', callback=self.delete_callback)
        self.menu_handler.insert('save', callback=self.menu_save_callback)
        # チェックで停止点（stop: true）。状態は marker ごとに make_marker で反映する。
        self.stop_handle = self.menu_handler.insert(
            'stop', callback=self.stop_callback)

    def rebuild_markers(self):
        self.server.clear()
        for index, waypoint in enumerate(get_waypoints(self.config)):
            x, y, z, yaw = get_xyz_yaw(waypoint)
            self.make_marker(index, x, y, z, yaw)
        self.server.applyChanges()
        self.publish_routes()

    def make_marker(self, index, x, y, z, yaw):
        stop = is_stop(get_waypoints(self.config)[index])
        marker = build_waypoint_marker(
            str(index), self.frame_id, x, y, yaw,
            self.get_marker_scale(index),
            f'waypoint {index}' + (' (stop)' if stop else ''),
            STOP_DISC_COLOR if stop else DISC_COLOR)
        self.server.insert(marker, feedback_callback=self.feedback_callback)
        self.server.setCallback(
            marker.name, self.pose_update_callback,
            InteractiveMarkerFeedback.POSE_UPDATE)
        # MenuHandler のチェック状態は apply 時点の値が marker に焼き込まれる。
        self.menu_handler.setCheckState(
            self.stop_handle,
            MenuHandler.CHECKED if stop else MenuHandler.UNCHECKED)
        self.menu_handler.apply(self.server, marker.name)

    def stop_callback(self, feedback):
        index = int(feedback.marker_name)
        waypoint = get_waypoints(self.config)[index]
        if is_stop(waypoint):
            waypoint.pop('stop', None)
        else:
            waypoint['stop'] = True
        x, y, z, yaw = get_xyz_yaw(waypoint)
        self.make_marker(index, x, y, z, yaw)
        self.server.applyChanges()
        self.get_logger().info(
            f'waypoint {index}: stop={is_stop(waypoint)} (save to keep)')

    def get_marker_scale(self, index):
        waypoint = get_waypoints(self.config)[index]
        properties = waypoint.get('properties', {})
        radius = float(properties.get('goal_radius', 1.0))
        return max(radius, 0.3)

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
        new_waypoint.pop('stop', None)
        set_xyz_yaw(new_waypoint, x + 0.5, y, z, yaw)
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
            self.config = load_config(self.yaml_path)
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
        self.get_logger().info(f'Saved waypoints: {self.yaml_path}')

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

        segment_count = max(len(waypoints) - 1, 0)
        for index in range(segment_count, self._published_route_count):
            stale = Marker()
            stale.header.frame_id = self.frame_id
            stale.ns = 'waypoint_routes'
            stale.id = index
            stale.action = Marker.DELETE
            marker_array.markers.append(stale)
        self._published_route_count = segment_count

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
