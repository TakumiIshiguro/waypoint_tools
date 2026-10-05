#!/usr/bin/env python3
import os

from rcl_interfaces.msg import SetParametersResult

import rclpy
from geometry_msgs.msg import Point
from rclpy.node import Node
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray

from waypoint_tools.interactive_waypoints import ROUTE_TOPIC
from waypoint_tools.node_params import STRING, require_parameters
from waypoint_tools.waypoint_edit_markers import WaypointEditMarkers
from waypoint_tools.waypoint_yaml import (
    get_waypoints, get_xyz_yaw, load_config, save_config)


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

        self.edit_markers = WaypointEditMarkers(
            self, self.frame_id, lambda: get_waypoints(self.config),
            self.save_waypoints, on_change=self.publish_routes)

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

    def rebuild_markers(self):
        self.edit_markers.show()
        self.publish_routes()

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
