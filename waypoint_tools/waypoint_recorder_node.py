#!/usr/bin/env python3
"""ロボットの走行経路上に waypoint を自動生成する node.

TF ``map`` -> ``base_link`` を周期ポーリングし、基準点からの移動距離
または進行方位の変化がしきい値を超えたら現在位置に waypoint を打点する。
記録開始直後やファイル切り替え直後は waypoint を打たず、動き出して
しきい値を超えてから最初の点が置かれる。``next_file`` を呼ぶと現在位置に
waypoint を打ってから保存し、次の番号のファイルへ移る。

記録中の waypoint は RViz の interactive marker でそのままドラッグ編集・
右クリックメニュー操作でき、生成結果は waypoint_follower 形式の YAML
（``waypoints:`` リスト）で保存できる。
"""
import copy
import math
import os

import rclpy
from geometry_msgs.msg import Point
from interactive_markers import InteractiveMarkerServer, MenuHandler
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import (
    InteractiveMarkerFeedback,
    Marker,
    MarkerArray,
)

from waypoint_tools.action_sender import quaternion_to_yaw
from waypoint_tools.interactive_waypoints import build_waypoint_marker
from waypoint_tools.paths import pkg_path
from waypoint_tools.waypoint_yaml import (
    empty_config,
    get_xyz_yaw,
    make_waypoint,
    save_config,
    set_xyz_yaw,
)


DEFAULT_OUTPUT_DIR = pkg_path('config', 'waypoints', 'recorded')


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class WaypointRecorderNode(Node):
    def __init__(self):
        super().__init__('waypoint_recorder_node')

        self.declare_parameter('output_dir', DEFAULT_OUTPUT_DIR)
        self.declare_parameter('output_file_format', '{index}.yaml')
        self.declare_parameter('start_index', 0)
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('route_topic', '/waypoint_tools/routes')
        self.declare_parameter('distance_interval', 1.0)
        self.declare_parameter('yaw_interval_deg', 30.0)
        self.declare_parameter('min_move', 0.15)
        self.declare_parameter('poll_rate', 10.0)
        self.declare_parameter('goal_radius', 1.0)
        self.declare_parameter('save_on_shutdown', True)

        self.output_dir = os.path.expanduser(self.get_parameter(
            'output_dir').get_parameter_value().string_value)
        self.output_file_format = self.get_parameter(
            'output_file_format').get_parameter_value().string_value
        self.file_index = self.get_parameter(
            'start_index').get_parameter_value().integer_value
        self.map_frame = self.get_parameter(
            'map_frame').get_parameter_value().string_value
        self.robot_frame = self.get_parameter(
            'robot_frame').get_parameter_value().string_value
        route_topic = self.get_parameter(
            'route_topic').get_parameter_value().string_value
        self.distance_interval = self.get_parameter(
            'distance_interval').get_parameter_value().double_value
        self.yaw_interval = math.radians(self.get_parameter(
            'yaw_interval_deg').get_parameter_value().double_value)
        self.min_move = self.get_parameter(
            'min_move').get_parameter_value().double_value
        poll_rate = self.get_parameter(
            'poll_rate').get_parameter_value().double_value
        self.goal_radius = self.get_parameter(
            'goal_radius').get_parameter_value().double_value
        self.save_on_shutdown = self.get_parameter(
            'save_on_shutdown').get_parameter_value().bool_value

        self.config = empty_config()

        # しきい値判定の基準点（打点済みとは限らない）と直近セグメントの方位
        self.ref_xy = None
        self.last_heading = None
        self.recording = True

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.route_pub = self.create_publisher(MarkerArray, route_topic, 10)
        # 直近で publish した route セグメント数。next_file / clear などで
        # waypoint が減ったとき、余った古い marker を DELETE するのに使う。
        self._published_route_count = 0

        self.server = InteractiveMarkerServer(self, 'waypoint_tools')
        self.menu_handler = MenuHandler()
        self._init_menu()

        self.add_service = self.create_service(
            Trigger, '~/add_waypoint', self.add_waypoint_callback)
        self.save_service = self.create_service(
            Trigger, '~/save', self.save_callback)
        self.undo_service = self.create_service(
            Trigger, '~/undo', self.undo_callback)
        self.clear_service = self.create_service(
            Trigger, '~/clear', self.clear_callback)
        self.next_file_service = self.create_service(
            Trigger, '~/next_file', self.next_file_callback)
        self.pause_service = self.create_service(
            Trigger, '~/pause', self.pause_callback)
        self.resume_service = self.create_service(
            Trigger, '~/resume', self.resume_callback)

        self.poll_timer = self.create_timer(
            1.0 / max(poll_rate, 1.0), self.poll_callback)
        self.route_timer = self.create_timer(0.5, self.publish_routes)

        self.get_logger().info(
            f'Recording waypoints -> {self._current_path()}')
        self.get_logger().info(
            f'distance_interval={self.distance_interval} m, '
            f'yaw_interval={math.degrees(self.yaw_interval):.1f} deg')

    def _current_path(self):
        name = self.output_file_format.format(index=self.file_index)
        return os.path.join(self.output_dir, name)

    @property
    def waypoints(self):
        return self.config['waypoints']

    # ------------------------------------------------------------------
    # interactive marker
    # ------------------------------------------------------------------
    def _init_menu(self):
        self.menu_handler.insert(
            'insert after', callback=self._menu_insert_after)
        self.menu_handler.insert('delete', callback=self._menu_delete)
        self.menu_handler.insert('save', callback=self._menu_save)
        self.menu_handler.insert(
            'save & next file', callback=self._menu_next_file)
        self._rec_handle = self.menu_handler.insert(
            'recording', callback=self._menu_toggle_recording)
        self.menu_handler.setCheckState(
            self._rec_handle, MenuHandler.CHECKED)

    def _marker_scale(self):
        return max(self.goal_radius, 0.3)

    def _insert_marker(self, index):
        x, y, _, yaw = get_xyz_yaw(self.waypoints[index])
        marker = build_waypoint_marker(
            str(index), self.map_frame, x, y, yaw,
            self._marker_scale(), f'wp {index}')
        self.server.insert(marker, feedback_callback=self._feedback_callback)
        self.server.setCallback(
            marker.name, self.pose_update_callback,
            InteractiveMarkerFeedback.POSE_UPDATE)
        self.menu_handler.apply(self.server, marker.name)

    def rebuild_markers(self):
        self.server.clear()
        for index in range(len(self.waypoints)):
            self._insert_marker(index)
        self.server.applyChanges()
        self.publish_routes()

    def _feedback_callback(self, feedback):
        return

    def pose_update_callback(self, feedback):
        try:
            index = int(feedback.marker_name)
        except ValueError:
            return
        waypoints = self.waypoints
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
        if index == len(waypoints) - 1:
            self.ref_xy = (pose.position.x, pose.position.y)
            self.rebuild_headings()
        self.publish_routes()

    def _menu_insert_after(self, feedback):
        index = int(feedback.marker_name)
        waypoints = self.waypoints
        x, y, z, yaw = get_xyz_yaw(waypoints[index])
        new_waypoint = copy.deepcopy(waypoints[index])
        set_xyz_yaw(new_waypoint, x + 0.5, y, z, yaw)
        waypoints.insert(index + 1, new_waypoint)
        self.rebuild_headings()
        self.rebuild_markers()

    def _menu_delete(self, feedback):
        index = int(feedback.marker_name)
        waypoints = self.waypoints
        if index >= len(waypoints):
            return
        del waypoints[index]
        if waypoints:
            last_x, last_y, _, _ = get_xyz_yaw(waypoints[-1])
            self.ref_xy = (last_x, last_y)
        else:
            self.ref_xy = None
        self.rebuild_headings()
        self.rebuild_markers()

    def _menu_save(self, feedback):
        try:
            self.save_waypoints()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Save failed: {exc}')

    def _menu_next_file(self, feedback):
        try:
            self._advance_file()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'next_file failed: {exc}')

    def _advance_file(self):
        """現在位置に waypoint を打ってから保存し、次の番号へ切り替える."""
        transition_xy = self._place_final_waypoint()
        if transition_xy is None:
            raise RuntimeError('TF unavailable; cannot place final waypoint.')

        saved_path = self._current_path()
        self.save_waypoints()
        self.file_index += 1
        self.config = empty_config()
        # 次ファイルはこの遷移点を基準にする。切り替え直後は打点しない。
        self.ref_xy = transition_xy
        self.last_heading = None
        self.rebuild_markers()
        self.get_logger().info(
            f'Finished {saved_path}. Now recording -> {self._current_path()}')
        return saved_path

    def next_file_callback(self, request, response):
        try:
            saved_path = self._advance_file()
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = str(exc)
            return response
        response.success = True
        response.message = (
            f'Saved {saved_path}. Now recording -> {self._current_path()}')
        return response

    def _menu_toggle_recording(self, feedback):
        self._set_recording(not self.recording)

    def _set_recording(self, value):
        self.recording = value
        state = MenuHandler.CHECKED if value else MenuHandler.UNCHECKED
        self.menu_handler.setCheckState(self._rec_handle, state)
        self.menu_handler.reApply(self.server)
        self.server.applyChanges()
        self.get_logger().info(
            f'Recording {"resumed" if value else "paused"}.')

    # ------------------------------------------------------------------
    # ポーリング
    # ------------------------------------------------------------------
    def lookup_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.robot_frame, Time())
        except TransformException as exc:
            self.get_logger().warn(
                f'TF {self.map_frame}->{self.robot_frame} unavailable: {exc}',
                throttle_duration_sec=5.0)
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return t.x, t.y, yaw

    def poll_callback(self):
        if not self.recording:
            return
        pose = self.lookup_pose()
        if pose is None:
            return
        x, y, _ = pose

        if self.ref_xy is None:
            # 基準点を覚えるだけ。記録開始直後は waypoint を打たない。
            self.ref_xy = (x, y)
            return

        dx = x - self.ref_xy[0]
        dy = y - self.ref_xy[1]
        distance = math.hypot(dx, dy)
        if distance < self.min_move:
            return

        heading = math.atan2(dy, dx)
        heading_changed = (
            self.last_heading is not None
            and abs(normalize_angle(heading - self.last_heading))
            >= self.yaw_interval
        )

        if distance >= self.distance_interval or heading_changed:
            self._place_waypoint(x, y, heading)
            self.last_heading = heading
            self.get_logger().info(
                f'Recorded waypoint {len(self.waypoints) - 1} at '
                f'({x:.2f}, {y:.2f}), yaw={math.degrees(heading):.1f} deg '
                f'[total {len(self.waypoints)}]')

    # ------------------------------------------------------------------
    # 打点処理
    # ------------------------------------------------------------------
    def _place_waypoint(self, x, y, yaw):
        self.waypoints.append(make_waypoint(x, y, 0.0, yaw))
        self._insert_marker(len(self.waypoints) - 1)
        self.server.applyChanges()
        self.ref_xy = (x, y)
        self.publish_routes()

    def _heading_from_ref(self, x, y, tf_yaw):
        """基準点から現在位置への進行方位。動いていなければ TF の yaw."""
        if self.ref_xy is not None and math.hypot(
                x - self.ref_xy[0], y - self.ref_xy[1]) >= 1e-3:
            return math.atan2(y - self.ref_xy[1], x - self.ref_xy[0])
        return tf_yaw

    def _place_final_waypoint(self):
        """現在位置に終端 waypoint を打つ。

        既存の最終点と ``min_move`` 未満しか離れていなければ重複を避けて
        打点しない。戻り値は現在位置 (x, y)、TF が無ければ None.
        """
        pose = self.lookup_pose()
        if pose is None:
            return None
        x, y, tf_yaw = pose
        near_last = False
        if self.waypoints:
            lx, ly, _, _ = get_xyz_yaw(self.waypoints[-1])
            near_last = math.hypot(x - lx, y - ly) < self.min_move
        if not near_last:
            self._place_waypoint(x, y, self._heading_from_ref(x, y, tf_yaw))
        return (x, y)

    def add_waypoint_callback(self, request, response):
        pose = self.lookup_pose()
        if pose is None:
            response.success = False
            response.message = 'TF unavailable.'
            return response
        x, y, tf_yaw = pose
        heading = self._heading_from_ref(x, y, tf_yaw)
        self._place_waypoint(x, y, heading)
        self.last_heading = heading
        response.success = True
        response.message = (
            f'Added waypoint {len(self.waypoints) - 1} '
            f'({x:.2f}, {y:.2f}).')
        return response

    def save_callback(self, request, response):
        try:
            self.save_waypoints()
        except Exception as exc:  # noqa: BLE001
            response.success = False
            response.message = str(exc)
            return response
        response.success = True
        response.message = (
            f'Saved {len(self.waypoints)} waypoints: {self._current_path()}')
        return response

    def undo_callback(self, request, response):
        if not self.waypoints:
            response.success = False
            response.message = 'No waypoints to undo.'
            return response
        self.waypoints.pop()
        if self.waypoints:
            last_x, last_y, _, _ = get_xyz_yaw(self.waypoints[-1])
            self.ref_xy = (last_x, last_y)
        else:
            self.ref_xy = None
        self.last_heading = None
        self.rebuild_headings()
        self.rebuild_markers()
        response.success = True
        response.message = (
            f'Removed last waypoint. {len(self.waypoints)} left.')
        return response

    def clear_callback(self, request, response):
        self.config = empty_config()
        self.ref_xy = None
        self.last_heading = None
        self.rebuild_markers()
        response.success = True
        response.message = 'Cleared all waypoints.'
        return response

    def pause_callback(self, request, response):
        self._set_recording(False)
        response.success = True
        response.message = 'Recording paused.'
        return response

    def resume_callback(self, request, response):
        self._set_recording(True)
        response.success = True
        response.message = 'Recording resumed.'
        return response

    def rebuild_headings(self):
        """undo 後に last_heading を直近セグメントから復元する."""
        if len(self.waypoints) < 2:
            self.last_heading = None
            return
        x1, y1, _, _ = get_xyz_yaw(self.waypoints[-2])
        x2, y2, _, _ = get_xyz_yaw(self.waypoints[-1])
        self.last_heading = math.atan2(y2 - y1, x2 - x1)

    def save_waypoints(self):
        path = self._current_path()
        directory = os.path.dirname(path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        save_config(path, self.config)
        self.get_logger().info(
            f'Saved {len(self.waypoints)} waypoints: {path}')

    # ------------------------------------------------------------------
    # 可視化
    # ------------------------------------------------------------------
    def publish_routes(self):
        marker_array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        waypoints = self.waypoints

        for index in range(len(waypoints) - 1):
            x1, y1, _, _ = get_xyz_yaw(waypoints[index])
            x2, y2, _, _ = get_xyz_yaw(waypoints[index + 1])
            route = Marker()
            route.header.frame_id = self.map_frame
            route.header.stamp = stamp
            route.ns = 'waypoint_routes'
            route.id = index
            route.type = Marker.LINE_STRIP
            route.action = Marker.ADD
            route.scale.x = 0.05
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
            stale.header.frame_id = self.map_frame
            stale.header.stamp = stamp
            stale.ns = 'waypoint_routes'
            stale.id = index
            stale.action = Marker.DELETE
            marker_array.markers.append(stale)
        self._published_route_count = segment_count

        self.route_pub.publish(marker_array)

    def on_shutdown(self):
        if not self.save_on_shutdown:
            return
        # next_file と同様、最後のルートも停止位置で終端させる。
        try:
            self._place_final_waypoint()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Final waypoint not placed: {exc}')
        if self.waypoints:
            try:
                self.save_waypoints()
            except Exception as exc:  # noqa: BLE001
                self.get_logger().error(f'Save on shutdown failed: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.on_shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
