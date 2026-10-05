#!/usr/bin/env python3
import math
import os

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Point
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import SetBool, Trigger
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from waypoint_tools.action_sender import (
    make_navigate_to_pose_goal, yaw_to_quaternion)
from waypoint_tools.node_params import (
    BOOL, DOUBLE, INTEGER, STRING, require_parameters)
from waypoint_tools.waypoint_edit_markers import WaypointEditMarkers
from waypoint_tools.waypoint_yaml import (
    get_waypoints, get_xyz_yaw, is_stop, load_config, save_config)


# Nav2 の NavigateToPose action。経由点の管理はこの node が行う。
ACTION_NAME = '/navigate_to_pose'

# 送信する waypoint の表示 topic。Nav2 の RViz 設定
# （orne_box_navigation_executor/config/rviz/nav2_TC2024_view2.rviz）が表示する。
MARKER_TOPIC = '/waypoints'

# 円盤の色 (r, g, b, a)。停止点は赤、現在の目標点は緑。
DISC_COLOR = (0.1, 0.7, 1.0, 0.6)
STOP_DISC_COLOR = (1.0, 0.1, 0.1, 0.7)
TARGET_DISC_COLOR = (0.1, 0.9, 0.2, 0.8)

# 状態
IDLE = 'idle'            # 未送信（send_all / next_wp で走行開始）
RUNNING = 'running'      # 走行中
STOPPED = 'stopped'      # stop: true の点に到達し、next_wp 待ち
FINISHED = 'finished'    # 最後の点に到達
FAILED = 'failed'        # 再送しても到達できず停止
PAUSED = 'paused'        # 編集モードに入って goal を取り消した。next_wp で同じ点から再開


class WaypointSenderNode(Node):
    def __init__(self):
        super().__init__('waypoint_sender_node')

        params = require_parameters(self, {
            'yaml_path': STRING,
            'frame_id': STRING,
            'robot_frame': STRING,
            'switch_radius': DOUBLE,
            'max_retries': INTEGER,
            'skip_on_failure': BOOL,
        })

        self.yaml_path = os.path.expanduser(params['yaml_path'])
        if not os.path.isfile(self.yaml_path):
            raise RuntimeError(f'yaml_path must be a file: {self.yaml_path}')
        self.waypoints = []
        self.wp_index = 0
        self.state = IDLE

        self.frame_id = params['frame_id']
        self.robot_frame = params['robot_frame']
        self.switch_radius = params['switch_radius']
        self.max_retries = params['max_retries']
        self.skip_on_failure = params['skip_on_failure']

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.action_client = ActionClient(self, NavigateToPose, ACTION_NAME)
        # goal を送るたびに増やす。古い（先行 goal に置き換えられた）goal の
        # 結果は世代が合わないので無視する。
        self.goal_generation = 0
        self.goal_handle = None
        self.retries = 0

        self.create_service(Trigger, '~/send_all', self.send_all_callback)
        self.create_service(Trigger, '~/next_wp', self.next_wp_callback)
        self.create_service(SetBool, '~/edit', self.edit_callback)

        self.marker_pub = self.create_publisher(MarkerArray, MARKER_TOPIC, 10)
        # 直近で publish した waypoint 数。reload で減ったときに古い marker を消す。
        self._published_count = 0

        # 編集モード中だけ円盤を interactive marker にして編集できるようにする。
        self.editing = False
        self.edit_markers = WaypointEditMarkers(
            self, self.frame_id, lambda: self.waypoints, self.save_waypoints,
            on_insert=self._on_insert, on_delete=self._on_delete,
            on_change=self.publish_markers)

        # 起動時は YAML を読んで RViz に表示するだけ。走行は ~/send_all
        # （または ~/next_wp）で開始する。
        self._load()
        self.get_logger().info(
            'Waiting for ~/send_all (or ~/next_wp) to start navigation.')

        self.control_timer = self.create_timer(0.1, self.control_callback)
        # RViz の後起動でも表示されるよう定期的に publish する。
        self.marker_timer = self.create_timer(1.0, self.publish_markers)

    def _is_hold_point(self, index):
        """到達（Nav2 の SUCCEEDED）まで待つ点か。stop 指定の点と最後の点."""
        return (is_stop(self.waypoints[index])
                or index == len(self.waypoints) - 1)

    # -----------------------------------------------------------
    # 読み込みと送信開始
    # -----------------------------------------------------------
    def _load(self):
        """YAML を読み込んで RViz に表示する（走行はしない）."""
        config = load_config(self.yaml_path)
        waypoints = get_waypoints(config)
        if not waypoints:
            raise RuntimeError(f'No waypoints in {self.yaml_path}.')

        self.config = config
        self.waypoints = waypoints
        self.wp_index = 0
        stops = [i for i, wp in enumerate(waypoints) if is_stop(wp)]
        self.get_logger().info(
            f'Loaded {len(waypoints)} waypoints from {self.yaml_path} '
            f'(stop: {stops}).')
        self.publish_markers()

    def send_all(self):
        """最初の waypoint から走行を始める（編集後の waypoint を使う）."""
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f'{ACTION_NAME} is not available.')
        self._go_to(0)

    # -----------------------------------------------------------
    # 走行制御
    # -----------------------------------------------------------
    def _go_to(self, index):
        self.wp_index = index
        self.retries = 0
        self._send_current()
        self.publish_markers()

    def _send_current(self):
        self.goal_generation += 1
        generation = self.goal_generation
        goal = make_navigate_to_pose_goal(
            self.waypoints[self.wp_index], self.frame_id,
            self.get_clock().now().to_msg())
        self.state = RUNNING
        self.goal_handle = None
        hold = ' (stop)' if is_stop(self.waypoints[self.wp_index]) else ''
        self.get_logger().info(
            f'Waypoint {self.wp_index}/{len(self.waypoints) - 1}{hold}')
        future = self.action_client.send_goal_async(goal)
        future.add_done_callback(
            lambda f: self._goal_response_callback(f, generation))

    def _advance(self):
        if self.wp_index + 1 >= len(self.waypoints):
            self._finish()
            return
        self._go_to(self.wp_index + 1)

    def _finish(self):
        self.state = FINISHED
        self.get_logger().info('Reached the last waypoint.')

    def _cancel_current(self):
        # 世代を進めて、キャンセルした goal の結果を無視させる。
        self.goal_generation += 1
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()
            self.goal_handle = None

    def _goal_response_callback(self, future, generation):
        goal_handle = future.result()
        if generation != self.goal_generation:
            # 応答待ちの間に取り消された goal。受理されていたら止める。
            if goal_handle.accepted:
                goal_handle.cancel_goal_async()
            return
        if not goal_handle.accepted:
            self.get_logger().error(
                f'Goal for waypoint {self.wp_index} rejected.')
            self._on_failure()
            return
        self.goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            lambda f: self._result_callback(f, generation))

    def _result_callback(self, future, generation):
        if generation != self.goal_generation:
            return
        self.goal_handle = None
        status = future.result().status
        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().warn(
                f'Waypoint {self.wp_index} failed (status {status}).')
            self._on_failure()
            return

        waypoint = self.waypoints[self.wp_index]
        if self.wp_index == len(self.waypoints) - 1:
            self._finish()
        elif is_stop(waypoint):
            self.state = STOPPED
            self.get_logger().info(
                f'Stopped at waypoint {self.wp_index}. '
                'Call ~/next_wp to continue.')
        else:
            # switch_radius に入る前に Nav2 が到達判定した場合。
            self._advance()

    def _on_failure(self):
        if self.retries < self.max_retries:
            self.retries += 1
            self.get_logger().warn(
                f'Retry waypoint {self.wp_index} '
                f'({self.retries}/{self.max_retries}).')
            self._send_current()
        elif self.skip_on_failure:
            self.get_logger().warn(f'Skip waypoint {self.wp_index}.')
            self._advance()
        else:
            self.state = FAILED
            self.get_logger().error(
                f'Gave up waypoint {self.wp_index}. '
                'Call ~/next_wp to go to the next waypoint.')

    def control_callback(self):
        """通過点では switch_radius に入った時点で次の点を送る（停止しない）."""
        if self.state != RUNNING or self._is_hold_point(self.wp_index):
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame_id, self.robot_frame, Time())
        except TransformException:
            return
        x, y, _, _ = get_xyz_yaw(self.waypoints[self.wp_index])
        dist = math.hypot(x - tf.transform.translation.x,
                          y - tf.transform.translation.y)
        if dist <= self.switch_radius:
            self._advance()

    # -----------------------------------------------------------
    # RViz 表示
    # -----------------------------------------------------------
    def _new_marker(self, ns, index, marker_type):
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = ns
        marker.id = index
        marker.type = marker_type
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _target_index(self):
        """緑で表示する現在の目標点。走行していなければ None."""
        if self.state in (RUNNING, STOPPED, FAILED, PAUSED):
            return self.wp_index
        return None

    def publish_markers(self):
        """waypoint を円盤 + 向きの矢印 + 番号で、経路を線で表示する."""
        marker_array = MarkerArray()
        # 編集モード中は円盤・矢印・番号を interactive marker が出す。
        shown = [] if self.editing else self.waypoints
        for index, waypoint in enumerate(shown):
            x, y, _, yaw = get_xyz_yaw(waypoint)
            stop = is_stop(waypoint)

            disc = self._new_marker('waypoints', index, Marker.CYLINDER)
            disc.pose.position.x = x
            disc.pose.position.y = y
            disc.scale.x = disc.scale.y = 1.0
            disc.scale.z = 0.04
            if index == self._target_index():
                color = TARGET_DISC_COLOR
            elif stop:
                color = STOP_DISC_COLOR
            else:
                color = DISC_COLOR
            disc.color.r, disc.color.g, disc.color.b, disc.color.a = color
            marker_array.markers.append(disc)

            arrow = self._new_marker('waypoint_arrows', index, Marker.ARROW)
            arrow.pose.position.x = x
            arrow.pose.position.y = y
            (arrow.pose.orientation.x, arrow.pose.orientation.y,
             arrow.pose.orientation.z, arrow.pose.orientation.w) = \
                yaw_to_quaternion(yaw)
            arrow.scale.x = 0.6
            arrow.scale.y = arrow.scale.z = 0.08
            arrow.color.r, arrow.color.g, arrow.color.b = 1.0, 0.2, 0.1
            arrow.color.a = 1.0
            marker_array.markers.append(arrow)

            text = self._new_marker(
                'waypoint_labels', index, Marker.TEXT_VIEW_FACING)
            text.pose.position.x = x
            text.pose.position.y = y
            text.pose.position.z = 0.6
            text.scale.z = 0.5
            text.color.r = text.color.g = text.color.b = 0.0
            text.color.a = 1.0
            text.text = f'{index}' + (' (stop)' if stop else '')
            marker_array.markers.append(text)

        route = self._new_marker('waypoint_route', 0, Marker.LINE_STRIP)
        route.scale.x = 0.04
        route.color.r, route.color.g, route.color.b = 1.0, 0.9, 0.0
        route.color.a = 1.0
        for waypoint in self.waypoints:
            x, y, _, _ = get_xyz_yaw(waypoint)
            route.points.append(Point(x=x, y=y, z=0.0))
        if len(route.points) >= 2:
            marker_array.markers.append(route)

        for index in range(len(shown), self._published_count):
            for ns in ('waypoints', 'waypoint_arrows', 'waypoint_labels'):
                stale = self._new_marker(ns, index, Marker.CYLINDER)
                stale.action = Marker.DELETE
                marker_array.markers.append(stale)
        self._published_count = len(shown)

        self.marker_pub.publish(marker_array)

    # -----------------------------------------------------------
    # 編集モード
    # -----------------------------------------------------------
    def set_editing(self, enabled):
        if enabled == self.editing:
            return True, f'Edit mode is already {"on" if enabled else "off"}.'
        self.editing = enabled
        if enabled:
            if self.state == RUNNING:
                self._cancel_current()
                self.state = PAUSED
                self.get_logger().info(
                    f'Canceled the goal to waypoint {self.wp_index} for editing. '
                    'Call ~/next_wp to resume.')
            self.edit_markers.show()
        else:
            self.edit_markers.hide()
        self.publish_markers()
        return True, f'Edit mode {"on" if enabled else "off"}.'

    def _on_insert(self, index):
        # 目標点より前に挿入したら、目標点の番号をずらす。
        if self.state != IDLE and index <= self.wp_index:
            self.wp_index += 1

    def _on_delete(self, index):
        if self.state != IDLE and index < self.wp_index:
            self.wp_index -= 1
        self.wp_index = min(self.wp_index, len(self.waypoints) - 1)

    def save_waypoints(self):
        save_config(self.yaml_path, self.config)
        self.get_logger().info(f'Saved waypoints: {self.yaml_path}')

    # -----------------------------------------------------------
    # サービス
    # -----------------------------------------------------------
    @staticmethod
    def _respond(response, result):
        response.success, response.message = result
        return response

    def _try_send_all(self):
        try:
            self.send_all()
        except Exception as exc:  # noqa: BLE001
            return False, str(exc)
        return True, f'Sent {self.yaml_path}'

    def start(self):
        """最初の waypoint から走行を開始する（走行中ならやり直す）."""
        self._cancel_current()
        return self._try_send_all()

    def next_wp(self):
        """次の waypoint へ進む.

        未送信 -> 走行開始 / 走行中・停止点・失敗 -> 現在の点をやめて次の点へ /
        編集で一時停止 -> 向かっていた点から再開。
        """
        if self.state == IDLE:
            return self._try_send_all()
        if self.state == PAUSED:
            self._go_to(self.wp_index)
            return True, f'Resumed to waypoint {self.wp_index}.'
        if self.state == FINISHED:
            return False, 'Already reached the last waypoint.'
        self._cancel_current()
        self._advance()
        if self.state == FINISHED:
            return True, 'Reached the last waypoint.'
        return True, f'Go to waypoint {self.wp_index}.'

    def send_all_callback(self, request, response):
        return self._respond(response, self.start())

    def next_wp_callback(self, request, response):
        return self._respond(response, self.next_wp())

    def edit_callback(self, request, response):
        return self._respond(response, self.set_editing(request.data))


def main(args=None):
    rclpy.init(args=args)
    node = WaypointSenderNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
