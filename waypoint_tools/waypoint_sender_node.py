#!/usr/bin/env python3
import os

import rclpy
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

from waypoint_tools.action_sender import make_follow_waypoints_goal
from waypoint_tools.paths import pkg_path
from waypoint_tools.waypoint_yaml import list_waypoint_yamls, load_config


DEFAULT_WAYPOINT_YAML_PATH = pkg_path('config', 'waypoints', 'sample.yaml')


class WaypointSenderNode(Node):
    def __init__(self):
        super().__init__('waypoint_sender_node')

        # yaml_path はファイル/フォルダどちらでも可（editor と同じ）。
        #   ファイル -> その 1 ファイルを送る
        #   フォルダ -> 中の *.yaml を数値順に並べ、~/next_file /~/prev_file で送る
        self.declare_parameter('yaml_path', DEFAULT_WAYPOINT_YAML_PATH)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('action_name', '/follow_waypoints')
        self.declare_parameter('send_on_start', True)

        target = os.path.expanduser(self.get_parameter(
            'yaml_path').get_parameter_value().string_value)

        self.yaml_files = self._resolve_targets(target)
        self.file_index = 0

        self.frame_id = self.get_parameter(
            'frame_id').get_parameter_value().string_value
        self.action_name = self.get_parameter(
            'action_name').get_parameter_value().string_value
        self.send_on_start = self.get_parameter(
            'send_on_start').get_parameter_value().bool_value

        self.action_client = ActionClient(self, FollowWaypoints,
                                          self.action_name)
        self.send_service = self.create_service(
            Trigger, '~/send_all', self.send_all_callback)
        self.next_service = self.create_service(
            Trigger, '~/next_file', self.next_file_callback)
        self.prev_service = self.create_service(
            Trigger, '~/prev_file', self.prev_file_callback)
        self.goal_handle = None
        self.goal_running = False
        self.sent_on_start = False

        if self.send_on_start:
            self.start_timer = self.create_timer(0.5, self.send_once)

    # -----------------------------------------------------------
    # 対象（ファイル or フォルダ）の解決
    # -----------------------------------------------------------
    def _resolve_targets(self, target):
        if os.path.isdir(target):
            files = list_waypoint_yamls(target)
            if not files:
                raise RuntimeError(f'No yaml files in directory: {target}')
            self.get_logger().info(
                f'Folder mode: {len(files)} yaml files in {target}')
            return files
        self.get_logger().info(f'File mode: {target}')
        return [target]

    @property
    def current_path(self):
        return self.yaml_files[self.file_index]

    def send_once(self):
        if self.sent_on_start:
            return
        try:
            self.send_all()
            self.sent_on_start = True
            self.start_timer.cancel()
        except Exception as exc:
            self.get_logger().error(str(exc))

    def send_all_callback(self, request, response):
        try:
            self.send_all()
        except Exception as exc:
            response.success = False
            response.message = str(exc)
            return response

        response.success = True
        response.message = 'Sent all waypoints.'
        return response

    # -----------------------------------------------------------
    # ファイル送り（フォルダ / リストモード共通）
    # -----------------------------------------------------------
    def _step_file(self, delta):
        """delta だけファイルを進めて送信する。戻り値は (成功, メッセージ)."""
        if len(self.yaml_files) <= 1:
            return False, 'Only one waypoint YAML is loaded.'
        if self.goal_running:
            return False, 'Current FollowWaypoints goal is still running.'
        new_index = self.file_index + delta
        if new_index < 0 or new_index >= len(self.yaml_files):
            edge = 'first' if new_index < 0 else 'last'
            return False, (
                f'Already at the {edge} file '
                f'({self.file_index + 1}/{len(self.yaml_files)}).')
        self.file_index = new_index
        try:
            self.send_all()
        except Exception as exc:  # noqa: BLE001
            return False, str(exc)
        return True, (
            f'Sent [{self.file_index + 1}/{len(self.yaml_files)}]: '
            f'{os.path.basename(self.current_path)}')

    def next_file_callback(self, request, response):
        response.success, response.message = self._step_file(1)
        return response

    def prev_file_callback(self, request, response):
        response.success, response.message = self._step_file(-1)
        return response

    def send_all(self):
        yaml_path = self.current_path
        config = load_config(yaml_path)
        goal = make_follow_waypoints_goal(
            config, self.frame_id, self.get_clock().now().to_msg())

        if not goal.poses:
            raise RuntimeError('No waypoints to send.')

        self.get_logger().info(
            f'Loaded {len(goal.poses)} waypoints from '
            f'{yaml_path} [{self.file_index + 1}/'
            f'{len(self.yaml_files)}].')
        self.get_logger().info(f'Waiting for {self.action_name}...')
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f'{self.action_name} is not available.')

        future = self.action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.response_callback)
        self.goal_running = True
        self.get_logger().info('FollowWaypoints goal sent.')

    def feedback_callback(self, feedback_msg):
        current = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f'Current waypoint: {current}')

    def response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('FollowWaypoints goal rejected.')
            self.goal_running = False
            return

        self.get_logger().info('FollowWaypoints goal accepted.')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        status = future.result().status
        self.goal_running = False
        if result.missed_waypoints:
            self.get_logger().warn(
                f'Missed waypoints: {result.missed_waypoints}')
        self.get_logger().info(f'FollowWaypoints finished: {status}')
        if self.file_index + 1 < len(self.yaml_files):
            self.get_logger().info(
                'Last waypoint in current YAML reached. Call '
                '/waypoint_sender_node/next_file to send the next YAML.')
        else:
            self.get_logger().info('All waypoint YAML files have been sent.')


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
