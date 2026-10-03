#!/usr/bin/env python3
import math
import os

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from waypoint_tools.action_sender import make_navigate_to_pose_goal
from waypoint_tools.node_params import (
    BOOL, DOUBLE, INTEGER, STRING, require_parameters)
from waypoint_tools.waypoint_yaml import (
    get_waypoints, get_xyz_yaw, is_stop, load_config)


# Nav2 の NavigateToPose action。経由点の管理はこの node が行う。
ACTION_NAME = '/navigate_to_pose'

# 状態
IDLE = 'idle'            # 未送信
RUNNING = 'running'      # 走行中
STOPPED = 'stopped'      # stop: true の点に到達し、next_wp 待ち
FINISHED = 'finished'    # 最後の点に到達
FAILED = 'failed'        # 再送しても到達できず停止


class WaypointSenderNode(Node):
    def __init__(self):
        super().__init__('waypoint_sender_node')

        params = require_parameters(self, {
            'yaml_path': STRING,
            'frame_id': STRING,
            'robot_frame': STRING,
            'send_on_start': BOOL,
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
        self.send_on_start = params['send_on_start']
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
        self.create_service(Trigger, '~/skip', self.skip_callback)

        self.control_timer = self.create_timer(0.1, self.control_callback)
        self.sent_on_start = False
        if self.send_on_start:
            self.start_timer = self.create_timer(0.5, self.send_once)

    def _is_hold_point(self, index):
        """到達（Nav2 の SUCCEEDED）まで待つ点か。stop 指定の点と最後の点."""
        return (is_stop(self.waypoints[index])
                or index == len(self.waypoints) - 1)

    # -----------------------------------------------------------
    # 起動時の送信
    # -----------------------------------------------------------
    def send_once(self):
        if self.sent_on_start:
            return
        try:
            self.send_all()
            self.sent_on_start = True
            self.start_timer.cancel()
        except Exception as exc:
            self.get_logger().error(str(exc))

    def send_all(self):
        """YAML を読み込み、最初の waypoint から走行を始める."""
        config = load_config(self.yaml_path)
        waypoints = get_waypoints(config)
        if not waypoints:
            raise RuntimeError('No waypoints to send.')
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f'{ACTION_NAME} is not available.')

        self.waypoints = waypoints
        stops = [i for i, wp in enumerate(waypoints) if is_stop(wp)]
        self.get_logger().info(
            f'Loaded {len(waypoints)} waypoints from {self.yaml_path} '
            f'(stop: {stops}).')
        self._go_to(0)

    # -----------------------------------------------------------
    # 走行制御
    # -----------------------------------------------------------
    def _go_to(self, index):
        self.wp_index = index
        self.retries = 0
        self._send_current()

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
                f'Gave up waypoint {self.wp_index}. Call ~/next_wp to retry '
                'or ~/skip to go to the next waypoint.')

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

    def send_all_callback(self, request, response):
        """最初の waypoint からやり直す."""
        self._cancel_current()
        return self._respond(response, self._try_send_all())

    def next_wp_callback(self, request, response):
        """停止状態から走行を再開する.

        stop 点 -> 次の点へ / failed -> 現在の点を再送。
        """
        if self.state == STOPPED:
            self._advance()
            result = (True, f'Resumed to waypoint {self.wp_index}.')
        elif self.state == FAILED:
            self._go_to(self.wp_index)
            result = (True, f'Resumed waypoint {self.wp_index}.')
        elif self.state == IDLE:
            result = self._try_send_all()
        elif self.state == FINISHED:
            result = (False, 'Already reached the last waypoint.')
        else:
            result = (False, 'Already running.')
        return self._respond(response, result)

    def skip_callback(self, request, response):
        if self.state in (IDLE, FINISHED):
            return self._respond(response, (False, f'Nothing to skip ({self.state}).'))
        self._cancel_current()
        skipped = self.wp_index
        self._advance()
        return self._respond(response, (True, f'Skipped waypoint {skipped}.'))


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
