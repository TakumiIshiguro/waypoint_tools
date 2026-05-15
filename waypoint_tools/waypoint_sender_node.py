#!/usr/bin/env python3
import rclpy
from nav2_msgs.action import FollowWaypoints
from rclpy.action import ActionClient
from rclpy.node import Node
from std_srvs.srv import Trigger

from waypoint_tools.action_sender import make_follow_waypoints_goal
from waypoint_tools.waypoint_yaml import load_config


DEFAULT_WAYPOINT_YAML_PATH = (
    '/home/takumi/ros2_ws/src/orne-box/orne_box_navigation_executor/'
    'config/waypoints/tsudanuma2-3.yaml'
)


class WaypointSenderNode(Node):
    def __init__(self):
        super().__init__('waypoint_sender_node')

        self.declare_parameter('yaml_path', DEFAULT_WAYPOINT_YAML_PATH)
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('action_name', '/follow_waypoints')
        self.declare_parameter('send_on_start', True)

        self.yaml_path = self.get_parameter(
            'yaml_path').get_parameter_value().string_value
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
        self.goal_handle = None
        self.sent_on_start = False

        if self.send_on_start:
            self.start_timer = self.create_timer(0.5, self.send_once)

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

    def send_all(self):
        config, yaml_format = load_config(self.yaml_path)
        goal = make_follow_waypoints_goal(
            config, self.frame_id, self.get_clock().now().to_msg())

        if not goal.poses:
            raise RuntimeError('No waypoints to send.')

        self.get_logger().info(
            f'Loaded {len(goal.poses)} waypoints ({yaml_format}).')
        self.get_logger().info(f'Waiting for {self.action_name}...')
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError(f'{self.action_name} is not available.')

        future = self.action_client.send_goal_async(
            goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.response_callback)
        self.get_logger().info('FollowWaypoints goal sent.')

    def feedback_callback(self, feedback_msg):
        current = feedback_msg.feedback.current_waypoint
        self.get_logger().info(f'Current waypoint: {current}')

    def response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('FollowWaypoints goal rejected.')
            return

        self.get_logger().info('FollowWaypoints goal accepted.')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if result.missed_waypoints:
            self.get_logger().warn(
                f'Missed waypoints: {result.missed_waypoints}')
        self.get_logger().info(f'FollowWaypoints finished: {status}')


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
