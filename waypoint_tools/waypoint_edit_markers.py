"""waypoint を RViz の interactive marker で編集する（editor / sender 共通）."""
import copy

from interactive_markers import InteractiveMarkerServer, MenuHandler
from visualization_msgs.msg import InteractiveMarkerFeedback

from waypoint_tools.action_sender import quaternion_to_yaw
from waypoint_tools.interactive_waypoints import (
    DISC_COLOR, STOP_DISC_COLOR, build_waypoint_marker)
from waypoint_tools.waypoint_yaml import get_xyz_yaw, is_stop, set_xyz_yaw

# RViz の InteractiveMarkers 表示の namespace。
SERVER_NAMESPACE = 'waypoint_tools'


class WaypointEditMarkers:
    """waypoint ごとの interactive marker と右クリックメニュー.

    get_waypoints() が返すリストを直接書き換える。
    - 円盤をドラッグ: 位置移動 / 矢印をドラッグ: yaw 回転
    - メニュー: insert after / delete / save / stop
    on_insert(index) / on_delete(index) は挿入・削除した位置を、
    on_change() はすべての変更を呼び出し側に知らせる。
    """

    def __init__(self, node, frame_id, get_waypoints, save,
                 on_insert=None, on_delete=None, on_change=None):
        self.node = node
        self.frame_id = frame_id
        self.get_waypoints = get_waypoints
        self.save = save
        self.on_insert = on_insert or (lambda index: None)
        self.on_delete = on_delete or (lambda index: None)
        self.on_change = on_change or (lambda: None)

        self.server = InteractiveMarkerServer(node, SERVER_NAMESPACE)
        self.menu_handler = MenuHandler()
        self.menu_handler.insert('insert after', callback=self._insert_callback)
        self.menu_handler.insert('delete', callback=self._delete_callback)
        self.menu_handler.insert('save', callback=lambda feedback: self.save())
        # チェックで停止点（stop: true）。状態は marker ごとに _make_marker で反映する。
        self.stop_handle = self.menu_handler.insert(
            'stop', callback=self._stop_callback)

    def show(self):
        """全 waypoint の marker を作り直す."""
        self.server.clear()
        for index in range(len(self.get_waypoints())):
            self._make_marker(index)
        self.server.applyChanges()

    def hide(self):
        self.server.clear()
        self.server.applyChanges()

    def _make_marker(self, index):
        waypoint = self.get_waypoints()[index]
        x, y, _, yaw = get_xyz_yaw(waypoint)
        stop = is_stop(waypoint)
        marker = build_waypoint_marker(
            str(index), self.frame_id, x, y, yaw,
            self._marker_scale(waypoint),
            f'waypoint {index}' + (' (stop)' if stop else ''),
            STOP_DISC_COLOR if stop else DISC_COLOR)
        self.server.insert(marker)
        self.server.setCallback(
            marker.name, self._pose_update_callback,
            InteractiveMarkerFeedback.POSE_UPDATE)
        # MenuHandler のチェック状態は apply 時点の値が marker に焼き込まれる。
        self.menu_handler.setCheckState(
            self.stop_handle,
            MenuHandler.CHECKED if stop else MenuHandler.UNCHECKED)
        self.menu_handler.apply(self.server, marker.name)

    @staticmethod
    def _marker_scale(waypoint):
        properties = waypoint.get('properties', {})
        radius = float(properties.get('goal_radius', 1.0))
        return max(radius, 0.3)

    def _pose_update_callback(self, feedback):
        index = int(feedback.marker_name)
        waypoints = self.get_waypoints()
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
        self.on_change()

    def _stop_callback(self, feedback):
        index = int(feedback.marker_name)
        waypoint = self.get_waypoints()[index]
        if is_stop(waypoint):
            waypoint.pop('stop', None)
        else:
            waypoint['stop'] = True
        self._make_marker(index)
        self.server.applyChanges()
        self.node.get_logger().info(
            f'waypoint {index}: stop={is_stop(waypoint)} (save to keep)')
        self.on_change()

    def _insert_callback(self, feedback):
        index = int(feedback.marker_name)
        waypoints = self.get_waypoints()
        x, y, z, yaw = get_xyz_yaw(waypoints[index])
        new_waypoint = copy.deepcopy(waypoints[index])
        new_waypoint.pop('stop', None)
        set_xyz_yaw(new_waypoint, x + 0.5, y, z, yaw)
        waypoints.insert(index + 1, new_waypoint)
        self.on_insert(index + 1)
        self.show()
        self.on_change()

    def _delete_callback(self, feedback):
        waypoints = self.get_waypoints()
        if len(waypoints) <= 1:
            self.node.get_logger().warn('Cannot delete the last waypoint.')
            return
        index = int(feedback.marker_name)
        del waypoints[index]
        self.on_delete(index)
        self.show()
        self.on_change()
