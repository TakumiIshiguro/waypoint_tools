#ifndef WAYPOINT_TOOLS__WAYPOINT_EDIT_MARKERS_HPP_
#define WAYPOINT_TOOLS__WAYPOINT_EDIT_MARKERS_HPP_

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <interactive_markers/interactive_marker_server.hpp>
#include <interactive_markers/menu_handler.hpp>
#include <rclcpp/rclcpp.hpp>

#include "waypoint_tools/waypoint_yaml.hpp"

namespace waypoint_tools
{

// RViz の InteractiveMarkers 表示の namespace。
constexpr char kServerNamespace[] = "waypoint_tools";

// waypoint ごとの interactive marker と右クリックメニュー（editor / sender / recorder 共通）。
//
// コンストラクタで渡した waypoints を直接書き換える。
// - 円盤をドラッグ: 位置移動 / 矢印をドラッグ: yaw 回転
// - メニュー: insert after / delete / save / stop（+ addCheckEntry() で足した項目）
class WaypointEditMarkers
{
public:
  using Feedback = visualization_msgs::msg::InteractiveMarkerFeedback;
  using EntryHandle = interactive_markers::MenuHandler::EntryHandle;

  struct Options
  {
    bool stop_entry = true;  // メニューに stop（停止点の切り替え）を出す
    bool keep_last = true;   // 最後の 1 点は削除させない
  };

  // 呼び出し側への通知。on_insert / on_delete / on_move は変更した位置を、
  // on_change はすべての変更を知らせる。
  struct Callbacks
  {
    std::function<void()> save;
    std::function<void(size_t)> on_insert;
    std::function<void(size_t)> on_delete;
    std::function<void(size_t)> on_move;
    std::function<void()> on_change;
  };

  WaypointEditMarkers(
    rclcpp::Node & node, std::string frame_id, Waypoints & waypoints, Callbacks callbacks,
    Options options);
  WaypointEditMarkers(
    rclcpp::Node & node, std::string frame_id, Waypoints & waypoints, Callbacks callbacks)
  : WaypointEditMarkers(node, std::move(frame_id), waypoints, std::move(callbacks), Options{}) {}

  // 全 waypoint の marker を作り直す。
  void show();
  void hide();
  // index の marker だけ作り直す（末尾に追加した点など）。
  void update(size_t index);

  // チェック付きのメニュー項目を末尾に足す。callback はクリックで呼ばれる。
  EntryHandle addCheckEntry(
    const std::string & title, bool checked, std::function<void()> callback);
  void setChecked(EntryHandle handle, bool checked);

private:
  void makeMarker(size_t index);
  bool markerIndex(const Feedback & feedback, size_t & index) const;
  // marker のコールバック中に marker 自身を作り直さないよう、実行後に回す。
  void defer(std::function<void()> task);

  void poseUpdateCallback(const Feedback::ConstSharedPtr & feedback);
  void insertCallback(const Feedback::ConstSharedPtr & feedback);
  void deleteCallback(const Feedback::ConstSharedPtr & feedback);
  void stopCallback(const Feedback::ConstSharedPtr & feedback);

  rclcpp::Logger logger_;
  std::string frame_id_;
  Waypoints & waypoints_;
  Callbacks callbacks_;
  Options options_;

  std::unique_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;
  EntryHandle stop_handle_ = 0;

  rclcpp::TimerBase::SharedPtr defer_timer_;
  std::vector<std::function<void()>> deferred_;
};

}  // namespace waypoint_tools

#endif  // WAYPOINT_TOOLS__WAYPOINT_EDIT_MARKERS_HPP_
