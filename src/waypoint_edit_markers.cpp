#include "waypoint_tools/waypoint_edit_markers.hpp"

#include <algorithm>
#include <chrono>
#include <utility>

#include "waypoint_tools/interactive_waypoints.hpp"

namespace waypoint_tools
{

using interactive_markers::MenuHandler;
using namespace std::chrono_literals;

namespace
{

void call(const std::function<void(size_t)> & callback, size_t index)
{
  if (callback) {
    callback(index);
  }
}

void call(const std::function<void()> & callback)
{
  if (callback) {
    callback();
  }
}

double marker_scale(const Waypoint & waypoint)
{
  return std::max(goal_radius(waypoint, 1.0), 0.3);
}

}  // namespace

WaypointEditMarkers::WaypointEditMarkers(
  rclcpp::Node & node, std::string frame_id, Waypoints & waypoints, Callbacks callbacks,
  Options options)
: logger_(node.get_logger()),
  frame_id_(std::move(frame_id)),
  waypoints_(waypoints),
  callbacks_(std::move(callbacks)),
  options_(options),
  server_(std::make_unique<interactive_markers::InteractiveMarkerServer>(
      kServerNamespace, &node))
{
  menu_handler_.insert(
    "insert after", [this](const auto & feedback) {insertCallback(feedback);});
  menu_handler_.insert(
    "delete", [this](const auto & feedback) {deleteCallback(feedback);});
  menu_handler_.insert("save", [this](const auto &) {call(callbacks_.save);});
  if (options_.stop_entry) {
    // チェックで停止点（stop: true）。状態は marker ごとに makeMarker で反映する。
    stop_handle_ = menu_handler_.insert(
      "stop", [this](const auto & feedback) {stopCallback(feedback);});
  }

  defer_timer_ = node.create_wall_timer(
    1ms, [this]() {
      defer_timer_->cancel();
      auto tasks = std::move(deferred_);
      deferred_.clear();
      for (auto & task : tasks) {
        task();
      }
    });
  defer_timer_->cancel();
}

void WaypointEditMarkers::show()
{
  server_->clear();
  for (size_t index = 0; index < waypoints_.size(); ++index) {
    makeMarker(index);
  }
  server_->applyChanges();
}

void WaypointEditMarkers::hide()
{
  // 消した後に、待っていた作り直しで marker が復活しないようにする。
  deferred_.clear();
  defer_timer_->cancel();
  server_->clear();
  server_->applyChanges();
}

void WaypointEditMarkers::update(size_t index)
{
  if (index >= waypoints_.size()) {
    return;
  }
  makeMarker(index);
  server_->applyChanges();
}

WaypointEditMarkers::EntryHandle WaypointEditMarkers::addCheckEntry(
  const std::string & title, bool checked, std::function<void()> callback)
{
  const auto handle = menu_handler_.insert(
    title, [callback = std::move(callback)](const auto &) {callback();});
  menu_handler_.setCheckState(handle, checked ? MenuHandler::CHECKED : MenuHandler::UNCHECKED);
  return handle;
}

void WaypointEditMarkers::setChecked(EntryHandle handle, bool checked)
{
  menu_handler_.setCheckState(handle, checked ? MenuHandler::CHECKED : MenuHandler::UNCHECKED);
  defer(
    [this]() {
      menu_handler_.reApply(*server_);
      server_->applyChanges();
    });
}

void WaypointEditMarkers::makeMarker(size_t index)
{
  const Waypoint & waypoint = waypoints_[index];
  const auto pose = get_pose(waypoint);
  const bool stop = options_.stop_entry && is_stop(waypoint);
  const auto marker = build_waypoint_marker(
    std::to_string(index), frame_id_, pose.x, pose.y, pose.yaw, marker_scale(waypoint),
    "waypoint " + std::to_string(index) + (stop ? " (stop)" : ""),
    stop ? kStopDiscColor : kDiscColor);
  server_->insert(marker);
  server_->setCallback(
    marker.name, [this](const auto & feedback) {poseUpdateCallback(feedback);},
    Feedback::POSE_UPDATE);
  if (options_.stop_entry) {
    // MenuHandler のチェック状態は apply 時点の値が marker に焼き込まれる。
    menu_handler_.setCheckState(
      stop_handle_, stop ? MenuHandler::CHECKED : MenuHandler::UNCHECKED);
  }
  menu_handler_.apply(*server_, marker.name);
}

bool WaypointEditMarkers::markerIndex(const Feedback & feedback, size_t & index) const
{
  try {
    index = std::stoul(feedback.marker_name);
  } catch (const std::exception &) {
    return false;
  }
  return index < waypoints_.size();
}

void WaypointEditMarkers::defer(std::function<void()> task)
{
  deferred_.push_back(std::move(task));
  defer_timer_->reset();
}

void WaypointEditMarkers::poseUpdateCallback(const Feedback::ConstSharedPtr & feedback)
{
  size_t index;
  if (!markerIndex(*feedback, index)) {
    return;
  }
  auto pose = feedback->pose;
  const double old_z = get_pose(waypoints_[index]).z;
  set_pose(
    waypoints_[index], {pose.position.x, pose.position.y, old_z, quaternion_to_yaw(pose.orientation)});
  pose.position.z = 0.0;
  server_->setPose(feedback->marker_name, pose);
  server_->applyChanges();
  call(callbacks_.on_move, index);
  call(callbacks_.on_change);
}

void WaypointEditMarkers::stopCallback(const Feedback::ConstSharedPtr & feedback)
{
  size_t index;
  if (!markerIndex(*feedback, index)) {
    return;
  }
  Waypoint & waypoint = waypoints_[index];
  set_stop(waypoint, !is_stop(waypoint));
  defer([this, index]() {update(index);});
  RCLCPP_INFO(
    logger_, "waypoint %zu: stop=%s (save to keep)", index,
    is_stop(waypoint) ? "true" : "false");
  call(callbacks_.on_change);
}

void WaypointEditMarkers::insertCallback(const Feedback::ConstSharedPtr & feedback)
{
  size_t index;
  if (!markerIndex(*feedback, index)) {
    return;
  }
  auto pose = get_pose(waypoints_[index]);
  Waypoint new_waypoint = waypoints_[index].clone();
  set_stop(new_waypoint, false);
  pose.x += 0.5;
  set_pose(new_waypoint, pose);
  waypoints_.insert(waypoints_.begin() + static_cast<std::ptrdiff_t>(index) + 1, new_waypoint);
  call(callbacks_.on_insert, index + 1);
  defer([this]() {show();});
  call(callbacks_.on_change);
}

void WaypointEditMarkers::deleteCallback(const Feedback::ConstSharedPtr & feedback)
{
  size_t index;
  if (!markerIndex(*feedback, index)) {
    return;
  }
  if (options_.keep_last && waypoints_.size() <= 1) {
    RCLCPP_WARN(logger_, "Cannot delete the last waypoint.");
    return;
  }
  waypoints_.erase(waypoints_.begin() + static_cast<std::ptrdiff_t>(index));
  call(callbacks_.on_delete, index);
  defer([this]() {show();});
  call(callbacks_.on_change);
}

}  // namespace waypoint_tools
