#ifndef WAYPOINT_TOOLS__INTERACTIVE_WAYPOINTS_HPP_
#define WAYPOINT_TOOLS__INTERACTIVE_WAYPOINTS_HPP_

#include <string>

#include <geometry_msgs/msg/quaternion.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "waypoint_tools/waypoint_yaml.hpp"

namespace waypoint_tools
{

// 経路マーカーの topic。RViz 設定（config/rviz/waypoint_tools.rviz）と揃える。
constexpr char kRouteTopic[] = "/waypoint_tools/routes";

std_msgs::msg::ColorRGBA rgba(float r, float g, float b, float a);

// 円盤の色。停止点（stop: true）は赤で区別する。
inline const std_msgs::msg::ColorRGBA kDiscColor = rgba(0.1f, 0.7f, 1.0f, 0.45f);
inline const std_msgs::msg::ColorRGBA kStopDiscColor = rgba(1.0f, 0.1f, 0.1f, 0.6f);

geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw);
double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & quaternion);

// 位置移動(平面) + yaw 回転 + メニュー用ボタンを持つ InteractiveMarker。
visualization_msgs::msg::InteractiveMarker build_waypoint_marker(
  const std::string & name, const std::string & frame_id, double x, double y, double yaw,
  double scale, const std::string & description = "",
  const std_msgs::msg::ColorRGBA & disc_color = kDiscColor);

// waypoint 間の経路を区間ごとの線で publish する（editor / recorder 共通）。
class RouteMarkers
{
public:
  RouteMarkers(rclcpp::Node & node, std::string frame_id, double line_width);

  void publish(const Waypoints & waypoints);

private:
  rclcpp::Clock::SharedPtr clock_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr publisher_;
  std::string frame_id_;
  double line_width_;
  // 直近で publish した区間数。waypoint が減ったとき、余った古い marker を DELETE する。
  size_t published_count_ = 0;
};

}  // namespace waypoint_tools

#endif  // WAYPOINT_TOOLS__INTERACTIVE_WAYPOINTS_HPP_
