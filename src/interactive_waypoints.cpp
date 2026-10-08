#include "waypoint_tools/interactive_waypoints.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <geometry_msgs/msg/point.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace waypoint_tools
{

using visualization_msgs::msg::InteractiveMarker;
using visualization_msgs::msg::InteractiveMarkerControl;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace
{

Marker make_disc_marker(double scale, const std_msgs::msg::ColorRGBA & color)
{
  Marker marker;
  marker.type = Marker::CYLINDER;
  marker.scale.x = scale;
  marker.scale.y = scale;
  marker.scale.z = 0.04;
  marker.color = color;
  return marker;
}

Marker make_arrow_marker(double scale)
{
  Marker marker;
  marker.type = Marker::ARROW;
  marker.scale.x = std::max(scale * 0.6, 0.35);
  marker.scale.y = 0.08;
  marker.scale.z = 0.08;
  marker.color = rgba(1.0f, 0.2f, 0.1f, 1.0f);
  return marker;
}

// z 軸まわりに動かす control（orientation は y 軸を上に向ける）。
InteractiveMarkerControl make_control(uint8_t interaction_mode, const Marker & marker)
{
  InteractiveMarkerControl control;
  control.orientation.w = 1.0;
  control.orientation.x = 0.0;
  control.orientation.y = 1.0;
  control.orientation.z = 0.0;
  control.interaction_mode = interaction_mode;
  control.orientation_mode = InteractiveMarkerControl::INHERIT;
  control.always_visible = true;
  control.markers.push_back(marker);
  return control;
}

geometry_msgs::msg::Point point(double x, double y)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  return p;
}

}  // namespace

std_msgs::msg::ColorRGBA rgba(float r, float g, float b, float a)
{
  std_msgs::msg::ColorRGBA color;
  color.r = r;
  color.g = g;
  color.b = b;
  color.a = a;
  return color;
}

geometry_msgs::msg::Quaternion yaw_to_quaternion(double yaw)
{
  geometry_msgs::msg::Quaternion q;
  q.x = 0.0;
  q.y = 0.0;
  q.z = std::sin(yaw * 0.5);
  q.w = std::cos(yaw * 0.5);
  return q;
}

double quaternion_to_yaw(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

InteractiveMarker build_waypoint_marker(
  const std::string & name, const std::string & frame_id, double x, double y, double yaw,
  double scale, const std::string & description, const std_msgs::msg::ColorRGBA & disc_color)
{
  InteractiveMarker marker;
  marker.header.frame_id = frame_id;
  marker.name = name;
  marker.description = description;
  marker.scale = static_cast<float>(scale);
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = 0.0;
  marker.pose.orientation = yaw_to_quaternion(yaw);

  marker.controls.push_back(
    make_control(InteractiveMarkerControl::MOVE_PLANE, make_disc_marker(scale, disc_color)));
  marker.controls.push_back(
    make_control(InteractiveMarkerControl::ROTATE_AXIS, make_arrow_marker(scale)));

  InteractiveMarkerControl menu_control;
  menu_control.interaction_mode = InteractiveMarkerControl::BUTTON;
  marker.controls.push_back(menu_control);

  return marker;
}

RouteMarkers::RouteMarkers(rclcpp::Node & node, std::string frame_id, double line_width)
: clock_(node.get_clock()),
  publisher_(node.create_publisher<MarkerArray>(kRouteTopic, 10)),
  frame_id_(std::move(frame_id)),
  line_width_(line_width)
{
}

void RouteMarkers::publish(const Waypoints & waypoints)
{
  MarkerArray marker_array;
  const auto stamp = clock_->now();
  const size_t segment_count = waypoints.empty() ? 0 : waypoints.size() - 1;

  for (size_t index = 0; index < segment_count; ++index) {
    const auto p1 = get_pose(waypoints[index]);
    const auto p2 = get_pose(waypoints[index + 1]);
    Marker route;
    route.header.frame_id = frame_id_;
    route.header.stamp = stamp;
    route.ns = "waypoint_routes";
    route.id = static_cast<int>(index);
    route.type = Marker::LINE_STRIP;
    route.action = Marker::ADD;
    route.scale.x = line_width_;
    route.color = rgba(1.0f, 0.9f, 0.0f, 1.0f);
    route.points.push_back(point(p1.x, p1.y));
    route.points.push_back(point(p2.x, p2.y));
    marker_array.markers.push_back(route);
  }

  for (size_t index = segment_count; index < published_count_; ++index) {
    Marker stale;
    stale.header.frame_id = frame_id_;
    stale.header.stamp = stamp;
    stale.ns = "waypoint_routes";
    stale.id = static_cast<int>(index);
    stale.action = Marker::DELETE;
    marker_array.markers.push_back(stale);
  }
  published_count_ = segment_count;

  publisher_->publish(marker_array);
}

}  // namespace waypoint_tools
