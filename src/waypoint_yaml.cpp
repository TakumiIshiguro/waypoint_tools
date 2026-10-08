#include "waypoint_tools/waypoint_yaml.hpp"

#include <charconv>
#include <cstdlib>
#include <fstream>
#include <stdexcept>

namespace waypoint_tools
{

namespace
{

// 最短で元の値に戻る表記。PyYAML でも float と読めるよう必ず "." を含める。
std::string format_double(double value)
{
  char buffer[64];
  const auto result = std::to_chars(buffer, buffer + sizeof(buffer), value);
  std::string text(buffer, result.ptr);
  if (text.find_first_of(".ein") == std::string::npos) {
    text += ".0";
  } else if (text.find('.') == std::string::npos && text.find('e') != std::string::npos) {
    text.insert(text.find('e'), ".0");
  }
  return text;
}

double get_double(const Waypoint & waypoint, const char * key)
{
  const YAML::Node value = waypoint.yaml()[key];
  return value ? value.as<double>() : 0.0;
}

}  // namespace

Waypoint::Waypoint()
: node_(YAML::NodeType::Map)
{
}

Waypoint::Waypoint(const YAML::Node & node)
: node_(node)
{
}

Waypoint & Waypoint::operator=(const Waypoint & other)
{
  node_.reset(other.node_);
  return *this;
}

Waypoint Waypoint::clone() const
{
  return Waypoint(YAML::Clone(node_));
}

std::string expand_user(const std::string & path)
{
  if (path.empty() || path[0] != '~' || (path.size() > 1 && path[1] != '/')) {
    return path;
  }
  const char * home = std::getenv("HOME");
  return home ? std::string(home) + path.substr(1) : path;
}

WaypointFile load_waypoint_file(const std::string & path)
{
  WaypointFile file;
  try {
    file.root = YAML::LoadFile(expand_user(path));
  } catch (const YAML::BadFile &) {
    throw std::runtime_error("Cannot open: " + path);
  }
  if (!file.root.IsMap()) {
    throw std::runtime_error("YAML root must be a map.");
  }
  const YAML::Node waypoints = file.root["waypoints"];
  if (!waypoints || !waypoints.IsSequence()) {
    throw std::runtime_error("YAML must contain a top-level \"waypoints\" list.");
  }
  for (const auto & waypoint : waypoints) {
    file.waypoints.emplace_back(waypoint);
  }
  return file;
}

void save_waypoint_file(const std::string & path, const WaypointFile & file)
{
  YAML::Node root = YAML::Clone(file.root);
  YAML::Node waypoints(YAML::NodeType::Sequence);
  for (const auto & waypoint : file.waypoints) {
    waypoints.push_back(waypoint.yaml());
  }
  root["waypoints"] = waypoints;

  YAML::Emitter emitter;
  emitter << root;
  std::ofstream out(expand_user(path));
  if (!out) {
    throw std::runtime_error("Cannot write: " + path);
  }
  out << emitter.c_str() << '\n';
}

WaypointPose get_pose(const Waypoint & waypoint)
{
  return {
    get_double(waypoint, "x"), get_double(waypoint, "y"),
    get_double(waypoint, "z"), get_double(waypoint, "yaw")};
}

void set_pose(Waypoint & waypoint, const WaypointPose & pose)
{
  YAML::Node & node = waypoint.yaml();
  node["x"] = format_double(pose.x);
  node["y"] = format_double(pose.y);
  node["z"] = format_double(pose.z);
  node["yaw"] = format_double(pose.yaw);
}

Waypoint make_waypoint(const WaypointPose & pose)
{
  Waypoint waypoint;
  set_pose(waypoint, pose);
  return waypoint;
}

bool is_stop(const Waypoint & waypoint)
{
  const YAML::Node stop = waypoint.yaml()["stop"];
  return stop && stop.IsScalar() && stop.as<bool>(false);
}

void set_stop(Waypoint & waypoint, bool stop)
{
  if (stop) {
    waypoint.yaml()["stop"] = true;
  } else {
    waypoint.yaml().remove("stop");
  }
}

double goal_radius(const Waypoint & waypoint, double fallback)
{
  const YAML::Node properties = waypoint.yaml()["properties"];
  if (!properties || !properties.IsMap()) {
    return fallback;
  }
  const YAML::Node radius = properties["goal_radius"];
  return radius ? radius.as<double>(fallback) : fallback;
}

}  // namespace waypoint_tools
