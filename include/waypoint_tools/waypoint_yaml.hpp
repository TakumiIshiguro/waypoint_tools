#ifndef WAYPOINT_TOOLS__WAYPOINT_YAML_HPP_
#define WAYPOINT_TOOLS__WAYPOINT_YAML_HPP_

#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace waypoint_tools
{

// waypoint 1 点。YAML の map のまま持つ（x/y/z/yaw/stop 以外のキーも保存時に残す）。
//
// YAML::Node の代入は参照先の値を書き換えるため、YAML::Node を直接 std::vector に入れると
// insert / erase で要素がずれたときに waypoint 同士が同じ node を指してしまう。
// このクラスの代入は参照先を付け替えるだけにする。コピーは同じ node を指す。
class Waypoint
{
public:
  Waypoint();
  explicit Waypoint(const YAML::Node & node);
  Waypoint(const Waypoint & other) = default;
  Waypoint & operator=(const Waypoint & other);

  // 別の node として複製する。
  Waypoint clone() const;

  const YAML::Node & yaml() const {return node_;}
  YAML::Node & yaml() {return node_;}

private:
  YAML::Node node_;
};

using Waypoints = std::vector<Waypoint>;

struct WaypointPose
{
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  double yaw = 0.0;
};

// waypoint YAML ファイル（トップレベルの "waypoints" リスト）。
struct WaypointFile
{
  YAML::Node root{YAML::NodeType::Map};  // waypoints 以外のキーも保存時に残す
  Waypoints waypoints;
};

// "~" を展開する。
std::string expand_user(const std::string & path);

WaypointFile load_waypoint_file(const std::string & path);
void save_waypoint_file(const std::string & path, const WaypointFile & file);

WaypointPose get_pose(const Waypoint & waypoint);
void set_pose(Waypoint & waypoint, const WaypointPose & pose);
Waypoint make_waypoint(const WaypointPose & pose);

// stop: true の waypoint では到達後に停止し、next_wp を待つ。
bool is_stop(const Waypoint & waypoint);
void set_stop(Waypoint & waypoint, bool stop);

// properties.goal_radius（無ければ fallback）。
double goal_radius(const Waypoint & waypoint, double fallback);

}  // namespace waypoint_tools

#endif  // WAYPOINT_TOOLS__WAYPOINT_YAML_HPP_
