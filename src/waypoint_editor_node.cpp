// RViz の interactive marker で waypoint YAML を編集する node。
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "waypoint_tools/interactive_waypoints.hpp"
#include "waypoint_tools/node_params.hpp"
#include "waypoint_tools/waypoint_edit_markers.hpp"
#include "waypoint_tools/waypoint_yaml.hpp"

namespace waypoint_tools
{

using std_srvs::srv::Trigger;
using namespace std::chrono_literals;

class WaypointEditorNode : public rclcpp::Node
{
public:
  WaypointEditorNode()
  : Node("waypoint_editor_node")
  {
    require_parameters(
      *this, {
        {"yaml_path", rclcpp::PARAMETER_STRING},
        {"frame_id", rclcpp::PARAMETER_STRING},
      });
    frame_id_ = get_parameter("frame_id").as_string();
    yaml_path_ = checkFile(expand_user(get_parameter("yaml_path").as_string()));

    file_ = load_waypoint_file(yaml_path_);

    routes_.emplace(*this, frame_id_, 0.04);
    edit_markers_.emplace(
      *this, frame_id_, file_.waypoints,
      WaypointEditMarkers::Callbacks{
        [this]() {saveWaypoints();}, nullptr, nullptr, nullptr,
        [this]() {publishRoutes();}});

    save_service_ = create_service<Trigger>(
      "~/save", [this](
        const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        try {
          saveWaypoints();
        } catch (const std::exception & e) {
          response->success = false;
          response->message = e.what();
          return;
        }
        response->success = true;
        response->message = "Saved: " + yaml_path_;
      });
    reload_service_ = create_service<Trigger>(
      "~/reload", [this](
        const Trigger::Request::SharedPtr, Trigger::Response::SharedPtr response) {
        try {
          file_ = load_waypoint_file(yaml_path_);
          rebuildMarkers();
        } catch (const std::exception & e) {
          response->success = false;
          response->message = e.what();
          return;
        }
        response->success = true;
        response->message = "Reloaded: " + yaml_path_;
      });

    // yaml_path パラメータの動的変更で別のファイルを開く。
    param_callback_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        for (const auto & param : params) {
          if (param.get_name() != "yaml_path" ||
          param.get_type() != rclcpp::PARAMETER_STRING || param.as_string().empty())
          {
            continue;
          }
          try {
            loadFile(checkFile(expand_user(param.as_string())));
          } catch (const std::exception & e) {
            result.successful = false;
            result.reason = e.what();
          }
        }
        return result;
      });

    rebuildMarkers();
    timer_ = create_wall_timer(500ms, [this]() {publishRoutes();});

    RCLCPP_INFO(get_logger(), "Loaded waypoints: %s", yaml_path_.c_str());
  }

private:
  static std::string checkFile(const std::string & path)
  {
    if (!std::filesystem::is_regular_file(path)) {
      throw std::runtime_error("yaml_path must be a file: " + path);
    }
    return path;
  }

  void loadFile(const std::string & path)
  {
    auto file = load_waypoint_file(path);
    yaml_path_ = path;
    file_ = std::move(file);
    rebuildMarkers();
    RCLCPP_INFO(get_logger(), "Loaded: %s", yaml_path_.c_str());
  }

  void rebuildMarkers()
  {
    edit_markers_->show();
    publishRoutes();
  }

  void saveWaypoints()
  {
    save_waypoint_file(yaml_path_, file_);
    RCLCPP_INFO(get_logger(), "Saved waypoints: %s", yaml_path_.c_str());
  }

  void publishRoutes()
  {
    routes_->publish(file_.waypoints);
  }

  std::string frame_id_;
  std::string yaml_path_;
  WaypointFile file_;

  std::optional<RouteMarkers> routes_;
  std::optional<WaypointEditMarkers> edit_markers_;
  rclcpp::Service<Trigger>::SharedPtr save_service_;
  rclcpp::Service<Trigger>::SharedPtr reload_service_;
  OnSetParametersCallbackHandle::SharedPtr param_callback_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace waypoint_tools

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<waypoint_tools::WaypointEditorNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("waypoint_editor_node"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
