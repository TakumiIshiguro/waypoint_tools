#include "waypoint_tools/node_params.hpp"

#include <stdexcept>

namespace waypoint_tools
{

void require_parameters(rclcpp::Node & node, const ParameterSpecs & specs)
{
  std::string missing;
  for (const auto & [name, type] : specs) {
    node.declare_parameter(name, type);
    try {
      node.get_parameter(name);
    } catch (const rclcpp::exceptions::ParameterUninitializedException &) {
      missing += (missing.empty() ? "" : ", ") + name;
    }
  }
  if (!missing.empty()) {
    throw std::runtime_error(
            std::string(node.get_name()) + ": parameter not set: " + missing +
            " (config/params/waypoint_tools_params.yaml を確認してください)");
  }
}

}  // namespace waypoint_tools
