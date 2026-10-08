#ifndef WAYPOINT_TOOLS__NODE_PARAMS_HPP_
#define WAYPOINT_TOOLS__NODE_PARAMS_HPP_

#include <string>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace waypoint_tools
{

using ParameterSpecs = std::vector<std::pair<std::string, rclcpp::ParameterType>>;

// specs を既定値なしで宣言する。値は node.get_parameter() で取り出す。
// 1 つでも未設定なら、未設定の名前をすべて挙げて std::runtime_error を投げる。
void require_parameters(rclcpp::Node & node, const ParameterSpecs & specs);

}  // namespace waypoint_tools

#endif  // WAYPOINT_TOOLS__NODE_PARAMS_HPP_
