cmake_minimum_required(VERSION 3.8)
project(waypoint_tools)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# ---- node 共通ライブラリ ----
add_library(${PROJECT_NAME}_core SHARED
  src/interactive_waypoints.cpp
  src/node_params.cpp
  src/waypoint_edit_markers.cpp
  src/waypoint_yaml.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_core
  geometry_msgs interactive_markers rclcpp std_msgs visualization_msgs)
target_link_libraries(${PROJECT_NAME}_core yaml-cpp)

# ---- node ----
foreach(node waypoint_editor_node waypoint_sender_node waypoint_recorder_node)
  add_executable(${node} src/${node}.cpp)
  target_link_libraries(${node} ${PROJECT_NAME}_core)
endforeach()
ament_target_dependencies(waypoint_editor_node
  rcl_interfaces rclcpp std_srvs visualization_msgs)
ament_target_dependencies(waypoint_sender_node
  geometry_msgs nav2_msgs rclcpp rclcpp_action std_srvs tf2 tf2_ros visualization_msgs)
ament_target_dependencies(waypoint_recorder_node
  geometry_msgs rclcpp std_srvs tf2 tf2_ros visualization_msgs)

# ---- RViz panel ----
set(CMAKE_AUTOMOC ON)
add_library(${PROJECT_NAME}_rviz SHARED
  src/rviz/waypoint_sender_panel.cpp
  include/waypoint_tools/waypoint_sender_panel.hpp
)
target_include_directories(${PROJECT_NAME}_rviz PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}_rviz
  pluginlib rclcpp rviz_common std_srvs)
target_link_libraries(${PROJECT_NAME}_rviz Qt5::Widgets)
pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)

# launch ファイルが使うパス解決ヘルパ（waypoint_tools.paths）。
ament_python_install_package(${PROJECT_NAME})

install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_rviz
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS waypoint_editor_node waypoint_sender_node waypoint_recorder_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
