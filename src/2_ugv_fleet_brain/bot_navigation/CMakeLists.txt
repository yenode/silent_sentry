cmake_minimum_required(VERSION 3.8)
project(bot_navigation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pluginlib REQUIRED)

# --- BT node: GoalToPath ---
add_library(goal_to_path_bt_node SHARED
  src/goal_to_path_bt_node.cpp
)

target_include_directories(goal_to_path_bt_node PRIVATE
  include
)

target_compile_definitions(goal_to_path_bt_node PRIVATE BT_PLUGIN_EXPORT)

ament_target_dependencies(goal_to_path_bt_node
  rclcpp
  behaviortree_cpp
  nav2_behavior_tree
  geometry_msgs
  nav_msgs
)

# --- Nav2 controller plugin: APFController ---
add_library(apf_controller SHARED
  src/apf_controller.cpp
)

target_include_directories(apf_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(apf_controller
  rclcpp
  nav2_core
  nav2_costmap_2d
  nav2_util
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_ros
  pluginlib
)

pluginlib_export_plugin_description_file(nav2_core apf_controller_plugin.xml)

install(TARGETS goal_to_path_bt_node apf_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch config maps behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(goal_to_path_bt_node apf_controller)
ament_export_include_directories(include)

ament_package()
