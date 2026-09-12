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
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

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

install(TARGETS goal_to_path_bt_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY launch config maps behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(goal_to_path_bt_node)

ament_package()
