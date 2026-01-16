cmake_minimum_required(VERSION 3.8)
project(waypoint_bt_navigation)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav_msgs REQUIRED)

# Include directories
include_directories(include)

# Add executable nodes
add_executable(navigate_to_waypoint src/navigate_to_waypoint.cpp)
ament_target_dependencies(navigate_to_waypoint rclcpp behaviortree_cpp_v3 nav_msgs)

add_executable(spin_action src/spin_action.cpp)
ament_target_dependencies(spin_action rclcpp behaviortree_cpp_v3)

add_executable(final_spin_action src/final_spin_action.cpp)
ament_target_dependencies(final_spin_action rclcpp behaviortree_cpp_v3)

add_executable(waypoint_input src/waypoint_input.cpp)
ament_target_dependencies(waypoint_input rclcpp)

add_executable(behavior_tree_main src/behavior_tree_main.cpp)
ament_target_dependencies(behavior_tree_main rclcpp behaviortree_cpp_v3)

install(TARGETS
  navigate_to_waypoint
  spin_action
  final_spin_action
  waypoint_input
  behavior_tree_main
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}/)
install(DIRECTORY trees DESTINATION share/${PROJECT_NAME}/)

ament_package()
