cmake_minimum_required(VERSION 3.8)
project(create3_autonomous_explorer)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(create3_bot_interfaces REQUIRED)

# Pure C++ library: no ROS dependencies (domain logic, unit-testable).
add_library(${PROJECT_NAME}_lib
  src/frontier_detector.cpp
  src/goal_evaluator.cpp
  src/map_naming.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# ROS 2 node executable (ExplorerNode + main).
add_executable(explorer_node
  src/explorer_node.cpp
  src/main.cpp
)
target_link_libraries(explorer_node ${PROJECT_NAME}_lib)
ament_target_dependencies(explorer_node
  rclcpp
  rclcpp_action
  nav_msgs
  nav2_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  create3_bot_interfaces
)

# Map manager node (SaveExplorationMap service -> nav2 map_saver + slam_toolbox serialize).
add_executable(map_manager_node
  src/map_manager_node.cpp
  src/map_manager_main.cpp
)
target_link_libraries(map_manager_node ${PROJECT_NAME}_lib)
ament_target_dependencies(map_manager_node
  rclcpp
  nav2_msgs
  slam_toolbox
  create3_bot_interfaces
)

# Odometry QoS relay (Create 3 BEST_EFFORT odom -> RELIABLE odom_reliable for Nav2).
add_executable(odom_relay_node
  src/odom_relay_node.cpp
)
ament_target_dependencies(odom_relay_node
  rclcpp
  nav_msgs
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS explorer_node map_manager_node odom_relay_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_frontier_detector test/test_frontier_detector.cpp)
  target_link_libraries(test_frontier_detector ${PROJECT_NAME}_lib)

  ament_add_gtest(test_goal_evaluator test/test_goal_evaluator.cpp)
  target_link_libraries(test_goal_evaluator ${PROJECT_NAME}_lib)

  ament_add_gtest(test_map_naming test/test_map_naming.cpp)
  target_link_libraries(test_map_naming ${PROJECT_NAME}_lib)
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_action
  nav_msgs
  nav2_msgs
  geometry_msgs
  visualization_msgs
  tf2
  tf2_ros
  slam_toolbox
  create3_bot_interfaces
)
ament_package()
