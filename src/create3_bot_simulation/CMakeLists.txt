cmake_minimum_required(VERSION 3.8)
project(create3_bot_simulation)

find_package(ament_cmake REQUIRED)

# Only launch files, bridge config and Gazebo worlds: no nodes are built in this package.
install(DIRECTORY config launch worlds DESTINATION share/${PROJECT_NAME})

ament_package()
