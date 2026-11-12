cmake_minimum_required(VERSION 3.8)
project(create3_bot_bringup)

find_package(ament_cmake REQUIRED)

# Only launch files and parameter files: no nodes are built in this package.
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
