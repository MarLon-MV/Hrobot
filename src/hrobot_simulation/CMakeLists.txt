cmake_minimum_required(VERSION 3.8)
project(hrobot_simulation)

find_package(ament_cmake REQUIRED)

# Alleen launch-bestanden, brugconfiguratie en Gazebo-werelden: dit pakket bouwt geen nodes.
install(DIRECTORY config launch worlds DESTINATION share/${PROJECT_NAME})

ament_package()
