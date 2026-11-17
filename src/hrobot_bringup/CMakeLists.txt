cmake_minimum_required(VERSION 3.8)
project(hrobot_bringup)

find_package(ament_cmake REQUIRED)

# Launch files, parameter files and two Python helpers (odom_relay.py, scan_resampler.py); no C++ is built here.
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/odom_relay.py scripts/scan_resampler.py DESTINATION lib/${PROJECT_NAME})

ament_package()
