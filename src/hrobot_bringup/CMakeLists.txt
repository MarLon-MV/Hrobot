cmake_minimum_required(VERSION 3.8)
project(hrobot_bringup)

find_package(ament_cmake REQUIRED)

# Launch-bestanden, parameterbestanden en twee Python-hulpscripts (odom_relay.py, scan_resampler.py); hier wordt geen C++ gebouwd.
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/odom_relay.py scripts/scan_resampler.py DESTINATION lib/${PROJECT_NAME})

ament_package()
