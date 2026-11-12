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

# NOTE: src/*.cpp are created in T-04; the build fails until then.
add_library(${PROJECT_NAME}_lib
  src/frontier_detector.cpp
  src/goal_evaluator.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

  ament_add_gtest(test_frontier_detector test/test_frontier_detector.cpp)
  target_link_libraries(test_frontier_detector ${PROJECT_NAME}_lib)

  ament_add_gtest(test_goal_evaluator test/test_goal_evaluator.cpp)
  target_link_libraries(test_goal_evaluator ${PROJECT_NAME}_lib)
endif()

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME})
ament_package()
