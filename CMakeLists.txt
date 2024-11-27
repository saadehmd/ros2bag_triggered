cmake_minimum_required(VERSION 3.8)
project(ros2bag_triggered)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_library(${PROJECT_NAME}_examples SHARED
  examples/battery_health_trigger.cpp
  examples/navsat_invalid_fix_trigger.cpp
  examples/zone_trigger.cpp
)

target_include_directories(${PROJECT_NAME}_examples PRIVATE include)

ament_target_dependencies(${PROJECT_NAME}_examples 
  rclcpp
  rosbag2_cpp
  rosbag2_transport
  rosbag2
  rosgraph_msgs
  geometry_msgs
  sensor_msgs
)

install(TARGETS ${PROJECT_NAME}_examples 
  EXPORT export_${PROJECT_NAME}_examples 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME}_examples HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME}_examples)
ament_export_dependencies(rclcpp rosbag2_cpp rosbag2_transport rosbag2 rosgraph_msgs geometry_msgs sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
