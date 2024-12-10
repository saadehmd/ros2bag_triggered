cmake_minimum_required(VERSION 3.8)
project(ros2bag_triggered)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}_examples SHARED
  examples/battery_health_trigger.cpp
  examples/navsat_invalid_fix_trigger.cpp
  examples/zone_trigger_with_pose_stamped.cpp
  examples/zone_trigger_with_navsat_fix.cpp
)

add_library(${PROJECT_NAME} SHARED
  src/triggered_recorder_node.cpp
)

set(ament_cmake_dependencies
  rclcpp
  rosbag2_cpp
  rosbag2_transport
  rosbag2_storage
  rosbag2
  rosgraph_msgs
  geometry_msgs
  sensor_msgs
  ament_index_cpp
)

include_directories(SYSTEM include)
#target_include_directories(${PROJECT_NAME}_examples ${PROJECT_NAME} PRIVATE include)

ament_target_dependencies(${PROJECT_NAME}_examples ${ament_cmake_dependencies})
ament_target_dependencies(${PROJECT_NAME} ${ament_cmake_dependencies})

install(TARGETS ${PROJECT_NAME}_examples 
  EXPORT export_${PROJECT_NAME}_examples 
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

#ament_export_include_directories(include)
#ament_export_targets(export_${PROJECT_NAME} export_${PROJECT_NAME}_examples HAS_LIBRARY_TARGET)
#ament_export_libraries(${PROJECT_NAME} ${PROJECT_NAME}_examples)
#ament_export_dependencies(rclcpp rosbag2_cpp rosbag2_transport rosbag2 rosgraph_msgs geometry_msgs sensor_msgs)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
