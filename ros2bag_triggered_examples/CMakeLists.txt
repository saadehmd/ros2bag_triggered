cmake_minimum_required(VERSION 3.8)
project(ros2bag_triggered_examples)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(ros2bag_triggered REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/battery_health_trigger.cpp
  src/navsat_invalid_fix_trigger.cpp
  src/zone_trigger_with_pose_stamped.cpp
  src/zone_trigger_with_navsat_fix.cpp
  src/pose_covariance_trigger.cpp
  src/velocity_trigger.cpp
  src/joint_effort_trigger.cpp
)

add_executable(example_node src/main.cpp)

set(ament_cmake_dependencies
  rclcpp
  ros2bag_triggered
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  trajectory_msgs
)

ament_target_dependencies(${PROJECT_NAME} ${ament_cmake_dependencies})
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(example_node ${PROJECT_NAME})
target_include_directories(example_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(TARGETS
    example_node
    DESTINATION lib/${PROJECT_NAME}
)


install(DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION config
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_CURRENT_SOURCE_DIR}/.clang-format")
  # Exclude uncrustify and use clang-format instead
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
