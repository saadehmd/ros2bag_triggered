cmake_minimum_required(VERSION 3.8)
project(ros2bag_triggered)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -lpthread -lutil -ldl)
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
if (rclcpp_VERSION VERSION_LESS "28.1.5")
    message(FATAL_ERROR "ROS2 Distro should be at least Jazzy.")
endif()
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2 REQUIRED)
find_package(rosgraph_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(matplotlib_cpp REQUIRED)

add_library(${PROJECT_NAME}_examples SHARED
  examples/battery_health_trigger.cpp
  examples/navsat_invalid_fix_trigger.cpp
  examples/zone_trigger_with_pose_stamped.cpp
  examples/zone_trigger_with_navsat_fix.cpp
)

add_library(${PROJECT_NAME} SHARED
  src/triggered_writer.cpp
)

add_executable(example_node examples/main.cpp)

set(ament_cmake_dependencies
  rclcpp
  rosbag2_cpp
  rosbag2_transport
  rosbag2_storage
  rosbag2
  std_msgs
  rosgraph_msgs
  geometry_msgs
  sensor_msgs
  ament_index_cpp
)

include_directories(SYSTEM include ${matplotlib_cpp_INCLUDE_DIRS})

ament_target_dependencies(${PROJECT_NAME} ${ament_cmake_dependencies})
target_link_libraries(${PROJECT_NAME} matplotlib_cpp::matplotlib_cpp)
ament_target_dependencies(${PROJECT_NAME}_examples ${ament_cmake_dependencies})
target_link_libraries(example_node ${PROJECT_NAME} ${PROJECT_NAME}_examples)
ament_target_dependencies(example_node ${ament_cmake_dependencies} )

install(TARGETS ${PROJECT_NAME}_examples 
  EXPORT export_${PROJECT_NAME}_examples 
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin
)

#install(TARGETS example_node
#  #EXPORT export_${PROJECT_NAME}_example_node
#  ARCHIVE DESTINATION lib/${PROJECT_NAME}
#  LIBRARY DESTINATION lib/${PROJECT_NAME}
#  RUNTIME DESTINATION bin
#)

install(TARGETS
  example_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config/
  DESTINATION config
)

install(
  DIRECTORY test/configs/
  DESTINATION test/configs
)

#ament_export_include_directories(include)
#ament_export_targets(export_${PROJECT_NAME} export_${PROJECT_NAME}_examples HAS_LIBRARY_TARGET)
#ament_export_libraries(${PROJECT_NAME} ${PROJECT_NAME}_examples)
#ament_export_dependencies(rclcpp rosbag2_cpp rosbag2_transport rosbag2 rosgraph_msgs geometry_msgs sensor_msgs)

set(BUILD_TESTING ON)
if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  #set(ament_cmake_copyright_FOUND TRUE)
  #set(ament_cmake_cpplint_FOUND TRUE)
  #ament_lint_auto_find_test_dependencies()


  set(TEST_SOURCE
    test/trigger_base_tests.cpp
    test/triggered_recorder_node_tests.cpp
    test/triggered_writer_tests.cpp)

  ament_add_gtest(trigger_base_tests ${TEST_SOURCE})
  set_tests_properties(trigger_base_tests PROPERTIES TIMEOUT 100)
  target_include_directories(trigger_base_tests PRIVATE include test)
  target_include_directories(trigger_base_tests SYSTEM PRIVATE ${GTEST_INCLUDE_DIRS})
  target_link_libraries(trigger_base_tests  ${PROJECT_NAME} ${PROJECT_NAME}_examples)

endif()

ament_package()
