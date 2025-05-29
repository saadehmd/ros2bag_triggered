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
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(matplotlib_cpp REQUIRED)
find_package(std_msgs REQUIRED)


add_library(${PROJECT_NAME} SHARED
  src/triggered_writer.cpp
)

set(ament_cmake_dependencies
  rclcpp
  rosbag2_cpp
  rosbag2_transport
  rosbag2_storage
  rosbag2
  rosgraph_msgs
  ament_index_cpp
  std_msgs
)

include_directories(SYSTEM ${matplotlib_cpp_INCLUDE_DIRS})

ament_target_dependencies(${PROJECT_NAME} ${ament_cmake_dependencies})
target_link_libraries(${PROJECT_NAME} matplotlib_cpp::matplotlib_cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
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

ament_export_include_directories(include)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(rclcpp rosbag2_cpp rosbag2_transport rosbag2_storage rosbag2 rosgraph_msgs std_msgs matplotlib_cpp yaml-cpp)

set(BUILD_TESTING ON)
if(BUILD_TESTING)
  #find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(sensor_msgs REQUIRED)

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
  target_link_libraries(trigger_base_tests  ${PROJECT_NAME})
  ament_target_dependencies(trigger_base_tests sensor_msgs std_msgs)

endif()

ament_package()
