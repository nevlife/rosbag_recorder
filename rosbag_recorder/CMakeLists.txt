cmake_minimum_required(VERSION 3.16)
project(rosbag_recorder)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(flux_cpp REQUIRED)
find_package(flux_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rmw REQUIRED)
find_package(rosbag2_compression REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(rosbag_recorder_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}_core SHARED
  src/catalog_diff.cpp
  src/config.cpp
  src/disk_watch.cpp
  src/preflight.cpp
  src/selection.cpp
  src/session_info.cpp
  src/start_gate.cpp
  src/stream.cpp
  src/summary.cpp
  src/text.cpp)
target_link_libraries(${PROJECT_NAME}_core PUBLIC
  rclcpp::rclcpp
  ${rosbag_recorder_msgs_TARGETS}
  ${tf2_msgs_TARGETS}
  yaml-cpp
  OpenSSL::Crypto)

add_library(${PROJECT_NAME}_component SHARED src/recorder_node.cpp src/flux_input.cpp)
target_compile_definitions(${PROJECT_NAME}_component PRIVATE
  ROSBAG_RECORDER_ROSBAG2_MINOR=${rosbag2_cpp_VERSION_MINOR})
target_link_libraries(${PROJECT_NAME}_component PUBLIC
  ${PROJECT_NAME}_core
  flux_cpp::flux_cpp
  ${flux_msgs_TARGETS}
  rclcpp_components::component
  rmw::rmw
  rosbag2_compression::rosbag2_compression
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  ${std_srvs_TARGETS})
if(rosbag2_cpp_VERSION VERSION_LESS 0.22)
  target_link_libraries(${PROJECT_NAME}_component PUBLIC rosbag2_transport::rosbag2_transport)
endif()
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "rosbag_recorder::RecorderNode"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR flux::Executor)
# The generated main includes only rclcpp, and an rclcpp executor delivers no Flux frame.
target_compile_options(${PROJECT_NAME}_node PRIVATE -include flux/executor.hpp)
target_link_libraries(${PROJECT_NAME}_node flux_cpp::flux_cpp)

install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(sensor_msgs REQUIRED)

  ament_add_gtest(test_core
    test/test_catalog_diff.cpp
    test/test_config.cpp
    test/test_disk_watch.cpp
    test/test_preflight.cpp
    test/test_selection.cpp
    test/test_session_info.cpp
    test/test_start_gate.cpp
    test/test_stream.cpp
    test/test_summary.cpp)
  target_compile_definitions(test_core PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  target_include_directories(test_core PRIVATE src)
  target_link_libraries(test_core ${PROJECT_NAME}_core)

  add_executable(many_publishers test/many_publishers.cpp)
  target_link_libraries(many_publishers rclcpp::rclcpp ${sensor_msgs_TARGETS})
  install(TARGETS many_publishers DESTINATION lib/${PROJECT_NAME})

  add_launch_test(test/test_scale.launch.py
    TIMEOUT 600
    ENV ROS_DOMAIN_ID=89)
  add_launch_test(test/test_record.launch.py
    TIMEOUT 180
    ENV ROS_DOMAIN_ID=87 TEST_DATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/data)
  add_launch_test(test/test_lifecycle.launch.py
    TIMEOUT 120
    ENV ROS_DOMAIN_ID=91 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)
  add_launch_test(test/test_selection.launch.py
    TIMEOUT 120
    ENV ROS_DOMAIN_ID=95 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST)
endif()

ament_package()
