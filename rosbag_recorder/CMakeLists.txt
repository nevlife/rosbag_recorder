cmake_minimum_required(VERSION 3.16)
project(rosbag_recorder)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rmw REQUIRED)
find_package(rosbag2_compression REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(rosbag2_storage REQUIRED)
find_package(rosbag2_transport REQUIRED)
find_package(rosbag_recorder_checks REQUIRED)
find_package(rosbag_recorder_msgs REQUIRED)
find_package(rosbag_recorder_source REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME}_core SHARED src/config.cpp src/session_info.cpp src/stream.cpp)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME}_core PUBLIC
  rclcpp::rclcpp
  rosbag_recorder_checks::rosbag_recorder_checks
  ${rosbag_recorder_msgs_TARGETS}
  ${tf2_msgs_TARGETS}
  yaml-cpp
  OpenSSL::Crypto)

add_library(${PROJECT_NAME}_component SHARED src/recorder_node.cpp)
target_compile_definitions(${PROJECT_NAME}_component PRIVATE
  ROSBAG_RECORDER_ROSBAG2_MINOR=${rosbag2_cpp_VERSION_MINOR})
target_link_libraries(${PROJECT_NAME}_component PUBLIC
  ${PROJECT_NAME}_core
  pluginlib::pluginlib
  rclcpp_components::component
  rmw::rmw
  rosbag2_compression::rosbag2_compression
  rosbag2_cpp::rosbag2_cpp
  rosbag2_storage::rosbag2_storage
  rosbag_recorder_source::rosbag_recorder_source
  ${std_srvs_TARGETS})
if(rosbag2_cpp_VERSION VERSION_LESS 0.22)
  target_link_libraries(${PROJECT_NAME}_component PUBLIC rosbag2_transport::rosbag2_transport)
endif()
rclcpp_components_register_node(${PROJECT_NAME}_component
  PLUGIN "rosbag_recorder::RecorderNode"
  EXECUTABLE ${PROJECT_NAME}_node
  EXECUTOR MultiThreadedExecutor)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME}_core ${PROJECT_NAME}_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(std_msgs REQUIRED)

  ament_add_gtest(test_core test/test_config.cpp test/test_stream.cpp)
  target_compile_definitions(test_core PRIVATE TEST_DATA_DIR="${CMAKE_CURRENT_SOURCE_DIR}/test/data")
  target_link_libraries(test_core ${PROJECT_NAME}_core ${std_msgs_TARGETS})

  add_library(rosbag_recorder_counter_source SHARED test/counter_source.cpp)
  target_link_libraries(rosbag_recorder_counter_source
    pluginlib::pluginlib
    rclcpp::rclcpp
    rosbag_recorder_source::rosbag_recorder_source
    ${sensor_msgs_TARGETS}
    ${std_msgs_TARGETS}
    yaml-cpp)
  install(TARGETS rosbag_recorder_counter_source LIBRARY DESTINATION lib)
  pluginlib_export_plugin_description_file(rosbag_recorder_source test/counter_source.xml)

  add_executable(many_publishers test/many_publishers.cpp)
  target_link_libraries(many_publishers rclcpp::rclcpp ${sensor_msgs_TARGETS})
  install(TARGETS many_publishers DESTINATION lib/${PROJECT_NAME})

  add_launch_test(test/test_scale.launch.py
    TIMEOUT 600
    ENV ROS_DOMAIN_ID=89)
  add_launch_test(test/test_record.launch.py
    TIMEOUT 180
    ENV ROS_DOMAIN_ID=87 TEST_DATA_DIR=${CMAKE_CURRENT_SOURCE_DIR}/test/data)
endif()

ament_export_dependencies(rclcpp rosbag_recorder_checks rosbag_recorder_msgs tf2_msgs yaml-cpp)
ament_package()
