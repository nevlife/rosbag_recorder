cmake_minimum_required(VERSION 3.16)
project(rosbag_recorder_checks)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rcpputils REQUIRED)
find_package(rosidl_runtime_c REQUIRED)
find_package(rosidl_typesupport_introspection_cpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED src/message.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_link_libraries(${PROJECT_NAME} PUBLIC
  rclcpp::rclcpp
  rcpputils::rcpputils
  rosidl_runtime_c::rosidl_runtime_c
  rosidl_typesupport_introspection_cpp::rosidl_typesupport_introspection_cpp)

add_library(rosbag_recorder_builtin_checks SHARED src/builtin_checks.cpp)
target_link_libraries(rosbag_recorder_builtin_checks PUBLIC
  ${PROJECT_NAME}
  pluginlib::pluginlib
  ${sensor_msgs_TARGETS}
  ${tf2_msgs_TARGETS})
pluginlib_export_plugin_description_file(${PROJECT_NAME} plugins.xml)

install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} rosbag_recorder_builtin_checks EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  find_package(std_msgs REQUIRED)
  ament_add_gtest(test_checks test/test_cdr.cpp test/test_type_info.cpp test/test_checks.cpp)
  target_include_directories(test_checks PRIVATE src)
  target_link_libraries(test_checks rosbag_recorder_builtin_checks ${std_msgs_TARGETS})
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(pluginlib rclcpp rcpputils rosidl_runtime_c
  rosidl_typesupport_introspection_cpp)
ament_package()
