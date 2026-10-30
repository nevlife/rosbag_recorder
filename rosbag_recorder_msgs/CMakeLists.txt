cmake_minimum_required(VERSION 3.16)
project(rosbag_recorder_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/AvailableTopic.msg
  msg/TopicSelection.msg
  msg/CatalogDiffItem.msg
  msg/CheckResult.msg
  msg/FileSnapshot.msg
  msg/Health.msg
  msg/Marker.msg
  msg/Session.msg
  msg/TopicHealth.msg
  srv/CatalogDiff.srv
  srv/ListTopics.srv
  srv/SetTopics.srv
  srv/Mark.srv
  srv/Select.srv
  srv/Start.srv
  srv/Status.srv
  srv/Stop.srv
  DEPENDENCIES builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
