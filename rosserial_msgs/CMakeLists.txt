cmake_minimum_required(VERSION 3.8)
project(rosserial_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(msg_files
  "msg/Log.msg"
  "msg/TopicInfo.msg"
)

set(srv_files
  "srv/RequestParam.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  ADD_LINTER_TESTS
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
