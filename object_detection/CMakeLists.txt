cmake_minimum_required(VERSION 3.8)
project(object_detection)
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedSurfaces.msg"
  "msg/DetectedObjects.msg"
  DEPENDENCIES geometry_msgs
)
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
ament_export_dependencies(rosidl_default_runtime geometry_msgs)
ament_package()
