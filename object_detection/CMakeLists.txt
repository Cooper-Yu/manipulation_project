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
install(PROGRAMS object_detection/object_detection.py DESTINATION lib/${PROJECT_NAME} RENAME object_detection)
install(DIRECTORY launch rviz DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/check_pcl.sh scripts/prepare_real_perception.sh scripts/probe_pointcloud.py DESTINATION share/${PROJECT_NAME}/scripts)
ament_export_dependencies(rosidl_default_runtime geometry_msgs)
ament_package()





