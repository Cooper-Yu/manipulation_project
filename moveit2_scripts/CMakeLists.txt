cmake_minimum_required(VERSION 3.8)
project(moveit2_scripts)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(pick_and_place src/pick_and_place.cpp)
ament_target_dependencies(
  pick_and_place
  geometry_msgs
  moveit_ros_planning_interface
  moveit_msgs
  rclcpp
  tf2
  tf2_geometry_msgs
)

add_executable(real_arm_lift_test src/real_arm_lift_test.cpp)
ament_target_dependencies(
  real_arm_lift_test
  moveit_ros_planning_interface
  rclcpp
)

add_executable(real_hover_calibration src/real_hover_calibration.cpp)
ament_target_dependencies(
  real_hover_calibration
  moveit_ros_planning_interface
  rclcpp
)

add_executable(real_safe_recovery_plan src/real_safe_recovery_plan.cpp)
ament_target_dependencies(
  real_safe_recovery_plan
  moveit_ros_planning_interface
  rclcpp
  std_srvs
)

add_executable(real_pregrasp_lin_probe src/real_pregrasp_lin_probe.cpp)
ament_target_dependencies(
  real_pregrasp_lin_probe
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  std_srvs
)

add_executable(real_high_pregrasp_lin_probe src/real_high_pregrasp_lin_probe.cpp)
ament_target_dependencies(
  real_high_pregrasp_lin_probe
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
)

add_executable(real_high_pregrasp_descent src/real_high_pregrasp_descent.cpp)
ament_target_dependencies(
  real_high_pregrasp_descent
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  std_srvs
)

add_executable(real_pick_place_continuation src/real_pick_place_continuation.cpp)
ament_target_dependencies(
  real_pick_place_continuation
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  rclcpp
  std_srvs
)

install(TARGETS
  pick_and_place
  real_arm_lift_test
  real_hover_calibration
  real_safe_recovery_plan
  real_pregrasp_lin_probe
  real_high_pregrasp_lin_probe
  real_high_pregrasp_descent
  real_pick_place_continuation
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
