#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  const auto node = rclcpp::Node::make_shared(
    "real_arm_lift_test",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  bool execute = false;
  double lift_distance = 0.020;
  node->get_parameter("execute", execute);
  node->get_parameter("lift_distance", lift_distance);

  if (!std::isfinite(lift_distance) || lift_distance <= 0.0 || lift_distance > 0.050) {
    RCLCPP_ERROR(
      node->get_logger(),
      "CONFIG FAIL: lift_distance must be within (0.0, 0.050] m; no motion was attempted.");
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
  move_group.setPoseReferenceFrame("world");
  move_group.setEndEffectorLink("tool0");

  const auto current_state = move_group.getCurrentState(5.0);
  if (!current_state) {
    RCLCPP_ERROR(
      node->get_logger(),
      "CURRENT_STATE FAIL: no fresh RobotState was received; no planning or motion was attempted.");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  const auto start_pose = move_group.getCurrentPose("tool0");
  if (start_pose.header.frame_id.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "CURRENT_POSE FAIL: tool0 pose has no frame; no planning or motion was attempted.");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  auto target_pose = start_pose;
  target_pose.header.stamp = node->now();
  target_pose.pose.position.z += lift_distance;

  RCLCPP_INFO(
    node->get_logger(),
    "LIFT_TARGET: frame=%s, start_z=%.6f m, target_z=%.6f m, delta=%.3f mm.",
    target_pose.header.frame_id.c_str(), start_pose.pose.position.z,
    target_pose.pose.position.z, lift_distance * 1000.0);

  move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
  move_group.setPlannerId("LIN");
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(1);
  move_group.setMaxVelocityScalingFactor(0.01);
  move_group.setMaxAccelerationScalingFactor(0.01);
  move_group.setStartState(*current_state);
  move_group.setPoseTarget(target_pose, "tool0");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto plan_result = move_group.plan(plan);
  const bool plan_success =
    plan_result == moveit::core::MoveItErrorCode::SUCCESS;

  if (!plan_success) {
    RCLCPP_ERROR(
      node->get_logger(),
      "REAL_ARM_LIFT_PLAN FAIL: no trajectory was executed.");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  if (!execute) {
    RCLCPP_INFO(
      node->get_logger(),
      "REAL_ARM_LIFT_PLAN_ONLY PASS: 20 mm-class LIN lift is plannable; execution remains locked."
      " Re-run with execute:=true only after RViz trajectory inspection and explicit authorization.");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
  }

  RCLCPP_WARN(
    node->get_logger(),
    "REAL_ARM_LIFT_EXECUTE: explicit execute gate is enabled; starting the reviewed low-speed trajectory.");
  const auto execute_result = move_group.execute(plan);
  if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(
      node->get_logger(),
      "REAL_ARM_LIFT_EXECUTION FAIL: controller execution did not complete successfully.");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::sleep_for(std::chrono::milliseconds(500));
  const auto final_pose = move_group.getCurrentPose("tool0");
  const double observed_delta =
    final_pose.pose.position.z - start_pose.pose.position.z;
  const double error = std::abs(observed_delta - lift_distance);

  if (error > 0.005) {
    RCLCPP_ERROR(
      node->get_logger(),
      "REAL_ARM_LIFT_VERIFICATION FAIL: observed_delta=%.6f m, expected=%.6f m, error=%.6f m.",
      observed_delta, lift_distance, error);
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "REAL_ARM_LIFT_EXECUTION PASS: observed tool0 z delta=%.6f m.", observed_delta);

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
