#include <cmath>
#include <limits>
#include <memory>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = rclcpp::Node::make_shared(
    "real_hover_calibration",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  double candidate_x = 0.343;
  double candidate_y = 0.132;
  node->get_parameter("candidate_x", candidate_x);
  node->get_parameter("candidate_y", candidate_y);

  bool execute = false;
  node->get_parameter("execute", execute);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
  move_group.setPoseReferenceFrame("world");
  move_group.setEndEffectorLink("tool0");

  const auto current_state = move_group.getCurrentState(5.0);
  if (!current_state) {
    RCLCPP_ERROR(node->get_logger(), "CURRENT_STATE FAIL: no plan or motion was attempted.");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  const auto current_pose = move_group.getCurrentPose("tool0");
  if (current_pose.header.frame_id.empty()) {
    RCLCPP_ERROR(node->get_logger(), "CURRENT_POSE FAIL: no plan or motion was attempted.");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  auto hover_target = current_pose;
  hover_target.header.stamp = node->now();
  hover_target.header.frame_id = "world";

  hover_target.pose.position.x = candidate_x;
  hover_target.pose.position.y = candidate_y;
  hover_target.pose.position.z = 0.267;
  hover_target.pose.orientation.x = -1;
  hover_target.pose.orientation.y = 0;
  hover_target.pose.orientation.z = 0;
  hover_target.pose.orientation.w = 0;

  if (!std::isfinite(hover_target.pose.position.x) ||
    !std::isfinite(hover_target.pose.position.y))
  {
    RCLCPP_ERROR(node->get_logger(), "TARGET FAIL: candidate x/y must be finite; no plan or motion was attempted.");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "HOVER_TARGET: frame=%s, current=(%.6f, %.6f, %.6f), target=(%.6f, %.6f, %.6f).",
    hover_target.header.frame_id.c_str(), current_pose.pose.position.x,
    current_pose.pose.position.y, current_pose.pose.position.z,
    hover_target.pose.position.x, hover_target.pose.position.y,
    hover_target.pose.position.z);

  move_group.setPlanningPipelineId("ompl");
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(1);
  move_group.setMaxVelocityScalingFactor(0.01);
  move_group.setMaxAccelerationScalingFactor(0.01);
  move_group.setStartState(*current_state);
  move_group.setPoseTarget(hover_target, "tool0");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "REAL_HOVER_PLAN FAIL: no trajectory was executed.");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  const auto & joint_trajectory = plan.trajectory_.joint_trajectory;
  if (joint_trajectory.points.empty()) {
    RCLCPP_ERROR(node->get_logger(), "TRAJECTORY FAIL: no points; no motion was attempted.");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  const auto & start_positions = joint_trajectory.points.front().positions;
  const auto & end_positions = joint_trajectory.points.back().positions;
  if (joint_trajectory.joint_names.size() != start_positions.size() ||
    start_positions.size() != end_positions.size())
  {
    RCLCPP_ERROR(node->get_logger(), "TRAJECTORY FAIL: joint arrays are inconsistent; no motion was attempted.");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  for (std::size_t i = 0; i < joint_trajectory.joint_names.size(); ++i) {
    const double delta = end_positions[i] - start_positions[i];
    RCLCPP_INFO(
      node->get_logger(), "JOINT_DELTA: %s start=%.6f end=%.6f delta=%.6f rad.",
      joint_trajectory.joint_names[i].c_str(), start_positions[i], end_positions[i], delta);
  }

  if (!execute)
  {
    RCLCPP_INFO(
      node->get_logger(),
      "REAL_HOVER_PLAN_ONLY PASS: candidate XY hover is plannable; execution remains locked.");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 0;
  }

  RCLCPP_WARN(
    node->get_logger(),
    "REAL_HOVER_EXECUTE: explicit execute gate enabled; starting reviewed trajectory.");
  const auto execute_result = move_group.execute(plan);
  if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(
      node->get_logger(),
      "REAL_HOVER_EXECUTION FAIL: controller execution did not complete successfully.");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "REAL_HOVER_EXECUTION PASS");
  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}
