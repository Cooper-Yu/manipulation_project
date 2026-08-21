#include <chrono>
#include <cmath>
#include <memory>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = rclcpp::Node::make_shared(
    "real_high_pregrasp_lin_probe",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  const auto display_publisher =
    node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path", rclcpp::QoS(1).transient_local().reliable());

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});

  const auto stop = [&executor, &spin_thread]() {
      executor.cancel();
      spin_thread.join();
      rclcpp::shutdown();
    };

  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
  move_group.setPoseReferenceFrame("world");
  move_group.setEndEffectorLink("tool0");
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(1);
  move_group.setMaxVelocityScalingFactor(0.01);
  move_group.setMaxAccelerationScalingFactor(0.01);

  const auto robot_model = move_group.getRobotModel();
  if (!robot_model) {
    RCLCPP_ERROR(
      node->get_logger(),
      "ROBOT_MODEL FAIL: no planning or motion was attempted.");
    stop();
    return 1;
  }

  moveit::core::RobotState real_home_state(robot_model);
  real_home_state.setToDefaultValues();
  const auto real_home_joint_values = move_group.getNamedTargetValues("real_home_01");
  if (real_home_joint_values.size() != 6U) {
    RCLCPP_ERROR(
      node->get_logger(),
      "REAL_HOME_01 FAIL: expected six named arm joints, received %zu; no planning or motion was attempted.",
      real_home_joint_values.size());
    stop();
    return 1;
  }
  for (const auto & [joint_name, position] : real_home_joint_values) {
    real_home_state.setVariablePosition(joint_name, position);
  }
  real_home_state.update();

  geometry_msgs::msg::PoseStamped anchor_target;
  anchor_target.header.frame_id = "world";
  anchor_target.header.stamp = node->now();
  anchor_target.pose.position.x = 0.346;
  anchor_target.pose.position.y = 0.131;
  anchor_target.pose.position.z = 0.238;

  constexpr double observed_qx = 0.997;
  constexpr double observed_qy = 0.072;
  constexpr double observed_qz = 0.009;
  constexpr double observed_qw = 0.001;
  const double quaternion_norm = std::sqrt(
    observed_qx * observed_qx + observed_qy * observed_qy +
    observed_qz * observed_qz + observed_qw * observed_qw);
  if (!std::isfinite(quaternion_norm) || quaternion_norm < 1e-9) {
    RCLCPP_ERROR(
      node->get_logger(),
      "ANCHOR_ORIENTATION FAIL: observed quaternion is invalid; no planning or motion was attempted.");
    stop();
    return 1;
  }
  anchor_target.pose.orientation.x = observed_qx / quaternion_norm;
  anchor_target.pose.orientation.y = observed_qy / quaternion_norm;
  anchor_target.pose.orientation.z = observed_qz / quaternion_norm;
  anchor_target.pose.orientation.w = observed_qw / quaternion_norm;

  geometry_msgs::msg::PoseStamped high_pregrasp_target = anchor_target;
  high_pregrasp_target.header.stamp = node->now();
  high_pregrasp_target.pose.position.z += 0.030;

  RCLCPP_INFO(
    node->get_logger(),
    "CALIBRATION_ANCHOR: frame=world pose=[%.6f, %.6f, %.6f] quaternion=[%.6f, %.6f, %.6f, %.6f].",
    anchor_target.pose.position.x, anchor_target.pose.position.y,
    anchor_target.pose.position.z, anchor_target.pose.orientation.x,
    anchor_target.pose.orientation.y, anchor_target.pose.orientation.z,
    anchor_target.pose.orientation.w);
  RCLCPP_INFO(
    node->get_logger(),
    "HIGH_PREGRASP_TARGET: frame=world pose=[%.6f, %.6f, %.6f]; anchor_z_offset=+30.000 mm.",
    high_pregrasp_target.pose.position.x, high_pregrasp_target.pose.position.y,
    high_pregrasp_target.pose.position.z);

  move_group.setPlanningPipelineId("ompl");
  move_group.setPlannerId("");
  move_group.setStartState(real_home_state);
  move_group.setPoseTarget(high_pregrasp_target, "tool0");

  moveit::planning_interface::MoveGroupInterface::Plan high_pregrasp_plan;
  const auto high_pregrasp_result = move_group.plan(high_pregrasp_plan);
  const auto & high_pregrasp_trajectory =
    high_pregrasp_plan.trajectory_.joint_trajectory;
  if (
    high_pregrasp_result != moveit::core::MoveItErrorCode::SUCCESS ||
    high_pregrasp_trajectory.points.empty())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "HIGH_PREGRASP_PLAN FAIL: OMPL did not produce a nonempty trajectory; no motion was attempted.");
    stop();
    return 1;
  }

  const auto & joint_names = high_pregrasp_trajectory.joint_names;
  const auto & endpoint_positions =
    high_pregrasp_trajectory.points.back().positions;
  if (joint_names.size() != 6U || joint_names.size() != endpoint_positions.size()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "HIGH_PREGRASP_TRAJECTORY FAIL: expected six consistent endpoint joints; no motion was attempted.");
    stop();
    return 1;
  }

  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    RCLCPP_INFO(
      node->get_logger(),
      "HIGH_PREGRASP_ENDPOINT_JOINT: %s=%.9f rad.",
      joint_names[i].c_str(), endpoint_positions[i]);
  }

  moveit::core::RobotState lin_start_state(real_home_state);
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    lin_start_state.setVariablePosition(joint_names[i], endpoint_positions[i]);
  }
  lin_start_state.update();

  move_group.setStartState(lin_start_state);
  move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
  move_group.setPlannerId("LIN");
  move_group.setPoseTarget(anchor_target, "tool0");

  moveit::planning_interface::MoveGroupInterface::Plan lin_plan;
  const auto lin_result = move_group.plan(lin_plan);
  const auto & lin_trajectory = lin_plan.trajectory_.joint_trajectory;
  if (
    lin_result != moveit::core::MoveItErrorCode::SUCCESS ||
    lin_trajectory.points.empty())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "ANCHOR_LIN_PLAN FAIL: Pilz did not produce a nonempty 30 mm descent trajectory; no motion was attempted.");
    stop();
    return 1;
  }

  const auto & lin_start_positions = lin_trajectory.points.front().positions;
  if (lin_trajectory.joint_names.size() != lin_start_positions.size()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "ANCHOR_LIN_TRAJECTORY FAIL: recorded start arrays are inconsistent; no motion was attempted.");
    stop();
    return 1;
  }

  constexpr double lin_start_tolerance = 0.01;
  bool lin_start_matches = true;
  for (std::size_t i = 0; i < lin_trajectory.joint_names.size(); ++i) {
    const double expected_position =
      lin_start_state.getVariablePosition(lin_trajectory.joint_names[i]);
    const double lin_start_error =
      std::abs(lin_start_positions[i] - expected_position);
    lin_start_matches =
      lin_start_matches && std::isfinite(lin_start_error) &&
      (lin_start_error <= lin_start_tolerance);
    RCLCPP_INFO(
      node->get_logger(),
      "ANCHOR_LIN_START_CHECK: %s recorded=%.6f expected=%.6f error=%.6f rad.",
      lin_trajectory.joint_names[i].c_str(), lin_start_positions[i],
      expected_position, lin_start_error);
  }

  if (!lin_start_matches) {
    RCLCPP_ERROR(
      node->get_logger(),
      "ANCHOR_LIN_START FAIL: recorded LIN start does not match the OMPL endpoint; no motion was attempted.");
    stop();
    return 1;
  }

  moveit_msgs::msg::DisplayTrajectory display_trajectory;
  display_trajectory.model_id = robot_model->getName();
  display_trajectory.trajectory_start = high_pregrasp_plan.start_state_;
  display_trajectory.trajectory.push_back(high_pregrasp_plan.trajectory_);
  display_trajectory.trajectory.push_back(lin_plan.trajectory_);
  display_publisher->publish(display_trajectory);

  RCLCPP_INFO(
    node->get_logger(),
    "HIGH_PREGRASP_DISPLAY_PUBLISHED: retained OMPL approach and retained 30 mm Pilz LIN descent sent to /display_planned_path.");
  RCLCPP_INFO(
    node->get_logger(),
    "REAL_HIGH_PREGRASP_LIN_PROBE PASS: both trajectories are plannable and start-continuous; zero Execute calls, no motion attempted.");

  std::this_thread::sleep_for(std::chrono::seconds(5));
  stop();
  return 0;
}
