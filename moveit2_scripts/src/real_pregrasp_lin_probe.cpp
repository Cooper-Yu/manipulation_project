#include <algorithm>
#include <cmath>
#include <memory>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = rclcpp::Node::make_shared(
    "real_pregrasp_lin_probe",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

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
    RCLCPP_ERROR(node->get_logger(), "ROBOT_MODEL FAIL: no planning or motion was attempted.");
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

  geometry_msgs::msg::PoseStamped pregrasp_target;
  pregrasp_target.header.frame_id = "world";
  pregrasp_target.header.stamp = node->now();
  pregrasp_target.pose.position.x = 0.343;
  pregrasp_target.pose.position.y = 0.132;
  pregrasp_target.pose.position.z = 0.267;
  pregrasp_target.pose.orientation.x = -1.0;
  pregrasp_target.pose.orientation.y = 0.0;
  pregrasp_target.pose.orientation.z = 0.0;
  pregrasp_target.pose.orientation.w = 0.0;

  move_group.setPlanningPipelineId("ompl");
  move_group.setPlannerId("");
  move_group.setStartState(real_home_state);
  move_group.setPoseTarget(pregrasp_target, "tool0");

  moveit::planning_interface::MoveGroupInterface::Plan pregrasp_plan;
  const auto pregrasp_plan_result = move_group.plan(pregrasp_plan);
  const auto & pregrasp_trajectory = pregrasp_plan.trajectory_.joint_trajectory;
  if (
    pregrasp_plan_result != moveit::core::MoveItErrorCode::SUCCESS ||
    pregrasp_trajectory.points.empty())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "PREGRASP_PLAN FAIL: OMPL did not produce a nonempty trajectory; no motion was attempted.");
    stop();
    return 1;
  }

  const auto & joint_names = pregrasp_trajectory.joint_names;
  const auto & endpoint_positions = pregrasp_trajectory.points.back().positions;
  if (joint_names.size() != endpoint_positions.size()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "PREGRASP_TRAJECTORY FAIL: endpoint joint arrays are inconsistent; no motion was attempted.");
    stop();
    return 1;
  }

  const auto & points = pregrasp_trajectory.points;
  std::vector<double> max_adjacent_changes(joint_names.size(), 0.0);
  std::vector<double> min_positions = points.front().positions;
  std::vector<double> max_positions = points.front().positions;
  for (std::size_t i = 1; i < points.size(); ++i) {
    if (
      points[i].positions.size() != joint_names.size() ||
      points[i - 1].positions.size() != joint_names.size())
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "PREGRASP_TRAJECTORY FAIL: waypoint joint arrays are inconsistent; no motion was attempted.");
      stop();
      return 1;
    }

    for (std::size_t j = 0; j < joint_names.size(); ++j) {
      max_adjacent_changes[j] = std::max(
        max_adjacent_changes[j],
        std::abs(points[i].positions[j] - points[i - 1].positions[j]));
      min_positions[j] = std::min(min_positions[j], points[i].positions[j]);
      max_positions[j] = std::max(max_positions[j], points[i].positions[j]);
    }
  }

  for (std::size_t j = 0; j < joint_names.size(); ++j) {
    RCLCPP_INFO(
      node->get_logger(),
      "PREGRASP_MAX_ADJACENT_CHANGE: %s %.6f rad.",
      joint_names[j].c_str(), max_adjacent_changes[j]);
    RCLCPP_INFO(
      node->get_logger(),
      "PREGRASP_JOINT_EXCURSION: %s min=%.6f max=%.6f excursion=%.6f rad.",
      joint_names[j].c_str(), min_positions[j], max_positions[j],
      max_positions[j] - min_positions[j]);
    RCLCPP_INFO(
      node->get_logger(),
      "PREGRASP_ENDPOINT_DELTA: %s %.6f rad.",
      joint_names[j].c_str(),
      points.back().positions[j] - points.front().positions[j]);
  }

  moveit::core::RobotState lin_start_state(real_home_state);
  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    lin_start_state.setVariablePosition(joint_names[i], endpoint_positions[i]);
  }
  lin_start_state.update();

  geometry_msgs::msg::PoseStamped lin_target = pregrasp_target;
  lin_target.header.stamp = node->now();
  lin_target.pose.position.z = 0.247;

  move_group.setStartState(lin_start_state);
  move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
  move_group.setPlannerId("LIN");
  move_group.setPoseTarget(lin_target, "tool0");

  moveit::planning_interface::MoveGroupInterface::Plan lin_plan;
  const auto lin_plan_result = move_group.plan(lin_plan);
  const auto & lin_trajectory = lin_plan.trajectory_.joint_trajectory;
  if (
    lin_plan_result != moveit::core::MoveItErrorCode::SUCCESS ||
    lin_trajectory.points.empty())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "LIN_PLAN FAIL: Pilz did not produce a nonempty 20 mm trajectory; no motion was attempted.");
    stop();
    return 1;
  }

  const auto & lin_start_positions = lin_trajectory.points.front().positions;
  if (lin_trajectory.joint_names.size() != lin_start_positions.size()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "LIN_TRAJECTORY FAIL: recorded start arrays are inconsistent; no motion was attempted.");
    stop();
    return 1;
  }

  constexpr double lin_start_tolerance = 0.01;
  bool lin_start_matches = true;
  for (std::size_t i = 0; i < lin_trajectory.joint_names.size(); ++i) {
    const double lin_start_error = std::abs(
      lin_start_positions[i] -
      lin_start_state.getVariablePosition(lin_trajectory.joint_names[i]));
    lin_start_matches =
      lin_start_matches && (lin_start_error <= lin_start_tolerance);
    RCLCPP_INFO(
      node->get_logger(),
      "LIN_START_CHECK: %s recorded=%.6f expected=%.6f error=%.6f rad.",
      lin_trajectory.joint_names[i].c_str(), lin_start_positions[i],
      lin_start_state.getVariablePosition(lin_trajectory.joint_names[i]),
      lin_start_error);
  }

  if (!lin_start_matches) {
    RCLCPP_ERROR(
      node->get_logger(),
      "LIN_START FAIL: recorded trajectory start does not match the OMPL endpoint; no motion was attempted.");
    stop();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "REAL_PREGRASP_LIN_PROBE PASS: OMPL pre-grasp and 20 mm Pilz LIN trajectories are plannable and start-continuous; zero Execute calls, no motion attempted.");
  stop();
  return 0;
}
