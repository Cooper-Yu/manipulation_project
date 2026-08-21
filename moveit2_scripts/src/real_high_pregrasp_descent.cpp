#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

enum class GateState
{
  LOCKED,
  READY,
  CONSUMED,
};

using Plan = moveit::planning_interface::MoveGroupInterface::Plan;

bool wait_for_gate(
  const rclcpp::Node::SharedPtr & node,
  std::atomic<GateState> & gate,
  const std::string & label)
{
  gate.store(GateState::READY);
  const auto deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(60);
  RCLCPP_WARN(
    node->get_logger(),
    "%s_AUTHORIZATION_READY: exact displayed retained Plan available for 60 seconds.",
    label.c_str());

  while (
    gate.load() != GateState::CONSUMED &&
    std::chrono::steady_clock::now() < deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  GateState expected = GateState::READY;
  if (gate.compare_exchange_strong(expected, GateState::LOCKED)) {
    RCLCPP_WARN(
      node->get_logger(),
      "%s_AUTHORIZATION_TIMEOUT: retained Plan must be invalidated; no motion was attempted for this stage.",
      label.c_str());
    return false;
  }

  return gate.load() == GateState::CONSUMED;
}

bool state_matches_plan_start(
  const rclcpp::Node::SharedPtr & node,
  const moveit::core::RobotStatePtr & actual_state,
  const Plan & plan,
  const std::string & label)
{
  if (!actual_state) {
    RCLCPP_ERROR(
      node->get_logger(),
      "%s_FRESH_STATE FAIL: current state unavailable.", label.c_str());
    return false;
  }

  const auto & trajectory = plan.trajectory_.joint_trajectory;
  if (trajectory.points.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "%s_FRESH_STATE FAIL: retained trajectory is empty.", label.c_str());
    return false;
  }

  const auto & start_positions = trajectory.points.front().positions;
  if (
    trajectory.joint_names.size() != 6U ||
    trajectory.joint_names.size() != start_positions.size())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "%s_FRESH_STATE FAIL: retained start arrays are inconsistent.", label.c_str());
    return false;
  }

  constexpr double start_tolerance = 0.01;
  bool matches = true;
  for (std::size_t i = 0; i < trajectory.joint_names.size(); ++i) {
    const double actual_position =
      actual_state->getVariablePosition(trajectory.joint_names[i]);
    const double error = std::abs(actual_position - start_positions[i]);
    matches =
      matches && std::isfinite(actual_position) && std::isfinite(error) &&
      (error <= start_tolerance);
    RCLCPP_INFO(
      node->get_logger(),
      "%s_FRESH_STATE_CHECK: %s retained=%.6f actual=%.6f error=%.6f rad.",
      label.c_str(), trajectory.joint_names[i].c_str(), start_positions[i],
      actual_position, error);
  }
  return matches;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = rclcpp::Node::make_shared(
    "real_high_pregrasp_descent",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  const auto display_publisher =
    node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path", rclcpp::QoS(1).transient_local().reliable());

  std::atomic<GateState> approach_gate{GateState::LOCKED};
  std::atomic<GateState> descent_gate{GateState::LOCKED};
  const auto approach_service = node->create_service<std_srvs::srv::Trigger>(
    "/real_high_pregrasp_descent/authorize_approach",
    [&approach_gate](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      GateState expected = GateState::READY;
      response->success =
        approach_gate.compare_exchange_strong(expected, GateState::CONSUMED);
      response->message = response->success ?
        "Approach authorization accepted; validating the retained Plan start." :
        "Approach authorization rejected: gate locked or already consumed.";
    });
  const auto descent_service = node->create_service<std_srvs::srv::Trigger>(
    "/real_high_pregrasp_descent/authorize_descent",
    [&descent_gate](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      GateState expected = GateState::READY;
      response->success =
        descent_gate.compare_exchange_strong(expected, GateState::CONSUMED);
      response->message = response->success ?
        "Descent authorization accepted; validating the retained LIN start." :
        "Descent authorization rejected: gate locked or already consumed.";
    });
  (void)approach_service;
  (void)descent_service;

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
    RCLCPP_ERROR(node->get_logger(), "ROBOT_MODEL FAIL: no motion was attempted.");
    stop();
    return 1;
  }

  moveit::core::RobotState real_home_state(robot_model);
  real_home_state.setToDefaultValues();
  const auto real_home_joint_values = move_group.getNamedTargetValues("real_home_01");
  if (real_home_joint_values.size() != 6U) {
    RCLCPP_ERROR(
      node->get_logger(),
      "REAL_HOME_01 FAIL: expected six named arm joints, received %zu; no motion was attempted.",
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
      "ANCHOR_ORIENTATION FAIL: observed quaternion is invalid; no motion was attempted.");
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

  geometry_msgs::msg::PoseStamped grasp_candidate_target = high_pregrasp_target;
  grasp_candidate_target.header.stamp = node->now();
  grasp_candidate_target.pose.position.z -= 0.060;

  RCLCPP_INFO(
    node->get_logger(),
    "HIGH_PREGRASP_TARGET: frame=world pose=[%.6f, %.6f, %.6f].",
    high_pregrasp_target.pose.position.x,
    high_pregrasp_target.pose.position.y,
    high_pregrasp_target.pose.position.z);
  RCLCPP_INFO(
    node->get_logger(),
    "GRASP_CANDIDATE_TARGET: frame=world pose=[%.6f, %.6f, %.6f]; descent=-60.000 mm.",
    grasp_candidate_target.pose.position.x,
    grasp_candidate_target.pose.position.y,
    grasp_candidate_target.pose.position.z);

  move_group.setPlanningPipelineId("ompl");
  move_group.setPlannerId("");
  move_group.setStartState(real_home_state);
  move_group.setPoseTarget(high_pregrasp_target, "tool0");

  Plan approach_plan;
  const auto approach_plan_result = move_group.plan(approach_plan);
  const auto & approach_trajectory = approach_plan.trajectory_.joint_trajectory;
  if (
    approach_plan_result != moveit::core::MoveItErrorCode::SUCCESS ||
    approach_trajectory.points.empty())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "HIGH_PREGRASP_PLAN FAIL: OMPL produced no retained Plan; no motion was attempted.");
    stop();
    return 1;
  }

  const auto & joint_names = approach_trajectory.joint_names;
  const auto & endpoint_positions = approach_trajectory.points.back().positions;
  if (joint_names.size() != 6U || joint_names.size() != endpoint_positions.size()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "HIGH_PREGRASP_TRAJECTORY FAIL: endpoint arrays are inconsistent; no motion was attempted.");
    stop();
    return 1;
  }

  for (std::size_t i = 0; i < joint_names.size(); ++i) {
    RCLCPP_INFO(
      node->get_logger(), "HIGH_PREGRASP_ENDPOINT_JOINT: %s=%.9f rad.",
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
  move_group.setPoseTarget(grasp_candidate_target, "tool0");

  Plan descent_plan;
  const auto descent_plan_result = move_group.plan(descent_plan);
  const auto & descent_trajectory = descent_plan.trajectory_.joint_trajectory;
  if (
    descent_plan_result != moveit::core::MoveItErrorCode::SUCCESS ||
    descent_trajectory.points.empty())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "GRASP_CANDIDATE_LIN_PLAN FAIL: Pilz produced no retained 60 mm descent Plan; no motion was attempted.");
    stop();
    return 1;
  }

  const auto & lin_start_positions = descent_trajectory.points.front().positions;
  if (descent_trajectory.joint_names.size() != lin_start_positions.size()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "GRASP_CANDIDATE_LIN_TRAJECTORY FAIL: start arrays are inconsistent; no motion was attempted.");
    stop();
    return 1;
  }
  bool lin_start_matches = true;
  for (std::size_t i = 0; i < descent_trajectory.joint_names.size(); ++i) {
    const double expected_position =
      lin_start_state.getVariablePosition(descent_trajectory.joint_names[i]);
    const double error = std::abs(lin_start_positions[i] - expected_position);
    lin_start_matches = lin_start_matches && std::isfinite(error) && error <= 0.01;
    RCLCPP_INFO(
      node->get_logger(),
      "GRASP_CANDIDATE_LIN_START_CHECK: %s recorded=%.6f expected=%.6f error=%.6f rad.",
      descent_trajectory.joint_names[i].c_str(), lin_start_positions[i],
      expected_position, error);
  }
  if (!lin_start_matches) {
    RCLCPP_ERROR(
      node->get_logger(),
      "GRASP_CANDIDATE_LIN_START FAIL: retained LIN start differs from OMPL endpoint; no motion was attempted.");
    stop();
    return 1;
  }

  moveit_msgs::msg::DisplayTrajectory display_trajectory;
  display_trajectory.model_id = robot_model->getName();
  display_trajectory.trajectory_start = approach_plan.start_state_;
  display_trajectory.trajectory.push_back(approach_plan.trajectory_);
  display_trajectory.trajectory.push_back(descent_plan.trajectory_);
  display_publisher->publish(display_trajectory);
  RCLCPP_INFO(
    node->get_logger(),
    "HIGH_PREGRASP_SEQUENCE_DISPLAY_PUBLISHED: exact retained approach and descent Plans sent to RViz; execution gates remain locked.");

  if (!wait_for_gate(node, approach_gate, "APPROACH")) {
    approach_plan.trajectory_.joint_trajectory.points.clear();
    descent_plan.trajectory_.joint_trajectory.points.clear();
    stop();
    return 0;
  }

  const auto fresh_home_state = move_group.getCurrentState(5.0);
  if (!state_matches_plan_start(node, fresh_home_state, approach_plan, "APPROACH")) {
    approach_plan.trajectory_.joint_trajectory.points.clear();
    descent_plan.trajectory_.joint_trajectory.points.clear();
    RCLCPP_ERROR(
      node->get_logger(),
      "APPROACH_STALE_PLAN REJECTED: actual state differs from retained start; no motion was attempted.");
    stop();
    return 1;
  }

  RCLCPP_WARN(
    node->get_logger(),
    "APPROACH_EXECUTE_START: executing the exact retained OMPL Plan at 1%% scaling.");
  const auto approach_result = move_group.execute(approach_plan);
  if (approach_result != moveit::core::MoveItErrorCode::SUCCESS) {
    descent_plan.trajectory_.joint_trajectory.points.clear();
    RCLCPP_ERROR(
      node->get_logger(),
      "APPROACH_EXECUTION FAIL: no retry, replan or descent attempted.");
    stop();
    return 1;
  }
  RCLCPP_INFO(
    node->get_logger(),
    "REAL_HIGH_PREGRASP_APPROACH PASS: retained OMPL Plan completed; descent remains locked.");

  if (!wait_for_gate(node, descent_gate, "DESCENT")) {
    descent_plan.trajectory_.joint_trajectory.points.clear();
    stop();
    return 0;
  }

  const auto fresh_high_state = move_group.getCurrentState(5.0);
  if (!state_matches_plan_start(node, fresh_high_state, descent_plan, "DESCENT")) {
    descent_plan.trajectory_.joint_trajectory.points.clear();
    RCLCPP_ERROR(
      node->get_logger(),
      "DESCENT_STALE_PLAN REJECTED: actual state differs from retained LIN start; no descent was attempted.");
    stop();
    return 1;
  }

  RCLCPP_WARN(
    node->get_logger(),
    "DESCENT_EXECUTE_START: executing the exact retained 60 mm Pilz LIN Plan at 1%% scaling.");
  const auto descent_result = move_group.execute(descent_plan);
  if (descent_result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(
      node->get_logger(),
      "DESCENT_EXECUTION FAIL: no retry, replan, gripper command or return motion attempted.");
    stop();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "REAL_GRASP_CANDIDATE_DESCENT PASS: exact retained 60 mm LIN completed; stopped at the grasp candidate with no gripper command or further motion.");
  stop();
  return 0;
}
