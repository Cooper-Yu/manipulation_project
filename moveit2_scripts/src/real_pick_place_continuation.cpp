#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

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

enum class StageResult
{
  SUCCESS,
  TIMEOUT,
  FAILURE,
};

using MoveGroup = moveit::planning_interface::MoveGroupInterface;
using Plan = MoveGroup::Plan;

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
      "%s_AUTHORIZATION_TIMEOUT: retained sequence invalidated; no later stage was attempted.",
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
      node->get_logger(), "%s_FRESH_STATE FAIL: current state unavailable.",
      label.c_str());
    return false;
  }

  const auto & trajectory = plan.trajectory_.joint_trajectory;
  if (
    trajectory.points.empty() || trajectory.joint_names.empty() ||
    trajectory.joint_names.size() != trajectory.points.front().positions.size())
  {
    RCLCPP_ERROR(
      node->get_logger(), "%s_FRESH_STATE FAIL: retained start arrays are invalid.",
      label.c_str());
    return false;
  }

  constexpr double kStartTolerance = 0.01;
  bool matches = true;
  for (std::size_t i = 0; i < trajectory.joint_names.size(); ++i) {
    const double actual =
      actual_state->getVariablePosition(trajectory.joint_names[i]);
    const double retained = trajectory.points.front().positions[i];
    const double error = std::abs(actual - retained);
    matches = matches && std::isfinite(actual) && std::isfinite(error) &&
      error <= kStartTolerance;
    RCLCPP_INFO(
      node->get_logger(),
      "%s_FRESH_STATE_CHECK: %s retained=%.6f actual=%.6f error=%.6f rad.",
      label.c_str(), trajectory.joint_names[i].c_str(), retained, actual, error);
  }
  return matches;
}

void apply_plan_endpoint(moveit::core::RobotState & state, const Plan & plan)
{
  const auto & trajectory = plan.trajectory_.joint_trajectory;
  const auto & endpoint = trajectory.points.back().positions;
  for (std::size_t i = 0; i < trajectory.joint_names.size(); ++i) {
    state.setVariablePosition(trajectory.joint_names[i], endpoint[i]);
  }
  state.update();
}

bool plan_is_valid(const Plan & plan)
{
  const auto & trajectory = plan.trajectory_.joint_trajectory;
  return !trajectory.joint_names.empty() && !trajectory.points.empty() &&
    trajectory.joint_names.size() == trajectory.points.front().positions.size() &&
    trajectory.joint_names.size() == trajectory.points.back().positions.size();
}

StageResult run_retained_stage(
  const rclcpp::Node::SharedPtr & node,
  MoveGroup & group,
  std::atomic<GateState> & gate,
  const std::string & label,
  Plan & plan)
{
  if (!wait_for_gate(node, gate, label)) {
    plan.trajectory_.joint_trajectory.points.clear();
    return StageResult::TIMEOUT;
  }

  const auto actual_state = group.getCurrentState(5.0);
  if (!state_matches_plan_start(node, actual_state, plan, label)) {
    plan.trajectory_.joint_trajectory.points.clear();
    RCLCPP_ERROR(
      node->get_logger(),
      "%s_STALE_PLAN REJECTED: actual state differs from retained start; no later stage was attempted.",
      label.c_str());
    return StageResult::FAILURE;
  }

  RCLCPP_WARN(
    node->get_logger(), "%s_EXECUTE_START: executing exact retained Plan.",
    label.c_str());
  const auto result = group.execute(plan);
  if (result != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(
      node->get_logger(), "%s_EXECUTION FAIL: no later stage was attempted.",
      label.c_str());
    return StageResult::FAILURE;
  }

  RCLCPP_INFO(
    node->get_logger(), "%s_EXECUTION PASS: retained Plan completed.",
    label.c_str());
  return StageResult::SUCCESS;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = rclcpp::Node::make_shared(
    "real_pick_place_continuation",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  const auto display_publisher =
    node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path", rclcpp::QoS(1).transient_local().reliable());

  std::atomic<GateState> close_gate{GateState::LOCKED};
  std::atomic<GateState> lift_gate{GateState::LOCKED};
  std::atomic<GateState> transfer_gate{GateState::LOCKED};
  std::atomic<GateState> release_gate{GateState::LOCKED};
  std::atomic<GateState> home_gate{GateState::LOCKED};

  const auto make_service = [&node](
      const std::string & name, std::atomic<GateState> & gate,
      const std::string & label)
    {
      return node->create_service<std_srvs::srv::Trigger>(
        name,
        [&gate, label](
          const std::shared_ptr<std_srvs::srv::Trigger::Request>,
          std::shared_ptr<std_srvs::srv::Trigger::Response> response)
        {
          GateState expected = GateState::READY;
          response->success =
            gate.compare_exchange_strong(expected, GateState::CONSUMED);
          response->message = response->success ?
            label + " authorization accepted; validating retained Plan start." :
            label + " authorization rejected: gate locked or already consumed.";
        });
    };

  const auto close_service = make_service(
    "/real_pick_place_continuation/authorize_close", close_gate, "Close");
  const auto lift_service = make_service(
    "/real_pick_place_continuation/authorize_lift", lift_gate, "Lift");
  const auto transfer_service = make_service(
    "/real_pick_place_continuation/authorize_transfer", transfer_gate, "Transfer");
  const auto release_service = make_service(
    "/real_pick_place_continuation/authorize_release", release_gate, "Release");
  const auto home_service = make_service(
    "/real_pick_place_continuation/authorize_home", home_gate, "Home");
  (void)close_service;
  (void)lift_service;
  (void)transfer_service;
  (void)release_service;
  (void)home_service;

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});
  const auto stop = [&executor, &spin_thread]() {
      executor.cancel();
      spin_thread.join();
      rclcpp::shutdown();
    };
  const auto finish_stage = [&stop](StageResult result) {
      if (result == StageResult::SUCCESS) {
        return -1;
      }
      stop();
      return result == StageResult::TIMEOUT ? 0 : 1;
    };

  MoveGroup arm_group(node, "ur_manipulator");
  MoveGroup gripper_group(node, "gripper");
  arm_group.setPoseReferenceFrame("world");
  arm_group.setEndEffectorLink("tool0");
  arm_group.setPlanningTime(5.0);
  arm_group.setNumPlanningAttempts(1);
  arm_group.setMaxVelocityScalingFactor(0.01);
  arm_group.setMaxAccelerationScalingFactor(0.01);
  gripper_group.setPlanningTime(5.0);
  gripper_group.setNumPlanningAttempts(1);
  gripper_group.setMaxVelocityScalingFactor(0.01);
  gripper_group.setMaxAccelerationScalingFactor(0.01);

  const auto robot_model = arm_group.getRobotModel();
  const auto actual_start = arm_group.getCurrentState(5.0);
  if (!robot_model || !actual_start) {
    RCLCPP_ERROR(
      node->get_logger(),
      "INITIAL_STATE FAIL: robot model or current state unavailable; no motion was attempted.");
    stop();
    return 1;
  }

  const std::map<std::string, double> expected_grasp_candidate = {
    {"shoulder_pan_joint", 0.000060223},
    {"shoulder_lift_joint", -1.301308171},
    {"elbow_joint", 1.727110688},
    {"wrist_1_joint", -2.014746805},
    {"wrist_2_joint", -1.571500127},
    {"wrist_3_joint", -1.714953248},
  };
  bool candidate_matches = true;
  for (const auto & [joint_name, expected] : expected_grasp_candidate) {
    const double actual = actual_start->getVariablePosition(joint_name);
    const double error = std::abs(actual - expected);
    candidate_matches = candidate_matches && std::isfinite(error) && error <= 0.01;
    RCLCPP_INFO(
      node->get_logger(),
      "GRASP_CANDIDATE_START_CHECK: %s expected=%.6f actual=%.6f error=%.6f rad.",
      joint_name.c_str(), expected, actual, error);
  }
  if (!candidate_matches) {
    RCLCPP_ERROR(
      node->get_logger(),
      "GRASP_CANDIDATE_START FAIL: current arm is not at accepted grasp_candidate_01; no motion was attempted.");
    stop();
    return 1;
  }

  gripper_group.setStartState(*actual_start);
  if (!gripper_group.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.643)) {
    RCLCPP_ERROR(node->get_logger(), "CLOSE_TARGET FAIL: no motion was attempted.");
    stop();
    return 1;
  }
  Plan close_plan;
  if (
    gripper_group.plan(close_plan) != moveit::core::MoveItErrorCode::SUCCESS ||
    !plan_is_valid(close_plan))
  {
    RCLCPP_ERROR(node->get_logger(), "CLOSE_PLAN FAIL: no motion was attempted.");
    stop();
    return 1;
  }

  moveit::core::RobotState after_close(*actual_start);
  apply_plan_endpoint(after_close, close_plan);

  geometry_msgs::msg::PoseStamped pregrasp_target;
  pregrasp_target.header.frame_id = "world";
  pregrasp_target.header.stamp = node->now();
  pregrasp_target.pose.position.x = 0.346;
  pregrasp_target.pose.position.y = 0.131;
  pregrasp_target.pose.position.z = 0.268;
  constexpr double qx = 0.997;
  constexpr double qy = 0.072;
  constexpr double qz = 0.009;
  constexpr double qw = 0.001;
  const double q_norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  pregrasp_target.pose.orientation.x = qx / q_norm;
  pregrasp_target.pose.orientation.y = qy / q_norm;
  pregrasp_target.pose.orientation.z = qz / q_norm;
  pregrasp_target.pose.orientation.w = qw / q_norm;

  arm_group.setStartState(after_close);
  arm_group.setPlanningPipelineId("pilz_industrial_motion_planner");
  arm_group.setPlannerId("LIN");
  arm_group.setPoseTarget(pregrasp_target, "tool0");
  Plan lift_plan;
  if (
    arm_group.plan(lift_plan) != moveit::core::MoveItErrorCode::SUCCESS ||
    !plan_is_valid(lift_plan))
  {
    RCLCPP_ERROR(node->get_logger(), "LIFT_PLAN FAIL: no motion was attempted.");
    stop();
    return 1;
  }

  moveit::core::RobotState after_lift(after_close);
  apply_plan_endpoint(after_lift, lift_plan);
  const auto * arm_jmg = after_lift.getJointModelGroup("ur_manipulator");
  if (arm_jmg == nullptr) {
    RCLCPP_ERROR(node->get_logger(), "TRANSFER_TARGET FAIL: arm group unavailable.");
    stop();
    return 1;
  }
  const auto & arm_joint_names = arm_jmg->getVariableNames();
  std::vector<double> transfer_values;
  after_lift.copyJointGroupPositions(arm_jmg, transfer_values);
  auto shoulder_it = std::find(
    arm_joint_names.begin(), arm_joint_names.end(), "shoulder_pan_joint");
  if (shoulder_it == arm_joint_names.end()) {
    RCLCPP_ERROR(node->get_logger(), "TRANSFER_TARGET FAIL: shoulder joint unavailable.");
    stop();
    return 1;
  }
  const auto shoulder_index = static_cast<std::size_t>(
    std::distance(arm_joint_names.begin(), shoulder_it));
  constexpr double kHalfTurn = 3.14159265358979323846;
  transfer_values[shoulder_index] += kHalfTurn;

  arm_group.clearPoseTargets();
  arm_group.setStartState(after_lift);
  arm_group.setPlanningPipelineId("ompl");
  arm_group.setPlannerId("");
  if (!arm_group.setJointValueTarget(transfer_values)) {
    RCLCPP_ERROR(node->get_logger(), "TRANSFER_TARGET FAIL: target rejected.");
    stop();
    return 1;
  }
  Plan transfer_plan;
  if (
    arm_group.plan(transfer_plan) != moveit::core::MoveItErrorCode::SUCCESS ||
    !plan_is_valid(transfer_plan))
  {
    RCLCPP_ERROR(node->get_logger(), "TRANSFER_PLAN FAIL: no motion was attempted.");
    stop();
    return 1;
  }

  moveit::core::RobotState after_transfer(after_lift);
  apply_plan_endpoint(after_transfer, transfer_plan);
  gripper_group.setStartState(after_transfer);
  if (!gripper_group.setNamedTarget("open")) {
    RCLCPP_ERROR(node->get_logger(), "RELEASE_TARGET FAIL: no motion was attempted.");
    stop();
    return 1;
  }
  Plan release_plan;
  if (
    gripper_group.plan(release_plan) != moveit::core::MoveItErrorCode::SUCCESS ||
    !plan_is_valid(release_plan))
  {
    RCLCPP_ERROR(node->get_logger(), "RELEASE_PLAN FAIL: no motion was attempted.");
    stop();
    return 1;
  }

  moveit::core::RobotState after_release(after_transfer);
  apply_plan_endpoint(after_release, release_plan);
  const auto home_values = arm_group.getNamedTargetValues("real_home_01");
  if (home_values.size() != 6U) {
    RCLCPP_ERROR(node->get_logger(), "HOME_TARGET FAIL: real_home_01 unavailable.");
    stop();
    return 1;
  }
  arm_group.setStartState(after_release);
  arm_group.setPlanningPipelineId("ompl");
  arm_group.setPlannerId("");
  if (!arm_group.setJointValueTarget(home_values)) {
    RCLCPP_ERROR(node->get_logger(), "HOME_TARGET FAIL: target rejected.");
    stop();
    return 1;
  }
  Plan home_plan;
  if (
    arm_group.plan(home_plan) != moveit::core::MoveItErrorCode::SUCCESS ||
    !plan_is_valid(home_plan))
  {
    RCLCPP_ERROR(node->get_logger(), "HOME_PLAN FAIL: no motion was attempted.");
    stop();
    return 1;
  }

  moveit_msgs::msg::DisplayTrajectory display;
  display.model_id = robot_model->getName();
  display.trajectory_start = close_plan.start_state_;
  display.trajectory.push_back(close_plan.trajectory_);
  display.trajectory.push_back(lift_plan.trajectory_);
  display.trajectory.push_back(transfer_plan.trajectory_);
  display.trajectory.push_back(release_plan.trajectory_);
  display.trajectory.push_back(home_plan.trajectory_);
  display_publisher->publish(display);
  RCLCPP_INFO(
    node->get_logger(),
    "CONTINUATION_DISPLAY_PUBLISHED: retained close, 60 mm lift, shoulder +pi transfer, release and home Plans sent to RViz; all gates locked.");

  auto stage = run_retained_stage(node, gripper_group, close_gate, "CLOSE", close_plan);
  if (const int result = finish_stage(stage); result >= 0) {return result;}
  RCLCPP_INFO(node->get_logger(), "GRASP_DWELL: holding closed for 2 seconds.");
  rclcpp::sleep_for(std::chrono::seconds(2));

  stage = run_retained_stage(node, arm_group, lift_gate, "LIFT", lift_plan);
  if (const int result = finish_stage(stage); result >= 0) {return result;}

  stage = run_retained_stage(node, arm_group, transfer_gate, "TRANSFER", transfer_plan);
  if (const int result = finish_stage(stage); result >= 0) {return result;}

  stage = run_retained_stage(node, gripper_group, release_gate, "RELEASE", release_plan);
  if (const int result = finish_stage(stage); result >= 0) {return result;}
  RCLCPP_INFO(node->get_logger(), "RELEASE_SETTLING: holding still for 2 seconds.");
  rclcpp::sleep_for(std::chrono::seconds(2));

  stage = run_retained_stage(node, arm_group, home_gate, "HOME", home_plan);
  if (const int result = finish_stage(stage); result >= 0) {return result;}

  RCLCPP_INFO(
    node->get_logger(),
    "REAL_PICK_PLACE_CONTINUATION PASS: all five exact retained stages completed.");
  stop();
  return 0;
}
