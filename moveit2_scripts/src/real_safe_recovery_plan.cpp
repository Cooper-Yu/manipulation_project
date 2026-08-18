#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

enum class GateState
{
  LOCKED,
  READY,
  CONSUMED,
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = rclcpp::Node::make_shared(
    "real_safe_recovery_plan",
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

  std::atomic<GateState> gate_state{GateState::LOCKED};
  const auto authorization_service = node->create_service<std_srvs::srv::Trigger>(
    "authorize_safe_recovery",
    [&gate_state](
      const std::shared_ptr<std_srvs::srv::Trigger::Request>,
      std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
      GateState expected = GateState::READY;
      bool accepted = false;
      // Consume READY exactly once; all other states reject authorization.
      accepted =
        gate_state.compare_exchange_strong(expected, GateState::CONSUMED);
      response->success = accepted;
      response->message = accepted ?
        "Authorization accepted; validating the retained Plan start state before execution." :
        "Authorization rejected: the one-shot gate is already consumed or still locked.";
    });
  (void)authorization_service;

  const auto current_state = move_group.getCurrentState(5.0);
  if (!current_state) {
    RCLCPP_ERROR(
      node->get_logger(),
      "RECOVERY_CURRENT_STATE FAIL: no plan or motion was attempted.");
    stop();
    return 1;
  }

  // Use the captured LEFT_HOVER_CANDIDATE_01 joint branch as the deterministic
  // recovery endpoint; the full planned path still requires human review.
  const std::map<std::string, double> recovery_joint_target = {
    {"shoulder_pan_joint", -0.0225732962},
    {"shoulder_lift_joint", -1.2307605904},
    {"elbow_joint", 1.2647030989},
    {"wrist_1_joint", -1.2279044551},
    {"wrist_2_joint", -1.4764826933},
    {"wrist_3_joint", 0.3865525126}
  };

  if (recovery_joint_target.size() != 6U) {
    RCLCPP_ERROR(
      node->get_logger(),
      "RECOVERY_TARGET FAIL: expected six named joints; no plan or motion was attempted.");
    stop();
    return 1;
  }

  move_group.setPlanningPipelineId("ompl");
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(1);
  move_group.setStartState(*current_state);
  if (!move_group.setJointValueTarget(recovery_joint_target)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "RECOVERY_TARGET FAIL: MoveIt rejected the named joint target; no plan or motion was attempted.");
    stop();
    return 1;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  if (move_group.plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(
      node->get_logger(),
      "RECOVERY_PLAN FAIL: no motion was attempted.");
    stop();
    return 1;
  }

  const auto & trajectory = plan.trajectory_.joint_trajectory;
  if (trajectory.points.empty()) {
    RCLCPP_ERROR(
      node->get_logger(),
      "RECOVERY_TRAJECTORY FAIL: no points; no motion was attempted.");
    stop();
    return 1;
  }

  const auto & start_positions = trajectory.points.front().positions;
  const auto & end_positions = trajectory.points.back().positions;
  if (trajectory.joint_names.size() != start_positions.size() ||
    start_positions.size() != end_positions.size())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "RECOVERY_TRAJECTORY FAIL: inconsistent joint arrays; no motion was attempted.");
    stop();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(), "RECOVERY_TRAJECTORY: points=%zu.", trajectory.points.size());
  for (std::size_t i = 0; i < trajectory.joint_names.size(); ++i) {
    RCLCPP_INFO(
      node->get_logger(),
      "RECOVERY_JOINT_DELTA: %s start=%.6f end=%.6f delta=%.6f rad.",
      trajectory.joint_names[i].c_str(), start_positions[i], end_positions[i],
      end_positions[i] - start_positions[i]);
  }

  gate_state.store(GateState::READY);
  const auto authorization_deadline =
    std::chrono::steady_clock::now() + std::chrono::seconds(60);
  RCLCPP_WARN(
    node->get_logger(),
    "RECOVERY_AUTHORIZATION_READY: reviewed Plan retained for up to 60 seconds; one-shot authorization and fresh-state validation are required before execution.");

  while (
    gate_state.load() != GateState::CONSUMED &&
    std::chrono::steady_clock::now() < authorization_deadline)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }

  GateState expected = GateState::READY;
  const bool timed_out =
    gate_state.compare_exchange_strong(expected, GateState::LOCKED);
  if (timed_out) {
    RCLCPP_WARN(
      node->get_logger(),
      "RECOVERY_AUTHORIZATION_TIMEOUT: retained Plan invalidated; no motion was attempted.");
    stop();
    return 0;
  }

  if (gate_state.load() != GateState::CONSUMED) {
    RCLCPP_ERROR(
      node->get_logger(),
      "RECOVERY_GATE FAIL: unexpected state; retained Plan invalidated and no motion was attempted.");
    gate_state.store(GateState::LOCKED);
    stop();
    return 1;
  }

  // Re-read the robot state after the one-shot authorization has been consumed
  // and before any future execution path.
  const auto fresh_state = move_group.getCurrentState(5.0);
  // Reject the retained Plan if an authorization-time state cannot be read.
  if (!fresh_state) {
    // Invalidate the retained trajectory before leaving the failed state-read path.
    plan.trajectory_.joint_trajectory.points.clear();
    // Stop the executor thread and shut down ROS before the failure exit.
    stop();
    // Exit before the authorization-consumed success path.
    return 1;
  }
  // Allow at most 0.01 rad of authorization-time start-state drift per joint.
  constexpr double start_tolerance = 0.01;
  // Start optimistic; every joint comparison can only keep or revoke the match.
  bool start_state_matches = true;
  // Compare every retained trajectory joint against the authorization-time state.
  for (std::size_t i = 0; i < trajectory.joint_names.size(); ++i) {
    // Query by retained joint name rather than assuming RobotState variable order.
    const double actual_position = fresh_state->getVariablePosition(trajectory.joint_names[i]);
    // Measure direction-independent drift from the retained trajectory start.
    const double position_error = std::abs(actual_position - start_positions[i]);
    // Preserve any earlier mismatch while adding the current joint result.
    start_state_matches = start_state_matches && (position_error <= start_tolerance);
  }
  // Reject the retained Plan if any authorization-time joint exceeds tolerance.
  if (!start_state_matches) {
    // Invalidate the retained trajectory before leaving the stale-state path.
    plan.trajectory_.joint_trajectory.points.clear();
    // Stop the executor thread and shut down ROS before the failure exit.
    stop();
    // Exit before the authorization-consumed success path.
    return 1;
  }
  // Execute the same retained Plan that passed authorization and freshness checks.
  const auto execute_result = move_group.execute(plan);
  // Treat every non-SUCCESS Execute result as a terminal failure.
  if (execute_result != moveit::core::MoveItErrorCode::SUCCESS) {
    // Stop the executor thread and shut down ROS after Execute failure.
    stop();
    // Propagate Execute failure to the caller without retrying or replanning.
    return 1;
  }
  // Record success before shutting down the ROS runtime.
  RCLCPP_INFO(
    node->get_logger(),
    "EXECUTE PASS: reviewed retained Plan Execute SUCCESS.");
  stop();
  return 0;
}
