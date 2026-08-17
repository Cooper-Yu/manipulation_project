#include <map>
#include <memory>
#include <string>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

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

  RCLCPP_INFO(
    node->get_logger(),
    "REAL_SAFE_RECOVERY_PLAN_ONLY PASS: trajectory is available for review; no motion was attempted.");
  stop();
  return 0;
}
