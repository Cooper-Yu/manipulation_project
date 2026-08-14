#include <memory>
#include <thread>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  const auto node = rclcpp::Node::make_shared(
    "pick_and_place_plan_probe",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });

  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
  move_group.setStartStateToCurrentState();

  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = "base_link";
  target.header.stamp = node->now();
  target.pose.position.x = 0.34;
  target.pose.position.y = -0.02;
  target.pose.position.z = 0.262399;
  target.pose.orientation =
    tf2::toMsg(tf2::Quaternion(-0.707, 0.707, 0.0, 0.0));

  move_group.setPoseTarget(target, "tool0");

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto result = move_group.plan(plan);
  const bool success =
    (result == moveit::core::MoveItErrorCode::SUCCESS);
  bool execution_success = false;

  if (success) {
    RCLCPP_INFO(
      node->get_logger(),
      "PLAN PASS: pre-grasp pose is plannable; starting trajectory execution.");

    const auto execution_result = move_group.execute(plan);
    execution_success =
      (execution_result == moveit::core::MoveItErrorCode::SUCCESS);

    if (execution_success) {
      RCLCPP_INFO(
        node->get_logger(),
        "PRE_GRASP_EXECUTION PASS: planned trajectory executed successfully.");
    } else {
      RCLCPP_ERROR(
        node->get_logger(),
        "PRE_GRASP_EXECUTION FAIL: planning passed, but execution failed.");
    }
  } else {
    RCLCPP_ERROR(
      node->get_logger(),
      "PLAN FAIL: pre-grasp pose could not be planned; execution was not attempted.");
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return execution_success ? 0 : 1;
}
