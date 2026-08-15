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
  moveit::planning_interface::MoveGroupInterface gripper_group(node, "gripper");
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
  bool gripper_open_success = false;
  bool approach_success = false;
  bool gripper_close_success = false;
  bool retreat_success = false;

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

  if (execution_success) {
    gripper_group.setStartStateToCurrentState();

    if (!gripper_group.setNamedTarget("open")) {
      RCLCPP_ERROR(
        node->get_logger(),
        "GRIPPER_OPEN_TARGET FAIL: named target 'open' was not accepted.");
    } else {
      moveit::planning_interface::MoveGroupInterface::Plan open_plan;
      const auto open_plan_result = gripper_group.plan(open_plan);
      const bool open_plan_success =
        (open_plan_result == moveit::core::MoveItErrorCode::SUCCESS);

      if (!open_plan_success) {
        RCLCPP_ERROR(
          node->get_logger(),
          "GRIPPER_OPEN_PLAN FAIL: execution was not attempted.");
      } else {
        RCLCPP_INFO(
          node->get_logger(),
          "GRIPPER_OPEN_PLAN PASS: starting gripper execution.");

        const auto open_execution_result = gripper_group.execute(open_plan);
        gripper_open_success =
          (open_execution_result == moveit::core::MoveItErrorCode::SUCCESS);

        if (gripper_open_success) {
          RCLCPP_INFO(
            node->get_logger(),
            "GRIPPER_OPEN_EXECUTION PASS: gripper open trajectory executed successfully.");
        } else {
          RCLCPP_ERROR(
            node->get_logger(),
            "GRIPPER_OPEN_EXECUTION FAIL: planning passed, but execution failed.");
        }
      }
    }
  }

  if (gripper_open_success) {
    geometry_msgs::msg::PoseStamped approach_target = target;
    approach_target.header.stamp = node->now();
    approach_target.pose.position.z = 0.167399;

    move_group.setStartStateToCurrentState();
    move_group.setPoseReferenceFrame("base_link");
    move_group.setEndEffectorLink("tool0");
    move_group.setPoseTarget(approach_target, "tool0");

    moveit::planning_interface::MoveGroupInterface::Plan approach_plan;
    const auto approach_plan_result = move_group.plan(approach_plan);
    const bool approach_plan_success =
      (approach_plan_result == moveit::core::MoveItErrorCode::SUCCESS);

    if (!approach_plan_success) {
      RCLCPP_ERROR(
        node->get_logger(),
        "APPROACH_PLAN FAIL: execution and gripper close were not attempted.");
    } else {
      RCLCPP_INFO(
        node->get_logger(),
        "APPROACH_PLAN PASS: starting trajectory execution.");

      const auto approach_execution_result =
        move_group.execute(approach_plan);
      approach_success =
        (approach_execution_result == moveit::core::MoveItErrorCode::SUCCESS);

      if (approach_success) {
        RCLCPP_INFO(
          node->get_logger(),
          "APPROACH_EXECUTION PASS: trajectory executed successfully.");
      } else {
        RCLCPP_ERROR(
          node->get_logger(),
          "APPROACH_EXECUTION FAIL: gripper close remains locked.");
      }
    }
  }

  if (approach_success) {
    gripper_group.setStartStateToCurrentState();

    const bool close_target_success =
      gripper_group.setJointValueTarget(
        "robotiq_85_left_knuckle_joint", 0.625);

    if (!close_target_success) {
      RCLCPP_ERROR(
        node->get_logger(),
        "GRIPPER_CLOSE_TARGET FAIL: planning was not attempted.");
    } else {
      moveit::planning_interface::MoveGroupInterface::Plan close_plan;
      const auto close_plan_result = gripper_group.plan(close_plan);
      const bool close_plan_success =
        (close_plan_result == moveit::core::MoveItErrorCode::SUCCESS);

      if (!close_plan_success) {
        RCLCPP_ERROR(
          node->get_logger(),
          "GRIPPER_CLOSE_PLAN FAIL: execution was not attempted.");
      } else {
        RCLCPP_INFO(
          node->get_logger(),
          "GRIPPER_CLOSE_PLAN PASS: starting gripper execution.");

        const auto close_execution_result =
          gripper_group.execute(close_plan);
        gripper_close_success =
          (close_execution_result == moveit::core::MoveItErrorCode::SUCCESS);

        if (gripper_close_success) {
          RCLCPP_INFO(
            node->get_logger(),
            "GRIPPER_CLOSE_EXECUTION PASS: close trajectory executed successfully.");
        } else {
          RCLCPP_ERROR(
            node->get_logger(),
            "GRIPPER_CLOSE_EXECUTION FAIL: retreat remains locked.");
        }
      }
    }
  }

  if (gripper_close_success) {
    geometry_msgs::msg::PoseStamped retreat_pose = target;
    retreat_pose.header.stamp = node->now();

    move_group.setStartStateToCurrentState();
    move_group.setPoseReferenceFrame("base_link");
    move_group.setEndEffectorLink("tool0");
    move_group.setPoseTarget(retreat_pose, "tool0");

    moveit::planning_interface::MoveGroupInterface::Plan retreat_plan;
    const auto retreat_plan_result = move_group.plan(retreat_plan);
    const bool retreat_plan_success =
      (retreat_plan_result == moveit::core::MoveItErrorCode::SUCCESS);

    if (!retreat_plan_success) {
      RCLCPP_ERROR(
        node->get_logger(),
        "RETREAT_PLAN FAIL: execution and shoulder transfer were not attempted.");
    } else {
      RCLCPP_INFO(
        node->get_logger(),
        "RETREAT_PLAN PASS: starting trajectory execution.");

      const auto retreat_execution_result =
        move_group.execute(retreat_plan);
      retreat_success =
        (retreat_execution_result == moveit::core::MoveItErrorCode::SUCCESS);

      if (retreat_success) {
        RCLCPP_INFO(
          node->get_logger(),
          "RETREAT_EXECUTION PASS: verify that the blue block moved with the gripper.");
      } else {
        RCLCPP_ERROR(
          node->get_logger(),
          "RETREAT_EXECUTION FAIL: shoulder transfer remains locked.");
      }
    }
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return retreat_success ? 0 : 1;
}
