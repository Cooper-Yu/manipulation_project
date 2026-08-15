#include <memory>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/robot_trajectory.hpp>
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
    approach_target.pose.position.z = 0.197399;

    move_group.setStartStateToCurrentState();
    move_group.setPoseReferenceFrame("base_link");
    move_group.setEndEffectorLink("tool0");

    const std::vector<geometry_msgs::msg::Pose> approach_waypoints{
      approach_target.pose};
    moveit_msgs::msg::RobotTrajectory approach_trajectory;
    constexpr double approach_eef_step = 0.01;
    constexpr double approach_jump_threshold = 0.0;
    constexpr double minimum_cartesian_fraction = 0.999;

    const double approach_fraction = move_group.computeCartesianPath(
      approach_waypoints,
      approach_eef_step,
      approach_jump_threshold,
      approach_trajectory,
      true);
    const bool approach_path_success =
      (approach_fraction >= minimum_cartesian_fraction);

    if (!approach_path_success) {
      moveit_msgs::msg::RobotTrajectory approach_no_collision_probe;
      const double approach_no_collision_fraction =
        move_group.computeCartesianPath(
          approach_waypoints,
          approach_eef_step,
          approach_jump_threshold,
          approach_no_collision_probe,
          false);

      moveit_msgs::msg::RobotTrajectory approach_dense_step_probe;
      constexpr double approach_dense_probe_eef_step = 0.001;
      const double approach_dense_step_fraction =
        move_group.computeCartesianPath(
          approach_waypoints,
          approach_dense_probe_eef_step,
          approach_jump_threshold,
          approach_dense_step_probe,
          true);

      RCLCPP_ERROR(
        node->get_logger(),
        "APPROACH_CARTESIAN_PATH FAIL: collision_check_fraction=%.3f, no_collision_probe_fraction=%.3f, dense_step_probe_fraction=%.3f; no diagnostic trajectory was executed.",
        approach_fraction,
        approach_no_collision_fraction,
        approach_dense_step_fraction);
    } else {
      RCLCPP_INFO(
        node->get_logger(),
        "APPROACH_CARTESIAN_PATH PASS: fraction=%.3f; starting trajectory execution.",
        approach_fraction);

      const auto approach_execution_result =
        move_group.execute(approach_trajectory);
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

    const std::vector<geometry_msgs::msg::Pose> retreat_waypoints{
      retreat_pose.pose};
    moveit_msgs::msg::RobotTrajectory retreat_trajectory;
    constexpr double retreat_eef_step = 0.01;
    constexpr double retreat_jump_threshold = 0.0;
    constexpr double minimum_retreat_cartesian_fraction = 0.999;

    const double retreat_fraction = move_group.computeCartesianPath(
      retreat_waypoints,
      retreat_eef_step,
      retreat_jump_threshold,
      retreat_trajectory,
      true);
    const bool retreat_path_success =
      (retreat_fraction >= minimum_retreat_cartesian_fraction);

    if (!retreat_path_success) {
      RCLCPP_ERROR(
        node->get_logger(),
        "RETREAT_CARTESIAN_PATH FAIL: fraction=%.3f; execution and shoulder transfer were not attempted.",
        retreat_fraction);
    } else {
      RCLCPP_INFO(
        node->get_logger(),
        "RETREAT_CARTESIAN_PATH PASS: fraction=%.3f; starting trajectory execution.",
        retreat_fraction);

      const auto retreat_execution_result =
        move_group.execute(retreat_trajectory);
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
