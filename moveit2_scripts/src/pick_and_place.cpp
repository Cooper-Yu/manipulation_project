#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
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
  bool approach_plan_only = false;
  node->get_parameter("approach_plan_only", approach_plan_only);
  bool stop_after_approach = false;
  node->get_parameter("stop_after_approach", stop_after_approach);
  bool skip_pre_grasp = false;
  node->get_parameter("skip_pre_grasp", skip_pre_grasp);
  bool stop_after_close = false;
  node->get_parameter("stop_after_close", stop_after_close);
  bool stop_after_transfer = true;
  node->get_parameter("stop_after_transfer", stop_after_transfer);

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

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  bool success = false;
  bool execution_success = false;
  bool gripper_open_success = false;
  bool approach_success = false;
  bool approach_plan_only_success = false;
  bool stop_after_approach_success = false;
  bool gripper_close_success = false;
  bool stop_after_close_success = false;
  bool retreat_success = false;
  bool transfer_success = false;

  if (skip_pre_grasp) {
    success = true;
    execution_success = true;
    RCLCPP_INFO(
      node->get_logger(),
      "PRE_GRASP_DIAGNOSTIC_SKIP: using the current robot state; no pre-grasp plan was requested.");
  } else {
    // Use the observed collision-free IK branch for the pre-grasp pose. Pose
    // IK alone can select a folded branch whose vertical LIN approach
    // self-collides even though the endpoint pose is identical.
    const std::map<std::string, double> pre_grasp_joint_target = {
      {"shoulder_pan_joint", -0.4537623629},
      {"shoulder_lift_joint", -1.4902915267},
      {"elbow_joint", 1.6791594026},
      {"wrist_1_joint", -1.7592179731},
      {"wrist_2_joint", -1.5706539700},
      {"wrist_3_joint", -0.4543265903},
    };
    move_group.setJointValueTarget(pre_grasp_joint_target);
    const auto result = move_group.plan(plan);
    success = (result == moveit::core::MoveItErrorCode::SUCCESS);
  }

  if (success && !skip_pre_grasp) {
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
  } else if (!success) {
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
    // Pilz Cartesian goals must use the robot model frame. The SRDF fixed
    // virtual joint makes world -> base_link an identity transform, so the
    // target coordinates remain unchanged.
    approach_target.header.frame_id = "world";
    approach_target.pose.position.z = 0.167399;

    // Keep the final descent geometrically predictable. Unlike sampling-based
    // OMPL paths, Pilz LIN constrains the tool motion to a straight segment.
    move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group.setPlannerId("LIN");
    move_group.setPlanningTime(5.0);
    move_group.setNumPlanningAttempts(1);
    move_group.setMaxVelocityScalingFactor(0.01);
    move_group.setMaxAccelerationScalingFactor(0.01);
    move_group.setStartStateToCurrentState();
    move_group.setPoseReferenceFrame("world");
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
    } else if (approach_plan_only) {
      approach_plan_only_success = true;
      RCLCPP_INFO(
        node->get_logger(),
        "APPROACH_LIN_PLAN_ONLY PASS: straight-line approach is plannable; execution remains locked.");
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
        if (stop_after_approach) {
          stop_after_approach_success = true;
          RCLCPP_INFO(
            node->get_logger(),
            "STOP_AFTER_APPROACH PASS: gripper close and retreat remain locked for geometry inspection.");
        }
      } else {
        RCLCPP_ERROR(
          node->get_logger(),
          "APPROACH_EXECUTION FAIL: gripper close remains locked.");
      }
    }
  }

  if (approach_success && !stop_after_approach) {
    gripper_group.setStartStateToCurrentState();

    const bool close_target_success =
      gripper_group.setJointValueTarget(
        "robotiq_85_left_knuckle_joint", 0.645);

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

          // Give the contact solver a short static settling interval. The
          // following retreat supplies the opposing friction forces used by
          // GazeboGraspFix to confirm attachment in the local simulation.
          RCLCPP_INFO(
            node->get_logger(),
            "GRASP_DWELL: holding the closed gripper for contact stabilization.");
          rclcpp::sleep_for(std::chrono::seconds(2));
          if (stop_after_close) {
            stop_after_close_success = true;
            RCLCPP_INFO(
              node->get_logger(),
              "STOP_AFTER_CLOSE PASS: retreat remains locked for contact inspection.");
          }
        } else {
          RCLCPP_ERROR(
            node->get_logger(),
            "GRIPPER_CLOSE_EXECUTION FAIL: retreat remains locked.");
        }
      }
    }
  }

  if (gripper_close_success && !stop_after_close) {
    geometry_msgs::msg::PoseStamped retreat_pose = target;
    retreat_pose.header.stamp = node->now();
    retreat_pose.header.frame_id = "world";

    move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group.setPlannerId("LIN");
    move_group.setPlanningTime(5.0);
    move_group.setNumPlanningAttempts(1);
    move_group.setMaxVelocityScalingFactor(0.01);
    move_group.setMaxAccelerationScalingFactor(0.01);
    move_group.setStartStateToCurrentState();
    move_group.setPoseReferenceFrame("world");
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

  if (retreat_success) {
    auto transfer_joint_values = move_group.getCurrentJointValues();
    const auto joint_names = move_group.getJointNames();
    const auto shoulder_it = std::find(
      joint_names.begin(), joint_names.end(), "shoulder_pan_joint");

    if (shoulder_it == joint_names.end()) {
      RCLCPP_ERROR(
        node->get_logger(),
        "TRANSFER_TARGET FAIL: shoulder_pan_joint was not found; transfer was not planned.");
    } else {
      const auto shoulder_index =
        static_cast<std::size_t>(std::distance(joint_names.begin(), shoulder_it));

      if (shoulder_index >= transfer_joint_values.size()) {
        RCLCPP_ERROR(
          node->get_logger(),
          "TRANSFER_TARGET FAIL: joint-name and joint-value vectors are inconsistent.");
      } else {
        constexpr double kHalfTurn = 3.14159265358979323846;
        transfer_joint_values[shoulder_index] += kHalfTurn;

        move_group.setPlanningPipelineId("ompl");
        move_group.setPlannerId("");
        move_group.setPlanningTime(5.0);
        move_group.setNumPlanningAttempts(5);
        move_group.setMaxVelocityScalingFactor(0.05);
        move_group.setMaxAccelerationScalingFactor(0.05);
        move_group.setStartStateToCurrentState();

        if (!move_group.setJointValueTarget(transfer_joint_values)) {
          RCLCPP_ERROR(
            node->get_logger(),
            "TRANSFER_TARGET FAIL: the 180-degree shoulder target was rejected.");
        } else {
          moveit::planning_interface::MoveGroupInterface::Plan transfer_plan;
          const auto transfer_plan_result = move_group.plan(transfer_plan);
          const bool transfer_plan_success =
            (transfer_plan_result == moveit::core::MoveItErrorCode::SUCCESS);

          if (!transfer_plan_success) {
            RCLCPP_ERROR(
              node->get_logger(),
              "TRANSFER_PLAN FAIL: execution and gripper release were not attempted.");
          } else {
            RCLCPP_INFO(
              node->get_logger(),
              "TRANSFER_PLAN PASS: starting 180-degree shoulder execution.");

            const auto transfer_execution_result =
              move_group.execute(transfer_plan);
            transfer_success =
              (transfer_execution_result == moveit::core::MoveItErrorCode::SUCCESS);

            if (transfer_success) {
              RCLCPP_INFO(
                node->get_logger(),
                "TRANSFER_EXECUTION PASS: loading-side shoulder motion completed.");
              if (stop_after_transfer) {
                RCLCPP_INFO(
                  node->get_logger(),
                  "STOP_AFTER_TRANSFER PASS: gripper release remains locked for visual inspection.");
              }
            } else {
              RCLCPP_ERROR(
                node->get_logger(),
                "TRANSFER_EXECUTION FAIL: gripper release remains locked.");
            }
          }
        }
      }
    }
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  const bool requested_result = approach_plan_only ? approach_plan_only_success :
    (stop_after_approach ? stop_after_approach_success :
    (stop_after_close ? stop_after_close_success : transfer_success));
  return requested_result ? 0 : 1;
}
