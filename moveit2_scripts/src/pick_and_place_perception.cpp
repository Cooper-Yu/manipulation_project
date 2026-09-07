#include <algorithm>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <object_detection/msg/detected_objects.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class DetectionState
{
public:
  void update(const object_detection::msg::DetectedObjects::SharedPtr msg)
  {
    if (!msg || !std::isfinite(msg->position.x) || !std::isfinite(msg->position.y) ||
      !std::isfinite(msg->position.z) || msg->height <= 0.0f ||
      msg->width <= 0.0f || msg->thickness <= 0.0f)
    {
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    object_ = *msg;
    detection_ready_ = true;
    condition_.notify_all();
  }

  bool wait_for_detection(object_detection::msg::DetectedObjects & output,
    std::chrono::seconds timeout)
  {
    std::unique_lock<std::mutex> lock(mutex_);
    if (!condition_.wait_for(lock, timeout, [this]() {return detection_ready_;})) {
      return false;
    }
    output = object_;
    return true;
  }

private:
  std::mutex mutex_;
  std::condition_variable condition_;
  bool detection_ready_{false};
  object_detection::msg::DetectedObjects object_{};
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  const auto node = rclcpp::Node::make_shared(
    "pick_and_place_perception",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  bool gripper_only = false;
  node->get_parameter("gripper_only", gripper_only);
  double gripper_close_position = 0.643;
  node->get_parameter("gripper_close_position", gripper_close_position);
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

  if (gripper_only) {
    if (!std::isfinite(gripper_close_position) || gripper_close_position < 0.0 ||
      gripper_close_position > 1.0)
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "GRIPPER_ONLY FAIL: gripper_close_position must be finite and between 0.0 and 1.0 rad.");
      executor.cancel();
      spin_thread.join();
      rclcpp::shutdown();
      return 1;
    }

    moveit::planning_interface::MoveGroupInterface gripper_group(node, "gripper");
    gripper_group.setStartStateToCurrentState();
    if (!gripper_group.setJointValueTarget(
        "robotiq_85_left_knuckle_joint", gripper_close_position))
    {
      RCLCPP_ERROR(
        node->get_logger(),
        "GRIPPER_ONLY_TARGET FAIL: could not set close position %.3f rad.",
        gripper_close_position);
      executor.cancel();
      spin_thread.join();
      rclcpp::shutdown();
      return 1;
    }

    moveit::planning_interface::MoveGroupInterface::Plan gripper_plan;
    const auto plan_result = gripper_group.plan(gripper_plan);
    if (plan_result != moveit::core::MoveItErrorCode::SUCCESS) {
      RCLCPP_ERROR(
        node->get_logger(),
        "GRIPPER_ONLY_PLAN FAIL: close position %.3f rad could not be planned.",
        gripper_close_position);
      executor.cancel();
      spin_thread.join();
      rclcpp::shutdown();
      return 1;
    }

    RCLCPP_INFO(
      node->get_logger(),
      "GRIPPER_ONLY_PLAN PASS: executing close position %.3f rad from the current arm pose.",
      gripper_close_position);
    const auto execution_result = gripper_group.execute(gripper_plan);
    const bool success = execution_result == moveit::core::MoveItErrorCode::SUCCESS;
    RCLCPP_INFO(
      node->get_logger(),
      "GRIPPER_ONLY_%s: arm motion and perception were skipped; robot remains at the current arm pose.",
      success ? "EXECUTION PASS" : "EXECUTION FAIL");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return success ? 0 : 1;
  }

  auto detection_state = std::make_shared<DetectionState>();
  auto detection_subscription = node->create_subscription<object_detection::msg::DetectedObjects>(
    "object_detected", rclcpp::QoS(10),
    [detection_state](const object_detection::msg::DetectedObjects::SharedPtr msg) {
      detection_state->update(msg);
    });

  object_detection::msg::DetectedObjects detected_object;
  if (!detection_state->wait_for_detection(detected_object, std::chrono::seconds(15))) {
    RCLCPP_ERROR(
      node->get_logger(),
      "DETECTION_WAIT FAIL: no valid /object_detected message received within 15 seconds; no motion was attempted.");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }
  RCLCPP_INFO(
    node->get_logger(),
    "DETECTION_READY PASS: object_id=%u position=[%.4f, %.4f, %.4f] size=[%.4f, %.4f, %.4f].",
    detected_object.object_id, detected_object.position.x, detected_object.position.y,
    detected_object.position.z, detected_object.thickness, detected_object.width,
    detected_object.height);

  moveit::planning_interface::MoveGroupInterface move_group(node, "ur_manipulator");
  moveit::planning_interface::MoveGroupInterface gripper_group(node, "gripper");

  // Capture the actual task-start arm configuration before any motion. This
  // is the Checkpoint's initial position and may differ from named/Section
  // home states.
  const auto initial_state = move_group.getCurrentState(5.0);
  if (!initial_state) {
    RCLCPP_ERROR(
      node->get_logger(),
      "INITIAL_STATE FAIL: no fresh RobotState was received within 5 seconds; no motion was attempted.");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  const auto * initial_joint_group =
    initial_state->getJointModelGroup("ur_manipulator");
  if (initial_joint_group == nullptr) {
    RCLCPP_ERROR(
      node->get_logger(),
      "INITIAL_STATE FAIL: ur_manipulator JointModelGroup was not found; no motion was attempted.");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  const auto initial_joint_names = initial_joint_group->getVariableNames();
  std::vector<double> initial_joint_values;
  initial_state->copyJointGroupPositions(
    initial_joint_group, initial_joint_values);
  if (initial_joint_names.empty() ||
    initial_joint_names.size() != initial_joint_values.size())
  {
    RCLCPP_ERROR(
      node->get_logger(),
      "INITIAL_STATE FAIL: joint-name and joint-value vectors are empty or inconsistent; no motion was attempted.");
    executor.cancel();
    spin_thread.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(
    node->get_logger(),
    "INITIAL_STATE PASS: saved %zu arm joint values for return home.",
    initial_joint_values.size());
  move_group.setStartStateToCurrentState();

  geometry_msgs::msg::PoseStamped target;
  target.header.frame_id = "base_link";
  target.header.stamp = node->now();
  // Reuse the verified Checkpoint 13 tool-center offset while replacing the
  // planar target coordinates with the detected object's base_link position.
  constexpr double kToolCenterToObjectCenterZ = 0.23217;
  // The gripper closes along base_link -Y.  Shift the tool target from the
  // detected object centroid to the intended contact-side grasp reference:
  // +X by half the object's X extent and -Y by half its Y extent.  Keep Z at
  // the previously calibrated value.
  target.pose.position.x = detected_object.position.x +
    static_cast<double>(detected_object.thickness) / 2.0;
  target.pose.position.y = detected_object.position.y -
    static_cast<double>(detected_object.width) / 2.0;
  target.pose.position.z = detected_object.position.z + kToolCenterToObjectCenterZ;
  RCLCPP_INFO(
    node->get_logger(),
    "GRASP_TARGET: centroid=[%.4f, %.4f, %.4f], half_size_shift=[+X %.4f, -Y %.4f], target=[%.4f, %.4f, %.4f].",
    detected_object.position.x, detected_object.position.y, detected_object.position.z,
    static_cast<double>(detected_object.thickness) / 2.0,
    static_cast<double>(detected_object.width) / 2.0,
    target.pose.position.x, target.pose.position.y, target.pose.position.z);
  target.pose.orientation =
    tf2::toMsg(tf2::Quaternion(-0.707, 0.707, 0.0, 0.0));

  // Use the verified Checkpoint 13 arm configuration as the IK seed. The
  // detected pose remains dynamic; this biases IK toward the known branch.
  const std::map<std::string, double> checkpoint13_ik_seed = {
    {"shoulder_pan_joint", -0.4537623629},
    {"shoulder_lift_joint", -1.4902915267},
    {"elbow_joint", 1.6791594026},
    {"wrist_1_joint", -1.7592179731},
    {"wrist_2_joint", -1.5706539700},
    {"wrist_3_joint", -0.4543265903},
  };
  const auto robot_model = move_group.getRobotModel();
  const auto * arm_group = robot_model->getJointModelGroup("ur_manipulator");
  if (arm_group == nullptr) {
    RCLCPP_ERROR(node->get_logger(), "IK_SEED FAIL: arm group was not found; no motion was attempted.");
    executor.cancel(); spin_thread.join(); rclcpp::shutdown(); return 1;
  }
  moveit::core::RobotState ik_seed_state(robot_model);
  ik_seed_state.setToDefaultValues();
  for (const auto & [joint_name, joint_value] : checkpoint13_ik_seed) {
    ik_seed_state.setJointPositions(joint_name, &joint_value);
  }
  ik_seed_state.update();
  if (!ik_seed_state.setFromIK(arm_group, target.pose, "tool0", 0.5)) {
    RCLCPP_ERROR(node->get_logger(), "IK_SEED FAIL: no solution near the Checkpoint 13 branch.");
    executor.cancel(); spin_thread.join(); rclcpp::shutdown(); return 1;
  }
  std::vector<double> seeded_joint_target;
  ik_seed_state.copyJointGroupPositions(arm_group, seeded_joint_target);
  RCLCPP_INFO(node->get_logger(), "IK_SEED PASS: dynamic target solved near the Checkpoint 13 arm branch.");

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
  bool release_success = false;
  bool home_success = false;

  if (skip_pre_grasp) {
    success = true;
    execution_success = true;
    RCLCPP_INFO(
      node->get_logger(),
      "PRE_GRASP_DIAGNOSTIC_SKIP: using the current robot state; no pre-grasp plan was requested.");
  } else {
    // The old Checkpoint 13 joint target belongs to the old fixed object
    // position. For a perception target, plan to the dynamically generated
    // Cartesian pre-grasp pose so MoveIt computes a new valid IK solution.
    move_group.setPlanningPipelineId("ompl");
    move_group.setPlannerId("");
    move_group.setPlanningTime(5.0);
    move_group.setNumPlanningAttempts(5);
    move_group.setMaxVelocityScalingFactor(0.05);
    move_group.setMaxAccelerationScalingFactor(0.05);
    move_group.setStartStateToCurrentState();
    move_group.setPoseReferenceFrame("base_link");
    move_group.setEndEffectorLink("tool0");
    move_group.setJointValueTarget(seeded_joint_target);
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
    constexpr double kPreGraspToPickZ = 0.095;
    approach_target.pose.position.z = target.pose.position.z - kPreGraspToPickZ;

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
        "robotiq_85_left_knuckle_joint", gripper_close_position);

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
    // Wait explicitly for one fresh RobotState after retreat. Reading the
    // names and positions from the same JointModelGroup keeps their ordering
    // consistent and avoids planning from an empty/stale value vector.
    const auto current_state = move_group.getCurrentState(5.0);
    if (!current_state) {
      RCLCPP_ERROR(
        node->get_logger(),
        "TRANSFER_STATE FAIL: no fresh RobotState was received within 5 seconds; transfer was not planned.");
    } else {
      const auto * joint_group =
        current_state->getJointModelGroup("ur_manipulator");
      if (joint_group == nullptr) {
        RCLCPP_ERROR(
          node->get_logger(),
          "TRANSFER_STATE FAIL: ur_manipulator JointModelGroup was not found; transfer was not planned.");
      } else {
        const auto & joint_names = joint_group->getVariableNames();
        std::vector<double> transfer_joint_values;
        current_state->copyJointGroupPositions(
          joint_group, transfer_joint_values);
        const auto shoulder_it = std::find(
          joint_names.begin(), joint_names.end(), "shoulder_pan_joint");

        if (joint_names.size() != transfer_joint_values.size()) {
          RCLCPP_ERROR(
            node->get_logger(),
            "TRANSFER_TARGET FAIL: joint-name and joint-value vectors are inconsistent.");
        } else if (shoulder_it == joint_names.end()) {
          RCLCPP_ERROR(
            node->get_logger(),
            "TRANSFER_TARGET FAIL: shoulder_pan_joint was not found; transfer was not planned.");
        } else {
          const auto shoulder_index = static_cast<std::size_t>(
            std::distance(joint_names.begin(), shoulder_it));
          constexpr double kHalfTurn = 3.14159265358979323846;
          transfer_joint_values[shoulder_index] += kHalfTurn;

          move_group.setPlanningPipelineId("ompl");
          move_group.setPlannerId("");
          move_group.setPlanningTime(5.0);
          move_group.setNumPlanningAttempts(5);
          move_group.setMaxVelocityScalingFactor(0.05);
          move_group.setMaxAccelerationScalingFactor(0.05);
          move_group.setStartState(*current_state);

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
  }

  if (transfer_success && !stop_after_transfer) {
    gripper_group.setStartStateToCurrentState();

    if (!gripper_group.setNamedTarget("open")) {
      RCLCPP_ERROR(
        node->get_logger(),
        "RELEASE_TARGET FAIL: named target 'open' was not accepted.");
    } else {
      moveit::planning_interface::MoveGroupInterface::Plan release_plan;
      const auto release_plan_result = gripper_group.plan(release_plan);
      const bool release_plan_success =
        (release_plan_result == moveit::core::MoveItErrorCode::SUCCESS);

      if (!release_plan_success) {
        RCLCPP_ERROR(
          node->get_logger(),
          "RELEASE_PLAN FAIL: execution and return home were not attempted.");
      } else {
        RCLCPP_INFO(
          node->get_logger(),
          "RELEASE_PLAN PASS: starting gripper execution.");

        const auto release_execution_result =
          gripper_group.execute(release_plan);
        release_success =
          (release_execution_result == moveit::core::MoveItErrorCode::SUCCESS);

        if (release_success) {
          RCLCPP_INFO(
            node->get_logger(),
            "RELEASE_EXECUTION PASS: gripper opened; verify the blue block placement visually.");
          RCLCPP_INFO(
            node->get_logger(),
            "RELEASE_SETTLING: holding the robot still for 2 seconds before return home.");
          rclcpp::sleep_for(std::chrono::seconds(2));
        } else {
          RCLCPP_ERROR(
            node->get_logger(),
            "RELEASE_EXECUTION FAIL: return home remains locked.");
        }
      }
    }
  }

  if (release_success) {
    const auto home_start_state = move_group.getCurrentState(5.0);
    if (!home_start_state) {
      RCLCPP_ERROR(
        node->get_logger(),
        "HOME_STATE FAIL: no fresh RobotState was received; return home was not planned.");
    } else {
      move_group.setPlanningPipelineId("ompl");
      move_group.setPlannerId("");
      move_group.setPlanningTime(5.0);
      move_group.setNumPlanningAttempts(5);
      move_group.setMaxVelocityScalingFactor(0.05);
      move_group.setMaxAccelerationScalingFactor(0.05);
      move_group.setStartState(*home_start_state);

      if (!move_group.setJointValueTarget(initial_joint_values)) {
        RCLCPP_ERROR(
          node->get_logger(),
          "HOME_TARGET FAIL: saved initial joint target was rejected.");
      } else {
        moveit::planning_interface::MoveGroupInterface::Plan home_plan;
        const auto home_plan_result = move_group.plan(home_plan);
        const bool home_plan_success =
          (home_plan_result == moveit::core::MoveItErrorCode::SUCCESS);

        if (!home_plan_success) {
          RCLCPP_ERROR(
            node->get_logger(),
            "HOME_PLAN FAIL: execution was not attempted.");
        } else {
          RCLCPP_INFO(
            node->get_logger(),
            "HOME_PLAN PASS: starting return-to-initial-position execution.");

          const auto home_execution_result = move_group.execute(home_plan);
          home_success =
            (home_execution_result == moveit::core::MoveItErrorCode::SUCCESS);

          if (home_success) {
            RCLCPP_INFO(
              node->get_logger(),
              "HOME_EXECUTION PASS: robot returned to the saved initial position.");
          } else {
            RCLCPP_ERROR(
              node->get_logger(),
              "HOME_EXECUTION FAIL: complete Pick & Place remains incomplete.");
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
    (stop_after_close ? stop_after_close_success :
    (stop_after_transfer ? transfer_success : home_success)));
  return requested_result ? 0 : 1;
}
