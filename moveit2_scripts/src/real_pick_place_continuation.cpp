#include <algorithm>
#include <chrono>
#include <cmath>
#include <future>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using MoveGroup = moveit::planning_interface::MoveGroupInterface;
using Plan = MoveGroup::Plan;
using GripperCommand = control_msgs::action::GripperCommand;
using GripperGoalHandle = rclcpp_action::ClientGoalHandle<GripperCommand>;

constexpr double kStateTolerance = 0.01;
constexpr double kOpenCommand = 0.0;
constexpr double kCloseCommand = 0.643;

bool plan_is_valid(const Plan & plan)
{
  const auto & trajectory = plan.trajectory_.joint_trajectory;
  return !trajectory.joint_names.empty() && !trajectory.points.empty() &&
    trajectory.joint_names.size() == trajectory.points.front().positions.size() &&
    trajectory.joint_names.size() == trajectory.points.back().positions.size();
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

void set_named_values(
  moveit::core::RobotState & state,
  const std::map<std::string, double> & values)
{
  for (const auto & [joint_name, position] : values) {
    state.setVariablePosition(joint_name, position);
  }
  state.update();
}

void set_planning_gripper_state(moveit::core::RobotState & state, double command)
{
  // The course backend publishes a fake gripper value on /joint_states.
  // Override it only inside explicit planning states; update refreshes mimics.
  const auto & variable_names = state.getRobotModel()->getVariableNames();
  if (std::find(
      variable_names.begin(), variable_names.end(),
      "robotiq_85_left_knuckle_joint") != variable_names.end())
  {
    state.setVariablePosition("robotiq_85_left_knuckle_joint", command);
    state.update();
  }
}

moveit_msgs::msg::RobotTrajectory make_display_trajectory(
  const moveit_msgs::msg::RobotTrajectory & arm_trajectory,
  double gripper_position)
{
  // Display-only copy: never add the gripper joint to the retained arm Plan
  // passed to execute(), because the arm controller accepts six arm joints.
  auto display_trajectory = arm_trajectory;
  auto & joint_trajectory = display_trajectory.joint_trajectory;
  const std::map<std::string, double> display_gripper_values = {
    {"robotiq_85_left_knuckle_joint", gripper_position},
    {"robotiq_85_right_knuckle_joint", gripper_position},
    {"robotiq_85_left_inner_knuckle_joint", gripper_position},
    {"robotiq_85_right_inner_knuckle_joint", gripper_position},
    {"robotiq_85_left_finger_tip_joint", -gripper_position},
    {"robotiq_85_right_finger_tip_joint", -gripper_position},
  };

  for (const auto & [joint_name, position] : display_gripper_values) {
    const auto existing = std::find(
      joint_trajectory.joint_names.begin(), joint_trajectory.joint_names.end(),
      joint_name);
    if (existing != joint_trajectory.joint_names.end()) {
      const auto index = static_cast<std::size_t>(
        std::distance(joint_trajectory.joint_names.begin(), existing));
      for (auto & point : joint_trajectory.points) {
        if (index < point.positions.size()) {
          point.positions[index] = position;
        }
      }
      continue;
    }

    const std::size_t original_joint_count = joint_trajectory.joint_names.size();
    joint_trajectory.joint_names.push_back(joint_name);
    for (auto & point : joint_trajectory.points) {
      point.positions.push_back(position);
      if (point.velocities.size() == original_joint_count) {
        point.velocities.push_back(0.0);
      }
      if (point.accelerations.size() == original_joint_count) {
        point.accelerations.push_back(0.0);
      }
      if (point.effort.size() == original_joint_count) {
        point.effort.push_back(0.0);
      }
    }
  }
  return display_trajectory;
}

void set_display_start_gripper_open(moveit_msgs::msg::RobotState & state)
{
  const std::map<std::string, double> open_values = {
    {"robotiq_85_left_knuckle_joint", 0.0},
    {"robotiq_85_right_knuckle_joint", 0.0},
    {"robotiq_85_left_inner_knuckle_joint", 0.0},
    {"robotiq_85_right_inner_knuckle_joint", 0.0},
    {"robotiq_85_left_finger_tip_joint", 0.0},
    {"robotiq_85_right_finger_tip_joint", 0.0},
  };
  for (const auto & [joint_name, position] : open_values) {
    const auto existing = std::find(
      state.joint_state.name.begin(), state.joint_state.name.end(), joint_name);
    if (existing == state.joint_state.name.end()) {
      state.joint_state.name.push_back(joint_name);
      state.joint_state.position.push_back(position);
      continue;
    }
    const auto index = static_cast<std::size_t>(
      std::distance(state.joint_state.name.begin(), existing));
    if (index < state.joint_state.position.size()) {
      state.joint_state.position[index] = position;
    }
  }
}

bool arm_state_matches(
  const rclcpp::Node::SharedPtr & node,
  const moveit::core::RobotStatePtr & actual_state,
  const moveit::core::RobotState & expected_state,
  const std::vector<std::string> & joint_names,
  const std::string & label)
{
  if (!actual_state) {
    RCLCPP_ERROR(node->get_logger(), "%s_STATE FAIL: current state unavailable.", label.c_str());
    return false;
  }
  bool matches = true;
  for (const auto & joint_name : joint_names) {
    const double actual = actual_state->getVariablePosition(joint_name);
    const double expected = expected_state.getVariablePosition(joint_name);
    const double error = std::abs(actual - expected);
    matches = matches && std::isfinite(error) && error <= kStateTolerance;
    RCLCPP_INFO(
      node->get_logger(), "%s_STATE_CHECK: %s expected=%.6f actual=%.6f error=%.6f rad.",
      label.c_str(), joint_name.c_str(), expected, actual, error);
  }
  return matches;
}

bool state_matches_plan_start(
  const rclcpp::Node::SharedPtr & node,
  const moveit::core::RobotStatePtr & actual_state,
  const Plan & plan,
  const std::string & label)
{
  if (!actual_state || !plan_is_valid(plan)) {
    RCLCPP_ERROR(node->get_logger(), "%s_FRESH_STATE FAIL: state or Plan unavailable.", label.c_str());
    return false;
  }
  const auto & trajectory = plan.trajectory_.joint_trajectory;
  bool matches = true;
  for (std::size_t i = 0; i < trajectory.joint_names.size(); ++i) {
    const double actual = actual_state->getVariablePosition(trajectory.joint_names[i]);
    const double expected = trajectory.points.front().positions[i];
    const double error = std::abs(actual - expected);
    matches = matches && std::isfinite(error) && error <= kStateTolerance;
    RCLCPP_INFO(
      node->get_logger(), "%s_FRESH_STATE_CHECK: %s expected=%.6f actual=%.6f error=%.6f rad.",
      label.c_str(), trajectory.joint_names[i].c_str(), expected, actual, error);
  }
  return matches;
}

bool execute_retained_arm_plan(
  const rclcpp::Node::SharedPtr & node, MoveGroup & arm_group,
  Plan & plan, const std::string & label)
{
  if (!state_matches_plan_start(node, arm_group.getCurrentState(5.0), plan, label)) {
    RCLCPP_ERROR(node->get_logger(), "%s_STALE_PLAN REJECTED.", label.c_str());
    return false;
  }
  RCLCPP_WARN(node->get_logger(), "%s_EXECUTE_START: executing exact retained Plan.", label.c_str());
  if (arm_group.execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_ERROR(node->get_logger(), "%s_EXECUTION FAIL.", label.c_str());
    return false;
  }
  RCLCPP_INFO(node->get_logger(), "%s_EXECUTION PASS.", label.c_str());
  return true;
}

bool send_gripper_command(
  const rclcpp::Node::SharedPtr & node,
  const rclcpp_action::Client<GripperCommand>::SharedPtr & client,
  double position, const std::string & label)
{
  if (!client->wait_for_action_server(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "%s FAIL: action server unavailable.", label.c_str());
    return false;
  }
  GripperCommand::Goal goal;
  goal.command.position = position;
  goal.command.max_effort = 0.0;
  auto goal_future = client->async_send_goal(goal);
  if (goal_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
    RCLCPP_ERROR(node->get_logger(), "%s FAIL: acceptance timeout.", label.c_str());
    return false;
  }
  const auto goal_handle = goal_future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(node->get_logger(), "%s FAIL: goal rejected.", label.c_str());
    return false;
  }
  auto result_future = client->async_get_result(goal_handle);
  if (result_future.wait_for(std::chrono::seconds(15)) != std::future_status::ready) {
    RCLCPP_ERROR(node->get_logger(), "%s FAIL: result timeout.", label.c_str());
    return false;
  }
  const GripperGoalHandle::WrappedResult wrapped = result_future.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result) {
    RCLCPP_ERROR(node->get_logger(), "%s FAIL: action did not succeed.", label.c_str());
    return false;
  }
  RCLCPP_INFO(
    node->get_logger(), "%s PASS: command=%.3f result_position=%.6f reached=%s stalled=%s.",
    label.c_str(), position, wrapped.result->position,
    wrapped.result->reached_goal ? "true" : "false",
    wrapped.result->stalled ? "true" : "false");
  return true;
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  const auto node = rclcpp::Node::make_shared(
    "real_pick_place_continuation",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
  bool execute = false;
  bool stop_at_grasp = false;
  node->get_parameter("execute", execute);
  node->get_parameter("stop_at_grasp", stop_at_grasp);
  const auto display_publisher = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
    "/display_planned_path", rclcpp::QoS(1).transient_local().reliable());
  const auto gripper_client = rclcpp_action::create_client<GripperCommand>(
    node, "/robotiq_gripper_controller/gripper_cmd");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() {executor.spin();});
  const auto stop = [&executor, &spin_thread]() {
      executor.cancel();
      if (spin_thread.joinable()) {spin_thread.join();}
      rclcpp::shutdown();
    };

  MoveGroup arm_group(node, "ur_manipulator");
  arm_group.setPoseReferenceFrame("world");
  arm_group.setEndEffectorLink("tool0");
  arm_group.setPlanningTime(5.0);
  arm_group.setNumPlanningAttempts(1);
  arm_group.setMaxVelocityScalingFactor(0.01);
  arm_group.setMaxAccelerationScalingFactor(0.01);

  const auto robot_model = arm_group.getRobotModel();
  const auto actual_start = arm_group.getCurrentState(5.0);
  const auto * arm_jmg = robot_model ? robot_model->getJointModelGroup("ur_manipulator") : nullptr;
  if (!robot_model || !actual_start || !arm_jmg) {
    RCLCPP_ERROR(node->get_logger(), "INITIAL_STATE FAIL: arm model/state unavailable.");
    stop(); return 1;
  }
  const auto arm_joint_names = arm_jmg->getVariableNames();
  const auto home_values = arm_group.getNamedTargetValues("real_home_01");
  if (home_values.size() != arm_joint_names.size()) {
    RCLCPP_ERROR(node->get_logger(), "HOME_TARGET FAIL: real_home_01 unavailable.");
    stop(); return 1;
  }

  moveit::core::RobotState home_state(*actual_start);
  set_named_values(home_state, home_values);
  set_planning_gripper_state(home_state, kOpenCommand);
  const bool starts_at_home = arm_state_matches(
    node, actual_start, home_state, arm_joint_names, "INITIAL_HOME");

  Plan recovery_plan;
  if (!starts_at_home) {
    moveit::core::RobotState recovery_start(*actual_start);
    set_planning_gripper_state(recovery_start, kOpenCommand);
    arm_group.setStartState(recovery_start);
    arm_group.setPlanningPipelineId("ompl");
    arm_group.setPlannerId("");
    if (!arm_group.setJointValueTarget(home_values) ||
      arm_group.plan(recovery_plan) != moveit::core::MoveItErrorCode::SUCCESS ||
      !plan_is_valid(recovery_plan))
    {
      RCLCPP_ERROR(node->get_logger(), "RECOVERY_HOME_PLAN FAIL.");
      stop(); return 1;
    }
    RCLCPP_INFO(node->get_logger(), "STARTUP_BRANCH: recovery to real_home_01 required.");
  } else {
    RCLCPP_INFO(node->get_logger(), "STARTUP_BRANCH: already at real_home_01; recovery skipped.");
  }

  geometry_msgs::msg::PoseStamped pregrasp_target;
  pregrasp_target.header.frame_id = "world";
  pregrasp_target.pose.position.x = 0.346;
  pregrasp_target.pose.position.y = 0.131;
  pregrasp_target.pose.position.z = 0.268;
  constexpr double qx = 0.997, qy = 0.072, qz = 0.009, qw = 0.001;
  const double q_norm = std::sqrt(qx * qx + qy * qy + qz * qz + qw * qw);
  pregrasp_target.pose.orientation.x = qx / q_norm;
  pregrasp_target.pose.orientation.y = qy / q_norm;
  pregrasp_target.pose.orientation.z = qz / q_norm;
  pregrasp_target.pose.orientation.w = qw / q_norm;

  arm_group.setStartState(home_state);
  arm_group.setPlanningPipelineId("ompl");
  arm_group.setPlannerId("");
  arm_group.setPoseTarget(pregrasp_target, "tool0");
  Plan approach_plan;
  if (arm_group.plan(approach_plan) != moveit::core::MoveItErrorCode::SUCCESS ||
    !plan_is_valid(approach_plan))
  {
    RCLCPP_ERROR(node->get_logger(), "PREGRASP_PLAN FAIL."); stop(); return 1;
  }

  moveit::core::RobotState pregrasp_state(home_state);
  apply_plan_endpoint(pregrasp_state, approach_plan);
  set_planning_gripper_state(pregrasp_state, kOpenCommand);
  auto grasp_target = pregrasp_target;
  grasp_target.pose.position.z = 0.208;
  arm_group.setStartState(pregrasp_state);
  arm_group.setPlanningPipelineId("pilz_industrial_motion_planner");
  arm_group.setPlannerId("LIN");
  arm_group.setPoseTarget(grasp_target, "tool0");
  Plan descent_plan;
  if (arm_group.plan(descent_plan) != moveit::core::MoveItErrorCode::SUCCESS ||
    !plan_is_valid(descent_plan))
  {
    RCLCPP_ERROR(node->get_logger(), "DESCENT_PLAN FAIL."); stop(); return 1;
  }

  if (stop_at_grasp) {
    moveit_msgs::msg::DisplayTrajectory display;
    display.model_id = robot_model->getName();
    display.trajectory_start = starts_at_home ? approach_plan.start_state_ : recovery_plan.start_state_;
    set_display_start_gripper_open(display.trajectory_start);
    if (!starts_at_home) {
      display.trajectory.push_back(
        make_display_trajectory(recovery_plan.trajectory_, kOpenCommand));
    }
    display.trajectory.push_back(
      make_display_trajectory(approach_plan.trajectory_, kOpenCommand));
    display.trajectory.push_back(
      make_display_trajectory(descent_plan.trajectory_, kOpenCommand));
    display_publisher->publish(display);
    RCLCPP_INFO(
      node->get_logger(),
      "STOP_AT_GRASP_DISPLAY_PUBLISHED: retained recovery/approach/60 mm descent only; gripper remains open.");

    if (!execute) {
      RCLCPP_INFO(
        node->get_logger(),
        "REAL_STOP_AT_GRASP_PLAN_ONLY PASS: no commands or motion attempted.");
      rclcpp::sleep_for(std::chrono::seconds(3)); stop(); return 0;
    }

    RCLCPP_WARN(
      node->get_logger(),
      "REAL_STOP_AT_GRASP_EXECUTE: moving only to the open-gripper grasp pose at 1%% scaling.");
    if (!starts_at_home &&
      !execute_retained_arm_plan(node, arm_group, recovery_plan, "RECOVERY_HOME"))
    {stop(); return 1;}
    if (!send_gripper_command(node, gripper_client, kOpenCommand, "STARTUP_OPEN"))
    {stop(); return 1;}
    RCLCPP_INFO(
      node->get_logger(), "STARTUP_OPEN_SETTLING: waiting 2 seconds before pregrasp.");
    rclcpp::sleep_for(std::chrono::seconds(2));
    if (!execute_retained_arm_plan(node, arm_group, approach_plan, "PREGRASP") ||
      !execute_retained_arm_plan(node, arm_group, descent_plan, "DESCENT_60MM"))
    {stop(); return 1;}

    RCLCPP_INFO(
      node->get_logger(),
      "REAL_STOP_AT_GRASP PASS: grasp pose reached with gripper open; no close, lift, transfer, release, or final-home command attempted.");
    stop(); return 0;
  }

  moveit::core::RobotState grasp_state(pregrasp_state);
  apply_plan_endpoint(grasp_state, descent_plan);
  set_planning_gripper_state(grasp_state, kCloseCommand);
  arm_group.setStartState(grasp_state);
  arm_group.setPoseTarget(pregrasp_target, "tool0");
  Plan lift_plan;
  if (arm_group.plan(lift_plan) != moveit::core::MoveItErrorCode::SUCCESS ||
    !plan_is_valid(lift_plan))
  {
    RCLCPP_ERROR(node->get_logger(), "LIFT_PLAN FAIL."); stop(); return 1;
  }

  moveit::core::RobotState lifted_state(grasp_state);
  apply_plan_endpoint(lifted_state, lift_plan);
  std::vector<double> transfer_values;
  lifted_state.copyJointGroupPositions(arm_jmg, transfer_values);
  const auto shoulder_it = std::find(
    arm_joint_names.begin(), arm_joint_names.end(), "shoulder_pan_joint");
  if (shoulder_it == arm_joint_names.end()) {
    RCLCPP_ERROR(node->get_logger(), "TRANSFER_TARGET FAIL."); stop(); return 1;
  }
  constexpr double kHalfTurn = 3.14159265358979323846;
  transfer_values[static_cast<std::size_t>(std::distance(arm_joint_names.begin(), shoulder_it))] +=
    kHalfTurn;
  arm_group.clearPoseTargets();
  arm_group.setStartState(lifted_state);
  arm_group.setPlanningPipelineId("ompl");
  arm_group.setPlannerId("");
  if (!arm_group.setJointValueTarget(transfer_values)) {
    RCLCPP_ERROR(node->get_logger(), "TRANSFER_TARGET FAIL."); stop(); return 1;
  }
  Plan transfer_plan;
  if (arm_group.plan(transfer_plan) != moveit::core::MoveItErrorCode::SUCCESS ||
    !plan_is_valid(transfer_plan))
  {
    RCLCPP_ERROR(node->get_logger(), "TRANSFER_PLAN FAIL."); stop(); return 1;
  }

  moveit::core::RobotState transfer_state(lifted_state);
  apply_plan_endpoint(transfer_state, transfer_plan);
  set_planning_gripper_state(transfer_state, kOpenCommand);
  arm_group.setStartState(transfer_state);
  if (!arm_group.setJointValueTarget(home_values)) {
    RCLCPP_ERROR(node->get_logger(), "FINAL_HOME_TARGET FAIL."); stop(); return 1;
  }
  Plan final_home_plan;
  if (arm_group.plan(final_home_plan) != moveit::core::MoveItErrorCode::SUCCESS ||
    !plan_is_valid(final_home_plan))
  {
    RCLCPP_ERROR(node->get_logger(), "FINAL_HOME_PLAN FAIL."); stop(); return 1;
  }

  moveit_msgs::msg::DisplayTrajectory display;
  display.model_id = robot_model->getName();
  display.trajectory_start = starts_at_home ? approach_plan.start_state_ : recovery_plan.start_state_;
  set_display_start_gripper_open(display.trajectory_start);
  if (!starts_at_home) {
    display.trajectory.push_back(
      make_display_trajectory(recovery_plan.trajectory_, kOpenCommand));
  }
  display.trajectory.push_back(
    make_display_trajectory(approach_plan.trajectory_, kOpenCommand));
  display.trajectory.push_back(
    make_display_trajectory(descent_plan.trajectory_, kOpenCommand));
  display.trajectory.push_back(
    make_display_trajectory(lift_plan.trajectory_, kCloseCommand));
  display.trajectory.push_back(
    make_display_trajectory(transfer_plan.trajectory_, kCloseCommand));
  display.trajectory.push_back(
    make_display_trajectory(final_home_plan.trajectory_, kOpenCommand));
  display_publisher->publish(display);
  RCLCPP_INFO(
    node->get_logger(),
    "FULL_SEQUENCE_DISPLAY_PUBLISHED: all six display-only gripper joints encode open/open/open/close/close/open; retained execution Plans unchanged.");

  if (!execute) {
    RCLCPP_INFO(node->get_logger(), "REAL_PICK_PLACE_PLAN_ONLY PASS: no commands or motion attempted.");
    rclcpp::sleep_for(std::chrono::seconds(3)); stop(); return 0;
  }

  RCLCPP_WARN(node->get_logger(), "REAL_PICK_PLACE_EXECUTE: automatic reviewed sequence at 1%% scaling.");
  if (!starts_at_home &&
    !execute_retained_arm_plan(node, arm_group, recovery_plan, "RECOVERY_HOME"))
  {stop(); return 1;}

  // Required startup order: reach/confirm home, open once, wait 2 s, approach.
  if (!send_gripper_command(node, gripper_client, kOpenCommand, "STARTUP_OPEN"))
  {stop(); return 1;}
  RCLCPP_INFO(node->get_logger(), "STARTUP_OPEN_SETTLING: waiting 2 seconds before pregrasp.");
  rclcpp::sleep_for(std::chrono::seconds(2));

  if (!execute_retained_arm_plan(node, arm_group, approach_plan, "PREGRASP") ||
    !execute_retained_arm_plan(node, arm_group, descent_plan, "DESCENT_60MM") ||
    !send_gripper_command(node, gripper_client, kCloseCommand, "GRASP_CLOSE_0643") ||
    !execute_retained_arm_plan(node, arm_group, lift_plan, "LIFT_60MM") ||
    !execute_retained_arm_plan(node, arm_group, transfer_plan, "TRANSFER_PLUS_PI") ||
    !send_gripper_command(node, gripper_client, kOpenCommand, "RELEASE_OPEN"))
  {stop(); return 1;}

  RCLCPP_INFO(node->get_logger(), "RELEASE_SETTLING: waiting 2 seconds before final home.");
  rclcpp::sleep_for(std::chrono::seconds(2));
  if (!execute_retained_arm_plan(node, arm_group, final_home_plan, "FINAL_HOME"))
  {stop(); return 1;}

  RCLCPP_INFO(node->get_logger(), "REAL_PICK_PLACE_CONTINUATION PASS: complete sequence finished.");
  stop(); return 0;
}
