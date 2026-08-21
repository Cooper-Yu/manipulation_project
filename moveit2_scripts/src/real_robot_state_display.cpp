#include <algorithm>
#include <chrono>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

class RealRobotStateDisplay : public rclcpp::Node
{
public:
  RealRobotStateDisplay()
  : Node("real_robot_state_display")
  {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    arm_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", qos,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        std::scoped_lock lock(mutex_);
        for (std::size_t i = 0; i < message->name.size() && i < message->position.size(); ++i) {
          if (std::find(arm_joint_names_.begin(), arm_joint_names_.end(), message->name[i]) !=
            arm_joint_names_.end())
          {
            arm_positions_[message->name[i]] = message->position[i];
          }
        }
        arm_stamp_ = now();
      });

    gripper_subscription_ = create_subscription<sensor_msgs::msg::JointState>(
      "/gripper/joint_states", qos,
      [this](sensor_msgs::msg::JointState::ConstSharedPtr message) {
        const auto joint = std::find(
          message->name.begin(), message->name.end(), "robotiq_85_left_knuckle_joint");
        if (joint == message->name.end()) {
          return;
        }
        const auto index = static_cast<std::size_t>(std::distance(message->name.begin(), joint));
        if (index >= message->position.size()) {
          return;
        }
        std::scoped_lock lock(mutex_);
        gripper_position_ = message->position[index];
        gripper_stamp_ = now();
        have_gripper_ = true;
      });

    publisher_ = create_publisher<moveit_msgs::msg::DisplayRobotState>(
      "/real_robot_state_display", rclcpp::QoS(1).reliable());
    timer_ = create_wall_timer(
      std::chrono::milliseconds(50), std::bind(&RealRobotStateDisplay::publish_state, this));

    RCLCPP_INFO(
      get_logger(),
      "REAL_ROBOT_STATE_DISPLAY_READY: merging six arm joints from /joint_states with real gripper from /gripper/joint_states.");
  }

private:
  void publish_state()
  {
    std::scoped_lock lock(mutex_);
    if (arm_positions_.size() != arm_joint_names_.size() || !have_gripper_) {
      return;
    }
    constexpr double kFreshSeconds = 1.0;
    if ((now() - arm_stamp_).seconds() > kFreshSeconds ||
      (now() - gripper_stamp_).seconds() > kFreshSeconds)
    {
      return;
    }

    moveit_msgs::msg::DisplayRobotState display;
    display.state.joint_state.header.stamp = now();
    for (const auto & joint_name : arm_joint_names_) {
      display.state.joint_state.name.push_back(joint_name);
      display.state.joint_state.position.push_back(arm_positions_.at(joint_name));
    }

    const std::map<std::string, double> gripper_positions = {
      {"robotiq_85_left_knuckle_joint", gripper_position_},
      {"robotiq_85_right_knuckle_joint", gripper_position_},
      {"robotiq_85_left_inner_knuckle_joint", gripper_position_},
      {"robotiq_85_right_inner_knuckle_joint", gripper_position_},
      {"robotiq_85_left_finger_tip_joint", -gripper_position_},
      {"robotiq_85_right_finger_tip_joint", -gripper_position_},
    };
    for (const auto & [joint_name, position] : gripper_positions) {
      display.state.joint_state.name.push_back(joint_name);
      display.state.joint_state.position.push_back(position);
    }
    display.state.is_diff = false;
    publisher_->publish(display);
  }

  const std::vector<std::string> arm_joint_names_ = {
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
  };
  std::mutex mutex_;
  std::map<std::string, double> arm_positions_;
  double gripper_position_{0.0};
  bool have_gripper_{false};
  rclcpp::Time arm_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time gripper_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_subscription_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr gripper_subscription_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayRobotState>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RealRobotStateDisplay>());
  rclcpp::shutdown();
  return 0;
}
