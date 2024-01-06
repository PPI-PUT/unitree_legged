// Copyright 2023 Maciej Krupka
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "unitree_a1_legged/unitree_a1_legged_node.hpp"

namespace unitree_a1_legged
{

UnitreeLeggedNode::UnitreeLeggedNode(const rclcpp::NodeOptions & options)
: Node("unitree_lowlevel", options)
{
  bool hot_start = this->declare_parameter("hot_start", false);
  hot_start = this->get_parameter("hot_start").as_bool();
  safety_factor_ = this->declare_parameter("safety_factor", 1);
  safety_factor_ = this->get_parameter("safety_factor").as_int();
  if (hot_start) {
    RCLCPP_WARN(this->get_logger(), "Starting Unitree Legged Node");
    RCLCPP_WARN(
      this->get_logger(),
      "Make sure the robot is lied on the ground and the motors are turned on.");
    RCLCPP_WARN(this->get_logger(), "Press L1 and hold A to sit down the robot.");
    RCLCPP_WARN(this->get_logger(), "Press L1+L2 and A to turn on joint control mode.");
    RCLCPP_WARN(this->get_logger(), "Press Enter to continue...");
    std::cin.get();
  }
  // Init motor Mode
  unitree_.setMotorMode(PMSM_SERVO_MODE);
  // state_thread_ = std::thread(std::bind(&UnitreeLeggedNode::updateLoop, this));
  state_publisher_ = this->create_publisher<unitree_a1_legged_msgs::msg::LowState>(
    "~/output/state",
    1);
  joint_state_publisher_ =
    this->create_publisher<sensor_msgs::msg::JointState>("~/output/joint_states", 1);
  joystick_publisher_ = this->create_publisher<sensor_msgs::msg::Joy>("~/output/joy", 1);
  imu_publisher_ = this->create_publisher<sensor_msgs::msg::Imu>("~/output/imu", 1);
  joint_state_subscriber_ = this->create_subscription<unitree_a1_legged_msgs::msg::JointCommand>(
    "~/input/joint_command", 1,
    std::bind(&UnitreeLeggedNode::receiveJointCommandCallback, this, std::placeholders::_1));
  low_command_subscriber_ = this->create_subscription<unitree_a1_legged_msgs::msg::LowCmd>(
    "~/input/command", 1,
    std::bind(&UnitreeLeggedNode::receiveCommandCallback, this, std::placeholders::_1));
  timer_ = this->create_wall_timer(2ms, std::bind(&UnitreeLeggedNode::updateStateCallback, this));
  foot_force_fr_publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "~/output/fr_contact", 1);
  foot_force_fl_publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "~/output/fl_contact", 1);
  foot_force_rr_publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "~/output/rr_contact", 1);
  foot_force_rl_publisher_ = this->create_publisher<geometry_msgs::msg::WrenchStamped>(
    "~/output/rl_contact", 1);
}
UnitreeLeggedNode::~UnitreeLeggedNode()
{
  if (state_thread_.joinable()) {
    state_thread_.join();
  }
}

void UnitreeLeggedNode::updateLoop()
{
  while (rclcpp::ok()) {
    unitree_a1_legged_msgs::msg::LowState low_state_ros;
    unitree_.recvLowState();
    low_state_ros = Converter::stateToMsg(unitree_.getLowState());
    state_publisher_->publish(low_state_ros);
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
}
void UnitreeLeggedNode::receiveCommandCallback(
  const unitree_a1_legged_msgs::msg::LowCmd::SharedPtr msg)
{
  // Convert to lowlevel cmd
  auto low_cmd_once = unitree_.getLowCmd();
  Converter::msgToCmd(msg, low_cmd_once);
  unitree_.sendProtectLowCmd(low_cmd_once, safety_factor_);
}
void UnitreeLeggedNode::receiveJointCommandCallback(
  const unitree_a1_legged_msgs::msg::JointCommand::SharedPtr msg)
{
  // Convert to lowlevel cmd
  auto low_cmd_once = unitree_.getLowCmd();
  Converter::msgToCmd(msg, low_cmd_once);
  unitree_.sendProtectLowCmd(low_cmd_once, safety_factor_);
}
void UnitreeLeggedNode::updateStateCallback()
{
  // Get state
  unitree_.recvLowState();
  auto stamp = this->now();
  // Convert to lowlevel state msgs
  auto low_state_ros = Converter::stateToMsg(unitree_.getLowState());
  low_state_ros.header.stamp = stamp;
  state_publisher_->publish(low_state_ros);
  // Convert to joint state msgs
  auto joint_state_msg = Converter::getJointStateMsg(unitree_.getLowState());
  joint_state_msg.header.stamp = stamp;
  joint_state_publisher_->publish(joint_state_msg);
  // Convert to joy msgs
  auto joy_msg = Converter::stateToMsg(unitree_.getLowState().wirelessRemote);
  joy_msg.header.stamp = stamp;
  joystick_publisher_->publish(joy_msg);
  // Convert to imu msgs
  auto imu_msg = Converter::stateToMsg(unitree_.getLowState().imu);
  imu_msg.header.stamp = stamp;
  imu_publisher_->publish(imu_msg);
  // Convert to foot force msgs
  geometry_msgs::msg::WrenchStamped fr_msg, fl_msg, rr_msg, rl_msg;
  fr_msg.header.stamp = stamp;
  fl_msg.header.stamp = stamp;
  rr_msg.header.stamp = stamp;
  rl_msg.header.stamp = stamp;
  Converter::getWrenchMsg(unitree_.getLowState().footForce, fr_msg, fl_msg, rr_msg, rl_msg);
  foot_force_fr_publisher_->publish(fr_msg);
  foot_force_fl_publisher_->publish(fl_msg);
  foot_force_rr_publisher_->publish(rr_msg);
  foot_force_rl_publisher_->publish(rl_msg);
}

}  // namespace unitree_a1_legged

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(unitree_a1_legged::UnitreeLeggedNode)
