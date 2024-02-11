// Copyright 2023 Maciej Krupka
// Perception for Physical Interaction Laboratory at Poznan University of Technology
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "unitree_a1_joystick/unitree_a1_joystick_node.hpp"

namespace unitree_a1_legged
{
UnitreeJoystickNode::UnitreeJoystickNode(const rclcpp::NodeOptions & options)
: Node("unitree_a1_joystick", options)
{
  // Parameters
  linear_x_velocity_ = 0.0;
  update_rate_ = this->declare_parameter<int>("update_rate", 50);
  double dir_button_deadzone = this->declare_parameter<double>("dir_button_deadzone", 0.3);
  double button_deadzone = this->declare_parameter<double>("button_deadzone", 1.0);
  joy_or_dir_button_ = this->declare_parameter<int>("joy_or_dir_button", 1);
  velocity_increment_ = this->declare_parameter<double>("velocity_increment", 0.05);
  linear_ratio_ = this->declare_parameter<double>("linear_ratio", 0.5);
  angular_ratio_ = this->declare_parameter<double>("angular_ratio", 0.5);
  linear_x_sensitivity_ = this->declare_parameter<double>("linear_x_sensitivity", 0.5);
  linear_y_sensitivity_ = this->declare_parameter<double>("linear_y_sensitivity", 0.5);
  linear_velocity_limit_ = this->declare_parameter<double>("linear_velocity_limit", 0.5);
  angular_z_sensitivity_ = this->declare_parameter<double>("angular_z_sensitivity", 0.5);
  angular_velocity_limit_ = this->declare_parameter<double>("angular_velocity_limit", 0.5);
  // Create timer
  const auto period_ns =
    std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(
      1.0 /
      update_rate_));
  timer_ = rclcpp::create_timer(
    this, get_clock(), period_ns, std::bind(&UnitreeJoystickNode::timerCallback, this));
  // Deadzone time
  int32_t sec;
  uint32_t nanosec;
  // Create buttons
  std::tie(sec, nanosec) = getDurationTime(dir_button_deadzone);
  direction_x_button_ = std::make_unique<Button>(
    sec, nanosec, velocity_increment_, 0.01, -linear_velocity_limit_,
    linear_velocity_limit_);
  direction_y_button_ = std::make_unique<Button>(
    sec, nanosec, velocity_increment_, 0.01, -linear_velocity_limit_,
    linear_velocity_limit_);
  direction_z_button_ = std::make_unique<Button>(
    sec, nanosec, velocity_increment_, 0.01, -linear_velocity_limit_,
    linear_velocity_limit_);
  std::tie(sec, nanosec) = getDurationTime(button_deadzone);
  button_a_stand_ = std::make_unique<Button>(
    sec, nanosec);
  button_b_walking_ = std::make_unique<Button>(
    sec, nanosec);
  button_x_stop_ = std::make_unique<Button>(
    0, nanosec);
  // Create publishers
  twist_publisher_ =
    this->create_publisher<geometry_msgs::msg::TwistStamped>("~/output/cmd_vel", 1);
  // Create subscribers
  joystick_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "~/input/joy", 1, std::bind(
      &UnitreeJoystickNode::receiveJoystickCallback, this,
      std::placeholders::_1));
  client_gait_ = this->create_client<unitree_a1_legged_msgs::srv::Gait>(
    "~/service/gait");
}

void UnitreeJoystickNode::receiveJoystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  last_joy_received_time_ = msg->header.stamp;
  joy_ = std::make_shared<UnitreeJoystick>(*msg);
}

void UnitreeJoystickNode::publishTwist()
{
  auto twist_msg = geometry_msgs::msg::TwistStamped();
  twist_msg.header.frame_id = "base";
  twist_msg.header.stamp = this->now();
  if (joy_or_dir_button_ == 0) {
    if (joy_->linear_x()) {
      twist_msg.twist.linear.x =
        linear_ratio_ *
        calcMappingdouble(static_cast<double>(joy_->linear_x()), linear_x_sensitivity_);
    }
    if (joy_->linear_y()) {
      twist_msg.twist.linear.y =
        linear_ratio_ *
        calcMappingdouble(static_cast<double>(joy_->linear_y()), linear_y_sensitivity_);
    }
    if (joy_->angular_z()) {
      twist_msg.twist.angular.z =
        angular_ratio_ * calcMappingdouble(
        static_cast<double>(joy_->angular_z()), angular_z_sensitivity_);
    }
  } else if (joy_or_dir_button_ == 1) {
    this->directionalButtonsCallback();
    twist_msg.twist.linear.x = direction_x_button_->getValue();
    twist_msg.twist.linear.y = direction_y_button_->getValue();
    twist_msg.twist.angular.z = direction_z_button_->getValue();
  } else if (joy_or_dir_button_ == -1) {
    return;
  }
  twist_publisher_->publish(twist_msg);
}

void UnitreeJoystickNode::indexButtonCallback()
{
  rclcpp::Time now = this->get_clock()->now();
  if (button_x_stop_->getAction(joy_->stop(), now, 0.0)) {
    RCLCPP_INFO(get_logger(), "STOP");
    this->indexButtonRequest(unitree_a1_legged_msgs::msg::ControllerType::STOP);
    return;
  }
  if (button_a_stand_->getAction(joy_->stand(), now, 0.0)) {
    RCLCPP_INFO(get_logger(), "Stand");
    this->indexButtonRequest(unitree_a1_legged_msgs::msg::ControllerType::FIXED_STAND);
    return;
  }
  if (button_b_walking_->getAction(joy_->neural(), now, 0.0)) {
    RCLCPP_INFO(get_logger(), "Neural Controller Walking");
    this->indexButtonRequest(unitree_a1_legged_msgs::msg::ControllerType::NEURAL);
    return;
  }
}

void UnitreeJoystickNode::indexButtonRequest(uint8_t type)
{
  auto request = std::make_shared<unitree_a1_legged_msgs::srv::Gait::Request>();
  auto msg = unitree_a1_legged_msgs::msg::ControllerType();
  msg.type = type;
  request->method = msg;
  client_gait_->async_send_request(request);
}

void UnitreeJoystickNode::directionalButtonsCallback()
{
  this->directionButton(joy_->increase_linear_x(), joy_->decrease_linear_x(), direction_x_button_);
  this->directionButton(joy_->increase_linear_y(), joy_->decrease_linear_y(), direction_y_button_);
  this->directionButton(
    joy_->increase_angular_z(), joy_->decrease_angular_z(),
    direction_z_button_);
}
void UnitreeJoystickNode::directionButton(bool increase, bool decrease, ButtonPtr & button)
{
  rclcpp::Time now = this->get_clock()->now();
  if (increase ^ decrease) {
    if (increase) {
      button->getAction(true, now, 1.0);
    } else if (decrease) {
      button->getAction(true, now, -1.0);
    }
  } else {
    button->getAction(false, now, 0.0);
  }
}

void UnitreeJoystickNode::timerCallback()
{
  if (!isDataReady()) {
    return;
  }
  publishTwist();
  indexButtonCallback();
}

bool UnitreeJoystickNode::isDataReady()
{
  if (!joy_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(),
      "waiting for joy msg...");
    return false;
  }
  constexpr auto timeout = 2.0;
  const auto time_diff = this->now() - last_joy_received_time_;
  if (time_diff.seconds() > timeout) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), std::chrono::milliseconds(5000).count(), "joy msg is timeout");
    return false;
  }
  return true;
}

std::tuple<int32_t, uint32_t> UnitreeJoystickNode::getDurationTime(double duration)
{
  int32_t sec = static_cast<int32_t>(duration);
  uint32_t nanosec = static_cast<uint32_t>((duration - sec) * 1e9);
  return std::make_tuple(sec, nanosec);
}

} // namespace unitree_a1_legged

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(unitree_a1_legged::UnitreeJoystickNode)