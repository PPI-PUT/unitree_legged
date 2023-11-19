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

#include "unitree_a1_joystick/unitree_a1_joystick_node.hpp"

namespace unitree_a1_joystick
{

double calcMappingdouble(const double input, const double sensitivity)
{
  const double exponent = 1.0 / (std::max(0.001, std::min(1.0, sensitivity)));
  return std::pow(input, exponent);
}

UnitreeA1JoystickNode::UnitreeA1JoystickNode(const rclcpp::NodeOptions & options)
: Node("unitree_a1_joystick", options)
{
  // Parameters
  update_rate_ = this->declare_parameter<double>("update_rate", 50.0);
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
    this, get_clock(), period_ns, std::bind(&UnitreeA1JoystickNode::timerCallback, this));

  // Create publishers
  twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("~/cmd_vel", 1);
  // Create subscribers
  joystick_subscriber_ = this->create_subscription<sensor_msgs::msg::Joy>(
    "/unitree_lowlevel/joy",
    1,
    std::bind(&UnitreeA1JoystickNode::receiveJoystickCallback, this, std::placeholders::_1));
}

void UnitreeA1JoystickNode::receiveJoystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  last_joy_received_time_ = msg->header.stamp;
  joy_ = std::make_shared<UnitreeA1Joystick>(*msg);
}

void UnitreeA1JoystickNode::publishTwist()
{
  auto twist_msg = geometry_msgs::msg::TwistStamped();
  twist_msg.header.stamp = this->now();
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
  twist_publisher_->publish(twist_msg);
}

void UnitreeA1JoystickNode::timerCallback()
{
  if (!isDataReady()) {
    return;
  }
  publishTwist();
}

bool UnitreeA1JoystickNode::isDataReady()
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

}  // namespace unitree_a1_joystick

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(unitree_a1_joystick::UnitreeA1JoystickNode)
