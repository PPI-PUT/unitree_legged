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

#ifndef UNITREE_A1_JOYSTICK__UNITREE_A1_JOYSTICK_NODE_HPP_
#define UNITREE_A1_JOYSTICK__UNITREE_A1_JOYSTICK_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "unitree_a1_joystick/unitree_a1_joystick.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
namespace unitree_a1_joystick
{
using UnitreeA1JoystickPtr = std::unique_ptr<unitree_a1_joystick::UnitreeA1Joystick>;

class UNITREE_A1_JOYSTICK_PUBLIC UnitreeA1JoystickNode : public rclcpp::Node
{
public:
  explicit UnitreeA1JoystickNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  double update_rate_;
  double linear_ratio_;
  double angular_ratio_;
  double linear_x_sensitivity_;
  double linear_y_sensitivity_;
  double linear_velocity_limit_;
  double angular_z_sensitivity_;
  double angular_velocity_limit_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscriber_;
  rclcpp::Time last_joy_received_time_;
  std::shared_ptr<UnitreeA1Joystick> joy_;
  void receiveJoystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void publishTwist();
  void timerCallback();
  bool isDataReady();
};
}  // namespace unitree_a1_joystick

#endif  // UNITREE_A1_JOYSTICK__UNITREE_A1_JOYSTICK_NODE_HPP_
