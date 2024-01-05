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

#ifndef UNITREE_A1_JOYSTICK__UNITREE_A1_JOYSTICK_NODE_HPP_
#define UNITREE_A1_JOYSTICK__UNITREE_A1_JOYSTICK_NODE_HPP_

#include "unitree_a1_joystick/unitree_a1_joystick.hpp"
#include "unitree_a1_legged_msgs/srv/gait.hpp"
#include "unitree_a1_legged_msgs/msg/controller_type.hpp"
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <memory>
#include <tuple>
#include <rclcpp/rclcpp.hpp>

double calcMappingdouble(const double input, const double sensitivity)
{
  const double exponent = 1.0 / (std::max(0.001, std::min(1.0, sensitivity)));
  if (input < 0.0) {
    return -std::pow(input, exponent);
  }
  return std::pow(input, exponent);
}

namespace unitree_a1_legged
{
class UNITREE_A1_JOYSTICK_PUBLIC UnitreeJoystickNode : public rclcpp::Node
{
public:
  explicit UnitreeJoystickNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  double update_rate_;
  double linear_ratio_;
  double angular_ratio_;
  double linear_x_velocity_;
  double linear_x_sensitivity_;
  double linear_y_sensitivity_;
  double linear_velocity_limit_;
  double angular_z_sensitivity_;
  double angular_velocity_limit_;
  double velocity_increment_;
  int joy_or_dir_button_;
  rclcpp::Duration button_hold_duration_ = rclcpp::Duration(0, 0);
  rclcpp::Duration dir_button_hold_duration_ = rclcpp::Duration(0, 0);
  rclcpp::Duration button_hold_deadzone_ = rclcpp::Duration(0, 0);
  rclcpp::Duration dir_button_hold_deadzone_ = rclcpp::Duration(0, 0);
  rclcpp::Time last_time_button_pressed_;
  rclcpp::Time last_time_dir_button_pressed_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joystick_subscriber_;
  rclcpp::Time last_joy_received_time_;
  std::shared_ptr<UnitreeJoystick> joy_;
  rclcpp::Client<unitree_a1_legged_msgs::srv::Gait>::SharedPtr client_gait_;
  void receiveJoystickCallback(const sensor_msgs::msg::Joy::SharedPtr msg);
  void publishTwist();
  void indexButtonCallback();
  int directionalButtons();
  void timerCallback();
  bool isDataReady();
  std::tuple<int32_t, uint32_t> getDurationTime(double duration);
};
} // namespace unitree_a1_legged

#endif // UNITREE_A1_JOYSTICK__UNITREE_A1_JOYSTICK_NODE_HPP_
