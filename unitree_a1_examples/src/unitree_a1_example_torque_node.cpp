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

#include "unitree_a1_examples/unitree_a1_example_torque_node.hpp"

using namespace UNITREE_LEGGED_SDK;

namespace unitree_a1_examples
{

UnitreeExampleTorqueNode::UnitreeExampleTorqueNode(const rclcpp::NodeOptions & options)
:  Node("unitree_torque_control", options)
{
  unitree_a1_examples_ = std::make_unique<unitree_a1_examples::UnitreeA1Examples>();
  state_ = this->create_subscription<unitree_a1_legged_msgs::msg::LowState>(
    "~/input/state", 1,
    std::bind(&UnitreeExampleTorqueNode::stateCallback, this, std::placeholders::_1));
  cmd_ = this->create_publisher<unitree_a1_legged_msgs::msg::LowCmd>("~/output/command", 1);
  pub_ = this->create_publisher<std_msgs::msg::Float64>("~/output/torque", 1);
}

void UnitreeExampleTorqueNode::stateCallback(unitree_a1_legged_msgs::msg::LowState::SharedPtr msg)
{
  double value = amplitude_ * sin(2.0 * M_PI * frequency_ * motiontime_ / 1000.0);
  motiontime_++;
  unitree_a1_legged_msgs::msg::LowCmd cmd;
  cmd.common.mode = 0x0A;       // motor switch to servo (PMSM) mode
  cmd.motor_cmd.front_right.hip.q = a1_Hip_min;
  cmd.motor_cmd.front_right.hip.dq = 0.0;
  cmd.motor_cmd.front_right.hip.tau = 0.0;
  cmd.motor_cmd.front_right.hip.kp = 10.0;
  cmd.motor_cmd.front_right.hip.kd = 1.0;
  cmd.motor_cmd.front_right.calf.q = a1_Calf_min;
  cmd.motor_cmd.front_right.calf.dq = 0.0;
  cmd.motor_cmd.front_right.calf.tau = 0.0;
  cmd.motor_cmd.front_right.calf.kp = 10.0;
  cmd.motor_cmd.front_right.calf.kd = 1.0;
  cmd.motor_cmd.front_right.thigh.q = PosStopF;
  cmd.motor_cmd.front_right.thigh.dq = VelStopF;
  cmd.motor_cmd.front_right.thigh.tau = value;
  if (msg->motor_state.front_right.thigh.q >= a1_Thigh_max - 0.5) {
    return;
  }
  if (msg->motor_state.front_right.thigh.q <= a1_Thigh_min + 0.5) {
    return;
  }
  cmd_->publish(cmd);
  auto msg_ = std_msgs::msg::Float64();
  msg_.data = value;
  pub_->publish(msg_);
}

}  // namespace unitree_a1_examples

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(unitree_a1_examples::UnitreeExampleTorqueNode)
