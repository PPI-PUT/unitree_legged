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

#include "unitree_a1_examples/unitree_a1_example_position_node.hpp"

namespace unitree_a1_examples
{

UnitreeExamplePositionNode::UnitreeExamplePositionNode(const rclcpp::NodeOptions & options)
:  Node("example_position_control", options)
{
  unitree_a1_examples_ = std::make_unique<unitree_a1_examples::UnitreeA1Examples>();
  state_ =
    this->create_subscription<unitree_a1_legged_msgs::msg::LowState>(
    "~/state", 1,
    std::bind(&UnitreeExamplePositionNode::stateCallback, this, std::placeholders::_1));
  cmd_ = this->create_publisher<unitree_a1_legged_msgs::msg::LowCmd>("~/command", 1);
  // unitree_a1_examples_->foo(param_name_);
}

void UnitreeExamplePositionNode::stateCallback(unitree_a1_legged_msgs::msg::LowState::SharedPtr msg)
{
  motiontime++;
  unitree_a1_legged_msgs::msg::LowCmd cmd;
  cmd.common.mode = 0x0A;       // motor switch to servo (PMSM) mode
  cmd.motor_cmd.front_right.hip.tau = -0.65f;
  cmd.motor_cmd.front_left.hip.tau = +0.65f;
  cmd.motor_cmd.rear_right.hip.tau = -0.65f;
  cmd.motor_cmd.rear_left.hip.tau = +0.65f;

  if (motiontime >= 0) {
    // first, get record initial position
    // if( motiontime >= 100 && motiontime < 500){
    if (motiontime >= 0 && motiontime < 10) {
      qInit[0] = msg->motor_state.front_right.hip.q;
      qInit[1] = msg->motor_state.front_right.thigh.q;
      qInit[2] = msg->motor_state.front_right.calf.q;
    }
    // second, move to the origin point of a sine movement with Kp Kd
    // if( motiontime >= 500 && motiontime < 1500){
    if (motiontime >= 10 && motiontime < 400) {
      rate_count++;
      double rate = rate_count / 200.0;           // needs count to 200
      Kp[0] = 5.0;
      Kp[1] = 5.0;
      Kp[2] = 5.0;
      Kd[0] = 1.0;
      Kd[1] = 1.0;
      Kd[2] = 1.0;

      qDes[0] = unitree_a1_examples_->jointLinearInterpolation(qInit[0], sin_mid_q[0], rate);
      qDes[1] = unitree_a1_examples_->jointLinearInterpolation(qInit[1], sin_mid_q[1], rate);
      qDes[2] = unitree_a1_examples_->jointLinearInterpolation(qInit[2], sin_mid_q[2], rate);
    }

    // last, do sine wave
    if (motiontime >= 400) {
      sin_count++;
      qDes[0] = sin_mid_q[0];
      qDes[1] = sin_mid_q[1];
      qDes[2] = sin_mid_q[2] - 0.6 * sin(1.8 * M_PI * sin_count / 1000.0);
      // qDes[2] = sin_mid_q[2];
    }
  }
  cmd.motor_cmd.front_left.hip.q = qDes[0];
  cmd.motor_cmd.front_left.hip.dq = 0;
  cmd.motor_cmd.front_left.hip.kp = Kp[0];
  cmd.motor_cmd.front_left.hip.kd = Kd[0];
  cmd.motor_cmd.front_left.hip.tau = -0.65f;

  cmd.motor_cmd.front_right.thigh.q = qDes[1];
  cmd.motor_cmd.front_right.thigh.dq = 0;
  cmd.motor_cmd.front_right.thigh.kp = Kp[1];
  cmd.motor_cmd.front_right.thigh.kd = Kd[1];
  cmd.motor_cmd.front_right.thigh.tau = 0.0f;

  cmd.motor_cmd.front_right.calf.q = qDes[2];
  cmd.motor_cmd.front_right.calf.dq = 0;
  cmd.motor_cmd.front_right.calf.kp = Kp[2];
  cmd.motor_cmd.front_right.calf.kd = Kd[2];
  cmd.motor_cmd.front_right.calf.tau = 0.0f;
  cmd_->publish(cmd);
}


}  // namespace unitree_a1_examples

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(unitree_a1_examples::UnitreeExamplePositionNode)
