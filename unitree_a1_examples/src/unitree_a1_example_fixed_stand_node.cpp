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

#include "unitree_a1_examples/unitree_a1_example_fixed_stand_node.hpp"

namespace unitree_a1_examples
{

UnitreeFixedStandNode::UnitreeFixedStandNode(const rclcpp::NodeOptions & options)
:  Node("unitree_fixed_stand", options)
{
  hip_kp_= this->declare_parameter("hip_kp", 70.0);
  hip_kd_= this->declare_parameter("hip_kd", 3.0);
  thigh_kp_= this->declare_parameter("thigh_kp", 180.0);
  thigh_kd_= this->declare_parameter("thigh_kd", 8.0);
  calf_kp_= this->declare_parameter("calf_kp", 300.0);
  calf_kd_= this->declare_parameter("calf_kd", 15.0);
  steps_= this->declare_parameter("steps", 2000.0);
  unitree_a1_examples_ = std::make_unique<unitree_a1_examples::UnitreeA1Examples>();
  state_ = this->create_subscription<unitree_a1_legged_msgs::msg::LowState>(
    "~/input/state", 1, std::bind(
      &UnitreeFixedStandNode::stateCallback, this,
      std::placeholders::_1));
  cmd_ = this->create_publisher<unitree_a1_legged_msgs::msg::LowCmd>("~/output/command", 1);
}

void UnitreeFixedStandNode::stateCallback(unitree_a1_legged_msgs::msg::LowState::SharedPtr msg)
{
  if (init_) {
    lastPos_[0] = msg->motor_state.front_right.hip.q;
    lastPos_[1] = msg->motor_state.front_right.thigh.q;
    lastPos_[2] = msg->motor_state.front_right.calf.q;
    lastPos_[3] = msg->motor_state.front_left.hip.q;
    lastPos_[4] = msg->motor_state.front_left.thigh.q;
    lastPos_[5] = msg->motor_state.front_left.calf.q;
    lastPos_[6] = msg->motor_state.rear_right.hip.q;
    lastPos_[7] = msg->motor_state.rear_right.thigh.q;
    lastPos_[8] = msg->motor_state.rear_right.calf.q;
    lastPos_[9] = msg->motor_state.rear_left.hip.q;
    lastPos_[10] = msg->motor_state.rear_left.thigh.q;
    lastPos_[11] = msg->motor_state.rear_left.calf.q;
    init_ = false;
  }
  this->initParam(cmd_msg_);
  if (motiontime_ < steps_) {
    percent_ = static_cast<float>(motiontime_) / steps_;
    for (size_t i = 0; i < targetPos_.size(); i++) {
      targetPos_[i] = unitree_a1_examples_->jointLinearInterpolation(lastPos_[i], standPos_[i], percent_);
    }
    RCLCPP_INFO(this->get_logger(), "percent: %f, targetPos: %f", percent_, targetPos_[0]);

    motiontime_++;

    cmd_msg_.motor_cmd.front_right.hip.q = targetPos_[0];
    cmd_msg_.motor_cmd.front_right.thigh.q = targetPos_[1];
    cmd_msg_.motor_cmd.front_right.calf.q = targetPos_[2];
    cmd_msg_.motor_cmd.front_left.hip.q = targetPos_[3];
    cmd_msg_.motor_cmd.front_left.thigh.q = targetPos_[4];
    cmd_msg_.motor_cmd.front_left.calf.q = targetPos_[5];
    cmd_msg_.motor_cmd.rear_right.hip.q = targetPos_[6];
    cmd_msg_.motor_cmd.rear_right.thigh.q = targetPos_[7];
    cmd_msg_.motor_cmd.rear_right.calf.q = targetPos_[8];
    cmd_msg_.motor_cmd.rear_left.hip.q = targetPos_[9];
    cmd_msg_.motor_cmd.rear_left.thigh.q = targetPos_[10];
    cmd_msg_.motor_cmd.rear_left.calf.q = targetPos_[11];
  }
  cmd_->publish(cmd_msg_);

  return;
}

void UnitreeFixedStandNode::initParam(unitree_a1_legged_msgs::msg::LowCmd & cmd_msg)
{
  cmd_msg.common.mode = 0x0A;
  cmd_msg.motor_cmd.front_right.hip.kp = hip_kp_;
  cmd_msg.motor_cmd.front_right.hip.kd = hip_kd_;
  cmd_msg.motor_cmd.front_left.hip.kp = hip_kp_;
  cmd_msg.motor_cmd.front_left.hip.kd = hip_kd_;
  cmd_msg.motor_cmd.rear_right.hip.kp = hip_kp_;
  cmd_msg.motor_cmd.rear_right.hip.kd = hip_kd_;
  cmd_msg.motor_cmd.rear_left.hip.kp = hip_kp_;
  cmd_msg.motor_cmd.rear_left.hip.kd = hip_kd_;

  cmd_msg.motor_cmd.front_right.thigh.kp = thigh_kp_;
  cmd_msg.motor_cmd.front_right.thigh.kd = thigh_kd_;
  cmd_msg.motor_cmd.front_left.thigh.kp = thigh_kp_;
  cmd_msg.motor_cmd.front_left.thigh.kd = thigh_kd_;
  cmd_msg.motor_cmd.rear_right.thigh.kp = thigh_kp_;
  cmd_msg.motor_cmd.rear_right.thigh.kd = thigh_kd_;
  cmd_msg.motor_cmd.rear_left.thigh.kp = thigh_kp_;
  cmd_msg.motor_cmd.rear_left.thigh.kd = thigh_kd_;

  cmd_msg.motor_cmd.front_right.calf.kp = calf_kp_;
  cmd_msg.motor_cmd.front_right.calf.kd = calf_kd_;
  cmd_msg.motor_cmd.front_left.calf.kp = calf_kp_;
  cmd_msg.motor_cmd.front_left.calf.kd = calf_kd_;
  cmd_msg.motor_cmd.rear_right.calf.kp = calf_kp_;
  cmd_msg.motor_cmd.rear_right.calf.kd = calf_kd_;
  cmd_msg.motor_cmd.rear_left.calf.kp = calf_kp_;
  cmd_msg.motor_cmd.rear_left.calf.kd = calf_kd_;
}
}  // namespace unitree_a1_examples

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(unitree_a1_examples::UnitreeFixedStandNode)
