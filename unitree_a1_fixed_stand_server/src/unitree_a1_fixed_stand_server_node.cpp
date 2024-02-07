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

#include "unitree_a1_fixed_stand_server/unitree_a1_fixed_stand_server_node.hpp"

using namespace std::placeholders;

namespace unitree_a1_fixed_stand_server
{

UnitreeFixedStandServerNode::UnitreeFixedStandServerNode(const rclcpp::NodeOptions & options)
:  Node("unitree_fixed_stand_server", options)
{
  state_ = std::make_shared<LowState>();
  request_fixed_stand_rate_ = this->declare_parameter("request_fixed_stand_rate", 50);
  auto qos = rclcpp::QoS(1);
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos.durability_volatile();
  sub_state_ = this->create_subscription<LowState>(
    "~/input/state", qos,
    std::bind(&UnitreeFixedStandServerNode::stateCallback, this, _1));
  pub_cmd_ = this->create_publisher<LowCmd>("~/output/cmd", 1);

  unitree_a1_fixed_stand_server_ =
    std::make_unique<unitree_a1_fixed_stand_server::UnitreeFixedStandServer>();
  fixed_stand_server_ = rclcpp_action::create_server<FixedStand>(
    this,
    "~/action/fixed_stand",
    std::bind(
      &UnitreeFixedStandServerNode::handleFixedStandGoal, this, _1, _2),
    std::bind(
      &UnitreeFixedStandServerNode::handleFixedStandCancel, this, _1),
    std::bind(
      &UnitreeFixedStandServerNode::handleAcceptedFixedStand, this, _1));
}

rclcpp_action::GoalResponse UnitreeFixedStandServerNode::handleFixedStandGoal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const FixedStand::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->start);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse UnitreeFixedStandServerNode::handleFixedStandCancel(
  const std::shared_ptr<FixedStandGoal> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  // send 0 to all motors
  // reset fixed stand
  return rclcpp_action::CancelResponse::ACCEPT;
}

void UnitreeFixedStandServerNode::handleAcceptedFixedStand(
  const std::shared_ptr<FixedStandGoal> goal_handle)
{
  std::thread{std::bind(
      &UnitreeFixedStandServerNode::executeFixedStand, this,
      std::placeholders::_1), goal_handle}.detach();
}

void UnitreeFixedStandServerNode::executeFixedStand(
  const std::shared_ptr<FixedStandGoal> goal_handle)
{
  rclcpp::Rate loop_rate(request_fixed_stand_rate_);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<FixedStand::Feedback>();
  auto & partial_step = feedback->percent;
  auto result = std::make_shared<FixedStand::Result>();
  while (rclcpp::ok()) {
    if (goal_handle->is_canceling() ) {
      goal_handle->canceled(result);
      return;
    }
    unitree_a1_fixed_stand_server_->fixedStand(state_);
    partial_step = unitree_a1_fixed_stand_server_->getPercent() * 100.0;
    goal_handle->publish_feedback(feedback);
    auto cmd = LowCmd();
    cmd.common.mode = 0x0A;
    cmd.header.stamp = this->now();
    cmd.motor_cmd = unitree_a1_fixed_stand_server_->getCmd();
    pub_cmd_->publish(cmd);
    // if partial_step == 100 then goal succeeded
    if (partial_step == 100.0) {
      break;
    }
    loop_rate.sleep();
  }
  result->success = true;
  goal_handle->succeed(result);
}

void UnitreeFixedStandServerNode::stateCallback(const LowState::SharedPtr msg)
{
  state_ = msg;
}

}  // namespace unitree_a1_fixed_stand_server

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(unitree_a1_fixed_stand_server::UnitreeFixedStandServerNode)
