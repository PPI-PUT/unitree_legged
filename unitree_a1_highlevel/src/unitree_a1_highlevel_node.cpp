// Copyright 2023 Maciej Krupka
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

#include "unitree_a1_highlevel/unitree_a1_highlevel_node.hpp"
#include <cinttypes>
namespace unitree_a1_highlevel
{
UnitreeStateMachineNode::UnitreeStateMachineNode(const rclcpp::NodeOptions & options)
:  Node("unitree_a1_highlevel", options)
{
  unitree_a1_state_machine_ = std::make_unique<unitree_a1_highlevel::UnitreeStateMachine>();
  sub_twist_ = this->create_subscription<TwistStamped>(
    "~/input/twist", 1,
    std::bind(&UnitreeStateMachineNode::twistCallback, this, _1));
  auto qos = rclcpp::QoS(1);
  qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
  qos.durability_volatile();
  pub_twist_ = this->create_publisher<TwistStampedHighlevel>("~/output/twist", qos);
  server_gait_ = this->create_service<Gait>(
    "~/service/gait",
    std::bind(
      &UnitreeStateMachineNode::handleGait, this, _1, _2));
  fixed_stand_client_ = rclcpp_action::create_client<FixedStand>(
    this,
    "~/action/fixed_stand");
  hold_position_client_ = rclcpp_action::create_client<FixedStand>(
    this,
    "~/action/hold_position");
}
void UnitreeStateMachineNode::twistCallback(TwistStamped::UniquePtr msg)
{
  auto twist = std::make_unique<TwistStampedHighlevel>();
  twist->header.stamp = this->get_clock()->now();
  twist->twist = msg->twist;
  twist->controller_type.type = unitree_a1_state_machine_->getControllerType();
  pub_twist_->publish(std::move(twist));
}
void UnitreeStateMachineNode::resultCallback(const FixedStandGoal::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Stand succeeded");
      unitree_a1_state_machine_->setState(unitree_a1_highlevel::State::HOLD);
      this->callHoldPosition();
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_INFO(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_INFO(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_INFO(this->get_logger(), "Unknown result code");
      return;
  }
}

void UnitreeStateMachineNode::callHoldPosition()
{
  if (!hold_position_client_->wait_for_action_server(std::chrono::milliseconds(500))) {
    RCLCPP_ERROR(
      this->get_logger(),
      "Hold position action server not available after waiting");
    return;
  }
  auto goal = FixedStand::Goal();
  goal.start = true;
  auto goal_handle_future = hold_position_client_->async_send_goal(goal);
}

void UnitreeStateMachineNode::callStand()
{
  if (!fixed_stand_client_->wait_for_action_server(std::chrono::milliseconds(500))) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    return;
  }
  auto goal = FixedStand::Goal();
  goal.start = true;
  auto send_goal_options = rclcpp_action::Client<FixedStand>::SendGoalOptions();
  send_goal_options.result_callback =
    std::bind(&UnitreeStateMachineNode::resultCallback, this, _1);
  auto goal_handle_future = fixed_stand_client_->async_send_goal(goal, send_goal_options);
}

void UnitreeStateMachineNode::handleGait(
  const std::shared_ptr<Gait::Request> request,
  std::shared_ptr<Gait::Response> response)
{
  switch (request->method.type) {
    case ControllerType::STOP:
      {
        RCLCPP_INFO(this->get_logger(), "STOP");
        unitree_a1_state_machine_->setState(unitree_a1_highlevel::State::STOP);
        fixed_stand_client_->async_cancel_all_goals();
        hold_position_client_->async_cancel_all_goals();
      } break;
    case ControllerType::API:
      {
        RCLCPP_INFO(this->get_logger(), "API");
      } break;
    case ControllerType::FIXED_STAND:
      {
        if (unitree_a1_state_machine_->getState() == unitree_a1_highlevel::State::UNKNOWN ||
          unitree_a1_state_machine_->getState() == unitree_a1_highlevel::State::STOP)
        {
          unitree_a1_state_machine_->setState(unitree_a1_highlevel::State::GROUND);
          RCLCPP_INFO(
            this->get_logger(), "Robot is on the ground, setting state to STAND is allowed");
        } else if (unitree_a1_state_machine_->getState() == unitree_a1_highlevel::State::GROUND) {
          unitree_a1_state_machine_->setState(unitree_a1_highlevel::State::STAND);
          RCLCPP_INFO(this->get_logger(), "Robot is standing...");
          RCLCPP_INFO(
            this->get_logger(), "State: %d",
            static_cast<int>(unitree_a1_state_machine_->getState()));
          this->callStand();
        }
      } break;
    case ControllerType::NEURAL:
      {
        if (unitree_a1_state_machine_->getState() == unitree_a1_highlevel::State::HOLD) {
          unitree_a1_state_machine_->setState(unitree_a1_highlevel::State::WALK);
        } else {
          RCLCPP_INFO(this->get_logger(), "Robot is not standing, cannot walk");
          response->success = false;
          return;
        }
      } break;
    default:
      {
        RCLCPP_ERROR(this->get_logger(), "Unknown controller type");
        response->success = true;
        return;
      } break;
  }
  response->success = true;
}

}  // namespace unitree_a1_highlevel

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(unitree_a1_highlevel::UnitreeStateMachineNode)
