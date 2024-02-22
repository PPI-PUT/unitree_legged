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

#ifndef UNITREE_A1_FIXED_STAND_SERVER__UNITREE_A1_FIXED_STAND_SERVER_NODE_HPP_
#define UNITREE_A1_FIXED_STAND_SERVER__UNITREE_A1_FIXED_STAND_SERVER_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <unitree_a1_legged_msgs/action/fixed_stand.hpp>

#include "unitree_a1_fixed_stand_server/unitree_a1_fixed_stand_server.hpp"

namespace unitree_a1_fixed_stand_server
{
using UnitreeA1FixedStandServerPtr =
  std::unique_ptr<unitree_a1_fixed_stand_server::UnitreeFixedStandServer>;
using FixedStand = unitree_a1_legged_msgs::action::FixedStand;
using FixedStandGoal = rclcpp_action::ServerGoalHandle<FixedStand>;

class UNITREE_A1_FIXED_STAND_SERVER_PUBLIC UnitreeFixedStandServerNode : public rclcpp::Node
{
public:
  explicit UnitreeFixedStandServerNode(const rclcpp::NodeOptions & options);

private:
  int request_fixed_stand_rate_;
  UnitreeA1FixedStandServerPtr unitree_a1_fixed_stand_server_{nullptr};
  LowState::SharedPtr state_;
  rclcpp::Subscription<LowState>::SharedPtr sub_state_;
  rclcpp::Publisher<LowCmd>::SharedPtr pub_cmd_;
  rclcpp_action::Server<FixedStand>::SharedPtr fixed_stand_server_;
  rclcpp_action::GoalResponse handleFixedStandGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FixedStand::Goal> goal);
  rclcpp_action::CancelResponse handleFixedStandCancel(
    const std::shared_ptr<FixedStandGoal> goal_handle);
  void handleAcceptedFixedStand(
    const std::shared_ptr<FixedStandGoal> goal_handle);
  void executeFixedStand(
    const std::shared_ptr<FixedStandGoal> goal_handle);
  rclcpp_action::Server<FixedStand>::SharedPtr hold_position_server_;
  rclcpp_action::GoalResponse handleHoldPositionGoal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FixedStand::Goal> goal);
  rclcpp_action::CancelResponse handleHoldPositionCancel(
    const std::shared_ptr<FixedStandGoal> goal_handle);
  void handleAcceptedHoldPosition(
    const std::shared_ptr<FixedStandGoal> goal_handle);
  void executeHoldPosition(
    const std::shared_ptr<FixedStandGoal> goal_handle);
  void stateCallback(const LowState::SharedPtr msg);
};
}  // namespace unitree_a1_fixed_stand_server

#endif  // UNITREE_A1_FIXED_STAND_SERVER__UNITREE_A1_FIXED_STAND_SERVER_NODE_HPP_
