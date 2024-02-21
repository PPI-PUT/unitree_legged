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

#ifndef UNITREE_A1_STATE_MACHINE__UNITREE_A1_STATE_MACHINE_NODE_HPP_
#define UNITREE_A1_STATE_MACHINE__UNITREE_A1_STATE_MACHINE_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include "unitree_a1_highlevel/unitree_a1_highlevel.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <std_srvs/srv/trigger.hpp>

namespace unitree_a1_highlevel
{
using UnitreeStateMachinePtr = std::unique_ptr<unitree_a1_highlevel::UnitreeStateMachine>;
using FixedStand = unitree_a1_legged_msgs::action::FixedStand;
using FixedStandGoal = rclcpp_action::ClientGoalHandle<FixedStand>;
using Gait = unitree_a1_legged_msgs::srv::Gait;
using LowCmd = unitree_a1_legged_msgs::msg::LowCmd;
using Trigger = std_srvs::srv::Trigger;
using namespace std::placeholders;

class UNITREE_A1_STATE_MACHINE_PUBLIC UnitreeStateMachineNode : public rclcpp::Node
{
public:
  explicit UnitreeStateMachineNode(const rclcpp::NodeOptions & options);

private:
  std::mutex publisherMutex;
  UnitreeStateMachinePtr unitree_a1_state_machine_{nullptr};
  rclcpp::Service<Gait>::SharedPtr server_gait_;
  rclcpp_action::Client<FixedStand>::SharedPtr fixed_stand_client_;
  rclcpp::Subscription<LowCmd>::SharedPtr stand_cmd_;
  LowCmd::SharedPtr stand_cmd_msg_;
  void standCallback(LowCmd::UniquePtr msg);
  rclcpp::Subscription<LowCmd>::SharedPtr walk_cmd_;
  LowCmd::SharedPtr walk_cmd_msg_;
  void walkCallback(LowCmd::UniquePtr msg);
  rclcpp::Publisher<LowCmd>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
  void controlLoop();
  void resultCallback(const FixedStandGoal::WrappedResult & result);
  void handleGait(
    const std::shared_ptr<Gait::Request> request,
    std::shared_ptr<Gait::Response> response);
  rclcpp::Client<Trigger>::SharedPtr client_reset_controller_;
};
}  // namespace unitree_a1_highlevel

#endif  // UNITREE_A1_STATE_MACHINE__UNITREE_A1_STATE_MACHINE_NODE_HPP_