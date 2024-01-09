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

#include "unitree_a1_state_machine/unitree_a1_state_machine_node.hpp"

constexpr double a1_Hip_max = 0.802;          // unit:radian ( = 46   degree)
constexpr double a1_Hip_min = -0.802;         // unit:radian ( = -46  degree)
constexpr double a1_Thigh_max = 4.19;         // unit:radian ( = 240  degree)
constexpr double a1_Thigh_min = -1.05;        // unit:radian ( = -60  degree)
constexpr double a1_Calf_max = -0.916;        // unit:radian ( = -52.5  degree)
constexpr double a1_Calf_min = -2.7;          // unit:radian ( = -154.5 degree)


namespace unitree_a1_state_machine
{
UnitreeStateMachineNode::UnitreeStateMachineNode(const rclcpp::NodeOptions & options)
:  Node("unitree_a1_state_machine", options)
{
  joint_state_publisher_ =
    this->create_publisher<sensor_msgs::msg::JointState>("~/output/nn/joint_states", 1);
  stand_cmd_msg_ = std::make_shared<LowCmd>();
  walk_cmd_msg_ = std::make_shared<LowCmd>();
  unitree_a1_state_machine_ = std::make_unique<unitree_a1_state_machine::UnitreeStateMachine>();
  stand_cmd_ = this->create_subscription<LowCmd>(
    "~/input/stand", 1,
    std::bind(&UnitreeStateMachineNode::standCallback, this, _1));
  walk_cmd_ = this->create_subscription<LowCmd>(
    "~/input/walk", 1,
    std::bind(&UnitreeStateMachineNode::walkCallback, this, _1));
  pub_cmd_ = this->create_publisher<LowCmd>("~/output/cmd", 1);
  timer_ =
    this->create_wall_timer(
    std::chrono::milliseconds(20),
    std::bind(&UnitreeStateMachineNode::controlLoop, this));
  server_gait_ = this->create_service<Gait>(
    "~/service/gait",
    std::bind(
      &UnitreeStateMachineNode::handleGait, this, _1, _2));
  fixed_stand_client_ = rclcpp_action::create_client<FixedStand>(
    this,
    "~/action/fixed_stand");
  client_reset_controller_ = this->create_client<Trigger>("~/service/reset_controller");
}
void UnitreeStateMachineNode::controlLoop()
{
  auto state = unitree_a1_state_machine_->getState();
  if (state == unitree_a1_state_machine::State::STAND) {
    pub_cmd_->publish(*stand_cmd_msg_);
  } else if (state == unitree_a1_state_machine::State::WALK) {
    pub_cmd_->publish(*walk_cmd_msg_);
  } else if (state == unitree_a1_state_machine::State::HOLD) {
    pub_cmd_->publish(*stand_cmd_msg_);
  }

  auto joints = sensor_msgs::msg::JointState();
  joints.header.stamp = this->now();
  joints.header.frame_id = "trunk";
  joints.name = {"FR_hip_joint", "FR_thigh_joint", "FR_calf_joint", "FL_hip_joint",
    "FL_thigh_joint", "FL_calf_joint", "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint"};
  joints.position = {
    walk_cmd_msg_->motor_cmd.front_right.hip.q,
    walk_cmd_msg_->motor_cmd.front_right.thigh.q,
    walk_cmd_msg_->motor_cmd.front_right.calf.q,
    walk_cmd_msg_->motor_cmd.front_left.hip.q,
    walk_cmd_msg_->motor_cmd.front_left.thigh.q,
    walk_cmd_msg_->motor_cmd.front_left.calf.q,
    walk_cmd_msg_->motor_cmd.rear_right.hip.q,
    walk_cmd_msg_->motor_cmd.rear_right.thigh.q,
    walk_cmd_msg_->motor_cmd.rear_right.calf.q,
    walk_cmd_msg_->motor_cmd.rear_left.hip.q,
    walk_cmd_msg_->motor_cmd.rear_left.thigh.q,
    walk_cmd_msg_->motor_cmd.rear_left.calf.q};
  joint_state_publisher_->publish(joints);
}
void UnitreeStateMachineNode::standCallback(const LowCmd::SharedPtr msg)
{
  stand_cmd_msg_ = msg;
}
void UnitreeStateMachineNode::walkCallback(const LowCmd::SharedPtr msg)
{
  walk_cmd_msg_ = msg;
}
void UnitreeStateMachineNode::resultCallback(const FixedStandGoal::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
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

void UnitreeStateMachineNode::handleGait(
  const std::shared_ptr<Gait::Request> request,
  std::shared_ptr<Gait::Response> response)
{
  switch (request->method.type) {
    case 1:
      {
        RCLCPP_INFO(this->get_logger(), "STOP");
      } break;
    case 2:
      {
        RCLCPP_INFO(this->get_logger(), "API");
      } break;
    case 3:
      {
        if (unitree_a1_state_machine_->getState() == unitree_a1_state_machine::State::UNKNOWN) {
          unitree_a1_state_machine_->nextState(); // skip UNKNOWN state
          unitree_a1_state_machine_->nextState(); // skip STOP state
          RCLCPP_INFO(this->get_logger(), "SKIP UNKNOWN AND STOP STATE");
        } else {return;}
        unitree_a1_state_machine_->nextState();
        RCLCPP_INFO(this->get_logger(), "STAND");
        if (!fixed_stand_client_->wait_for_action_server(std::chrono::milliseconds(500))) {
          RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
          response->success = false;
          return;
        }
        auto goal = FixedStand::Goal();
        goal.start = true;
        auto send_goal_options = rclcpp_action::Client<FixedStand>::SendGoalOptions();
        send_goal_options.result_callback =
          std::bind(&UnitreeStateMachineNode::resultCallback, this, _1);

        auto goal_handle_future = fixed_stand_client_->async_send_goal(goal, send_goal_options);
      } break;
    case 4:
      {
        client_reset_controller_->async_send_request(
          std::make_shared<Trigger::Request>());
        unitree_a1_state_machine_->nextState();
        RCLCPP_INFO(this->get_logger(), "Walk");
      } break;
    default:
      {
        RCLCPP_INFO(this->get_logger(), "Unknown");
        response->success = true;
        return;
      } break;
  }
  response->success = true;
}

}  // namespace unitree_a1_state_machine

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(unitree_a1_state_machine::UnitreeStateMachineNode)
