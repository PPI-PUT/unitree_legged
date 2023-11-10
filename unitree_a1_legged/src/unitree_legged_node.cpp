// Copyright 2023 Mobile Robots Laboratory at Poznan University of Technology
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

#include "unitree_a1_legged/unitree_legged_node.hpp"

namespace unitree_a1_legged

{

    UnitreeLeggedNode::UnitreeLeggedNode(const rclcpp::NodeOptions &options) : Node("unitree_lowlevel", options)
    {
        bool hot_start = this->declare_parameter("hot_start", false);
        hot_start = this->get_parameter("hot_start").as_bool();
        if (!hot_start)
        {
            RCLCPP_INFO(this->get_logger(), "Starting Unitree Legged Node");
            RCLCPP_INFO(this->get_logger(), "Make sure the robot is lied on the ground and the motors are turned on.");
            RCLCPP_INFO(this->get_logger(), "Press L1 and hold A to sit down the robot.");
            RCLCPP_INFO(this->get_logger(), "Press L1+L2 and A to turn on joint control mode.");
            RCLCPP_INFO(this->get_logger(), "Press Enter to continue...");
            std::cin.get();
        }
        // Init motor Mode
        unitree_.setMotorMode(PMSM_SERVO_MODE);
        // state_thread_ = std::thread(std::bind(&UnitreeLeggedNode::updateLoop, this));
        state_publisher_ = this->create_publisher<unitree_a1_legged_msgs::msg::LowState>("~/state", 1);
        joint_state_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 1);
        joint_state_subscriber_ = this->create_subscription<unitree_a1_legged_msgs::msg::JointCommand>("~/joint_command", 1, std::bind(&UnitreeLeggedNode::receiveJointCommandCallback, this, std::placeholders::_1));
        low_command_subscriber_ = this->create_subscription<unitree_a1_legged_msgs::msg::LowCmd>("~/command", 1, std::bind(&UnitreeLeggedNode::receiveCommandCallback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(2ms, std::bind(&UnitreeLeggedNode::updateStateCallback, this));
    }
    UnitreeLeggedNode::~UnitreeLeggedNode()
    {
        if (state_thread_.joinable())
            state_thread_.join();
    }

    void UnitreeLeggedNode::updateLoop()
    {
        while (rclcpp::ok())
        {
            unitree_a1_legged_msgs::msg::LowState low_state_ros;
            unitree_.recvLowState();
            low_state_ros = Converter::stateToMsg(unitree_.getLowState());
            state_publisher_->publish(low_state_ros);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }
    void UnitreeLeggedNode::receiveCommandCallback(const unitree_a1_legged_msgs::msg::LowCmd::SharedPtr msg)
    {
        // Convert to lowlevel cmd
        auto low_cmd_once = unitree_.getLowCmd();
        Converter::msgToCmd(msg, low_cmd_once);
        unitree_.sendLowCmd(low_cmd_once);
    }
    void UnitreeLeggedNode::receiveJointCommandCallback(const unitree_a1_legged_msgs::msg::JointCommand::SharedPtr msg)
    {
        // Convert to lowlevel cmd
        auto low_cmd_once = unitree_.getLowCmd();
        Converter::msgToCmd(msg, low_cmd_once);
        unitree_.sendLowCmd(low_cmd_once);
    }
    void UnitreeLeggedNode::updateStateCallback()
    {
        // Get state
        unitree_.recvLowState();
        // Convert to lowlevel state msgs
        auto low_state_ros = Converter::stateToMsg(unitree_.getLowState());
        low_state_ros.header.stamp = this->now();
        state_publisher_->publish(low_state_ros);
        // Convert to joint state msgs
        auto joint_state_msg = Converter::getJointStateMsg(unitree_.getLowState());
        joint_state_msg.header.stamp = this->now();
        joint_state_publisher_->publish(joint_state_msg);
    }

} // namespace unitree_a1_legged

// #include "rclcpp_components/register_node_macro.hpp"

// RCLCPP_COMPONENTS_REGISTER_NODE(unitree_a1_legged::UnitreeLeggedNode)
