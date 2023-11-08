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

#include "unitree_a1_legged/unitree_legged_converter.hpp"

#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/joint_state.hpp>

using namespace UNITREE_LEGGED_SDK;

namespace unitree_a1_legged

{
    /**
     * @brief Node for communication with the robot.
     */
    class UnitreeLeggedNode : public rclcpp::Node
    {
    public:
        /**
         * @brief Constructor for the UnitreeLeggedNode class.
         */
        explicit UnitreeLeggedNode(const rclcpp::NodeOptions &options);
        ~UnitreeLeggedNode();

    private:
        /**
         * @brief The thread for the update loop. Not used.
         */
        std::thread state_thread_;
        /**
         * @brief The UnitreeLegged object used for communication with the robot.
         */
        UnitreeLegged unitree_;
        /**
         * @brief The LowCmd object used for sending commands to the robot.
         */
        sensor_msgs::msg::JointState joint_state_msg_;
        /**
         * @brief Timer for the update loop.
         */
        rclcpp::TimerBase::SharedPtr timer_;
        /**
         * @brief Subscriber for the LowCmd ROS2 message.
         */
        rclcpp::Subscription<unitree_a1_legged_msgs::msg::LowCmd>::SharedPtr low_command_subscriber_;
        /**
         * @brief Subscriber for the QuadrupedCmd ROS2 message.
         */
        rclcpp::Subscription<unitree_a1_legged_msgs::msg::QuadrupedState>::SharedPtr quadruped_command__subscriber_;
        /**
         * @brief Publisher for the LowState ROS2 message.
         */
        rclcpp::Publisher<unitree_a1_legged_msgs::msg::LowState>::SharedPtr state_publisher_;
        /**
         * @brief Publisher for the JointState ROS2 message.
         */
        rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_publisher_;
        void updateLoop();
        /**
         * @brief Callback for receiving the LowCmd ROS2 message.
         */
        void receiveCommandCallback(const unitree_a1_legged_msgs::msg::LowCmd::SharedPtr msg);
        /**
         * @brief Callback for receiving the QuadrupedCmd ROS2 message.
         */
        void updateStateCallback();
    };
} // namespace unitree_a1_legged