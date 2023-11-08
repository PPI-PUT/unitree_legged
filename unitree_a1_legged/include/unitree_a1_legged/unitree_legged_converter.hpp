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

#include <unordered_map>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "unitree_a1_legged/unitree_legged.hpp"
#include "unitree_a1_legged_msgs/msg/low_cmd.hpp"
#include "unitree_a1_legged_msgs/msg/low_state.hpp"
#include "unitree_a1_legged_msgs/msg/motor_cmd.hpp"
#include "unitree_a1_legged_msgs/msg/motor_state.hpp"
#include "unitree_a1_legged_msgs/msg/quadruped_state.hpp"
#include "unitree_a1_legged_msgs/msg/quadruped_cmd.hpp"
#include "unitree_a1_legged_msgs/msg/foot_force_state.hpp"

using namespace UNITREE_LEGGED_SDK;

namespace unitree_a1_legged
{
    /**
     * @brief Class for converting between UNITREE_LEGGED_SDK and ROS2 messages.
     */
    class Converter
    {
    public:
        /**
         * @brief Converts the UNITREE_LEGGED_SDK::IMU to a ROS2 message.
         */
        static sensor_msgs::msg::Imu stateToMsg(const IMU &state);
        /**
         * @brief Converts the UNITREE_LEGGED_SDK::LowState to a ROS2 message.
         */
        static unitree_a1_legged_msgs::msg::LowState stateToMsg(const LowState &state);
        /**
         * @brief Converts the UNITREE_LEGGED_SDK::MotorState to a QuadrupedState ROS2 message.
         */
        static unitree_a1_legged_msgs::msg::QuadrupedState stateToMsg(const MotorState (&state)[20]);
        /**
         * @brief Converts the UNITREE_LEGGED_SDK::MotorState to a ROS2 message.
         */
        static unitree_a1_legged_msgs::msg::MotorState stateToMsg(const MotorState &state);
        /**
         * @brief Converts the UNITREE_LEGGED_SDK::FootForceState to a ROS2 message.
         */
        static unitree_a1_legged_msgs::msg::FootForceState stateToMsg(const int16_t state[4]);
        /**
         * @brief Converts from LowCmd ROS2 message to UNITREE_LEGGED_SDK::LowCmd.
         */
        static void msgToCmd(const unitree_a1_legged_msgs::msg::LowCmd::SharedPtr msg, LowCmd &cmd);
        /**
         * @brief Converts from QuadrupedCmd ROS2 message to UNITREE_LEGGED_SDK::LowCmd.
         */
        static void msgToCmd(const unitree_a1_legged_msgs::msg::QuadrupedCmd msg, LowCmd &cmd);
        /**
         * @brief Converts from MotorCmd ROS2 message to UNITREE_LEGGED_SDK::MotorCmd.
         */
        static MotorCmd msgToCmd(const unitree_a1_legged_msgs::msg::MotorCmd &msg);
        /**
         * @brief Map for joint names.
         */
        static const std::unordered_map<std::string, int> jointIndexMap;
        /**
         * @brief Returns a vector of joint names.
         */
        static std::vector<std::string> getJointNames();
        /**
         * @brief Returns the number of joints.
         */
        static size_t getJointCount();
        /**
         * @brief Converts the UNITREE_LEGGED_SDK::LowState to a ROS2 message.
         */
        static sensor_msgs::msg::JointState getJointStateMsg(const LowState &state);
    };

} // namespace unitree_a1_legged