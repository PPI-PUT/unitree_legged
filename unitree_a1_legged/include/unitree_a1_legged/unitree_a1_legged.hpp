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

#ifndef UNITREE_A1_LEGGED__UNITREE_A1_LEGGED_HPP_
#define UNITREE_A1_LEGGED__UNITREE_A1_LEGGED_HPP_

#include <cstdint>

#include "unitree_a1_legged/visibility_control.hpp"
#include "unitree_a1_legged_msgs/msg/motor_cmd.hpp"
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "unitree_legged_sdk/unitree_joystick.h"
#include "unitree_legged_sdk/a1_const.h"
using namespace UNITREE_LEGGED_SDK;

namespace unitree_a1_legged
{
/**
     * @brief The motor mode for joint control.
     */
constexpr uint8_t PMSM_SERVO_MODE = 0x0A;
/**
     * @brief Class for communication with the robot.
     */
class UNITREE_A1_LEGGED_PUBLIC UnitreeLegged
{
public:
  UnitreeLegged();
  /**
       * @brief Returns the UDP object used for communication with the robot.
       */
  UDP getLowUdp();
  /**
       * @brief Returns the LowCmd object used for sending commands to the robot.
       */
  LowCmd getLowCmd();
  /**
       * @brief Returns the LowState object used for receiving state from the robot.
       */
  LowState getLowState();
  /**
       * @brief Sets the motor mode for all motors.
       * @param mode The motor mode to set.
       */
  void setMotorMode(uint8_t mode);
  /**
       * @brief Sends the LowCmd object to the robot.
       * @param cmd The LowCmd object to send. It should be getLowCmd()
       */
  void sendLowCmd(LowCmd & cmd);
  /**
       * @brief Receives the LowState object from the robot.
       */
  void sendProtectLowCmd(LowCmd & cmd, const int input_factor);
  void recvLowState();

private:
  /**
       * @brief The UDP object used for communication with the robot.
       */
  UDP low_udp_;
  /**
       * @brief The LowCmd object used for sending commands to the robot. Ignore: missing-field-initializers
       */
  LowCmd low_cmd_ = {0};
  /**
       * @brief The LowState object used for receiving state from the robot. Ignore: missing-field-initializers
       */
  LowState low_state_ = {0};
  Safety safe_;
};

}  // namespace unitree_a1_legged

#endif  // UNITREE_A1_LEGGED__UNITREE_A1_LEGGED_HPP_
