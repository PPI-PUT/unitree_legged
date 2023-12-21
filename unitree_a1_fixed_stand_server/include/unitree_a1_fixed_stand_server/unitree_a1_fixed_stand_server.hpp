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

#ifndef UNITREE_A1_FIXED_STAND_SERVER__UNITREE_A1_FIXED_STAND_SERVER_HPP_
#define UNITREE_A1_FIXED_STAND_SERVER__UNITREE_A1_FIXED_STAND_SERVER_HPP_

#include <cstdint>

#include "unitree_a1_fixed_stand_server/visibility_control.hpp"
#include <unitree_a1_legged_msgs/msg/low_state.hpp>
#include <unitree_a1_legged_msgs/msg/low_cmd.hpp>

namespace unitree_a1_fixed_stand_server
{
using LowState = unitree_a1_legged_msgs::msg::LowState;
using LowCmd = unitree_a1_legged_msgs::msg::LowCmd;
using QuadrupedCmd = unitree_a1_legged_msgs::msg::QuadrupedCmd;

class UNITREE_A1_FIXED_STAND_SERVER_PUBLIC UnitreeFixedStandServer
{
public:
  UnitreeFixedStandServer();
  void fixedStand(LowState::SharedPtr state);
  float getPercent() const;
  QuadrupedCmd getCmd() const;

private:
  float steps_;
  float percent_;
  int motiontime_;
  bool init_;
  float hip_kp_;
  float hip_kd_;
  float thigh_kp_;
  float thigh_kd_;
  float calf_kp_;
  float calf_kd_;
  QuadrupedCmd cmd_;
  std::array<float, 12> lastPos_;
  std::array<float, 12> standPos_;
  void reset();
  void initParams();
  void targetToCmd(const std::vector<float> & targetPos);
  void initPose(LowState::SharedPtr state);
  double jointLinearInterpolation(
    const double initPos, const double targetPos,
    const double percent) const;
};

}  // namespace unitree_a1_fixed_stand_server

#endif  // UNITREE_A1_FIXED_STAND_SERVER__UNITREE_A1_FIXED_STAND_SERVER_HPP_
