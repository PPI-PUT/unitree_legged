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

#include "unitree_a1_fixed_stand_server/unitree_a1_fixed_stand_server.hpp"

#include <iostream>

namespace unitree_a1_fixed_stand_server
{

UnitreeFixedStandServer::UnitreeFixedStandServer()
{
  this->initParams();
}

LowCmd UnitreeFixedStandServer::fixedStand(
  const QuadrupedState & motor_state,
  const builtin_interfaces::msg::Time & stamp,
  float & percent)
{
  auto lowCmd = LowCmd();
  lowCmd.header.stamp = stamp;
  this->initPose(motor_state);
  auto targetPos = std::vector<float>(lastPos_.size(), 0.0);
  if (motiontime_ < steps_) {
    motiontime_++;
    percent = static_cast<float>(motiontime_) / steps_;
    for (size_t i = 0; i < targetPos.size(); i++) {
      targetPos[i] = this->jointLinearInterpolation(
        lastPos_[i], standPos_[i],
        percent);
    }
    percent = percent * 100;
    auto setpoint = this->initStandGains();
    this->targetToCmd(setpoint, targetPos);
    lowCmd.common.mode = 0x0A;
    lowCmd.motor_cmd = setpoint;
  }
  return lowCmd;
}

LowCmd UnitreeFixedStandServer::holdPosition(
  const QuadrupedState & state, const builtin_interfaces::msg::Time & stamp)
{
  LowCmd lowCmd;
  lowCmd.common.mode = 0x0A;
  lowCmd.header.stamp = stamp;
  auto hold_position = this->quadrupedStatetoVector(state);
  auto setpoint = this->initHoldGains();
  this->targetToCmd(setpoint, hold_position);
  lowCmd.motor_cmd = setpoint;
  return lowCmd;
}

void UnitreeFixedStandServer::reset()
{
  this->initParams();
}

void UnitreeFixedStandServer::targetToCmd(QuadrupedCmd & cmd, const std::vector<float> & targetPos)
{
  cmd.front_right.hip.q = targetPos[0];
  cmd.front_right.thigh.q = targetPos[1];
  cmd.front_right.calf.q = targetPos[2];
  cmd.front_left.hip.q = targetPos[3];
  cmd.front_left.thigh.q = targetPos[4];
  cmd.front_left.calf.q = targetPos[5];
  cmd.rear_right.hip.q = targetPos[6];
  cmd.rear_right.thigh.q = targetPos[7];
  cmd.rear_right.calf.q = targetPos[8];
  cmd.rear_left.hip.q = targetPos[9];
  cmd.rear_left.thigh.q = targetPos[10];
  cmd.rear_left.calf.q = targetPos[11];
}

double UnitreeFixedStandServer::jointLinearInterpolation(
  const double initPos, const double targetPos,
  const double percent) const
{
  double p;
  auto rate = std::min(std::max(percent, 0.0), 1.0);
  p = initPos * (1 - rate) + targetPos * rate;
  return p;
}

void UnitreeFixedStandServer::initPose(const QuadrupedState & state)
{
  if (init_) {
    lastPos_[0] = state.front_right.hip.q;
    lastPos_[1] = state.front_right.thigh.q;
    lastPos_[2] = state.front_right.calf.q;
    lastPos_[3] = state.front_left.hip.q;
    lastPos_[4] = state.front_left.thigh.q;
    lastPos_[5] = state.front_left.calf.q;
    lastPos_[6] = state.rear_right.hip.q;
    lastPos_[7] = state.rear_right.thigh.q;
    lastPos_[8] = state.rear_right.calf.q;
    lastPos_[9] = state.rear_left.hip.q;
    lastPos_[10] = state.rear_left.thigh.q;
    lastPos_[11] = state.rear_left.calf.q;
    init_ = false;
  }
}

std::vector<float> UnitreeFixedStandServer::quadrupedStatetoVector(const QuadrupedState & state)
{
  std::vector<float> vec;
  vec.push_back(state.front_right.hip.q);
  vec.push_back(state.front_right.thigh.q);
  vec.push_back(state.front_right.calf.q);
  vec.push_back(state.front_left.hip.q);
  vec.push_back(state.front_left.thigh.q);
  vec.push_back(state.front_left.calf.q);
  vec.push_back(state.rear_right.hip.q);
  vec.push_back(state.rear_right.thigh.q);
  vec.push_back(state.rear_right.calf.q);
  vec.push_back(state.rear_left.hip.q);
  vec.push_back(state.rear_left.thigh.q);
  vec.push_back(state.rear_left.calf.q);
  return vec;

}

void UnitreeFixedStandServer::initParams()
{
  steps_ = 500.0;
  motiontime_ = 1;
  standPos_ = {-0.1, 0.8, -1.5, 0.1, 0.8, -1.5,
    -0.1, 1.0, -1.5, 0.1, 1.0, -1.5};
  std::fill(lastPos_.begin(), lastPos_.end(), 0.0);
  init_ = true;
}

QuadrupedCmd UnitreeFixedStandServer::initStandGains()
{
  float hip_kp = 70.0;
  float hip_kd = 3.0;
  float thigh_kp = 180.0;
  float thigh_kd = 8.0;
  float calf_kp = 300.0;
  float calf_kd = 15.0;
  return this->initGains(hip_kp, hip_kd, thigh_kp, thigh_kd, calf_kp, calf_kd);
}
QuadrupedCmd UnitreeFixedStandServer::initHoldGains()
{
  float kp = 20.0;
  float kd = 0.5;
  return this->initGains(kp, kd, kp, kd, kp, kd);
}

QuadrupedCmd UnitreeFixedStandServer::initGains(
  float hip_kp, float hip_kd, float thigh_kp,
  float thigh_kd, float calf_kp, float calf_kd)
{
  QuadrupedCmd cmd;
  cmd.front_right.hip.kp = hip_kp;
  cmd.front_right.hip.kd = hip_kd;
  cmd.front_left.hip.kp = hip_kp;
  cmd.front_left.hip.kd = hip_kd;
  cmd.rear_right.hip.kp = hip_kp;
  cmd.rear_right.hip.kd = hip_kd;
  cmd.rear_left.hip.kp = hip_kp;
  cmd.rear_left.hip.kd = hip_kd;

  cmd.front_right.thigh.kp = thigh_kp;
  cmd.front_right.thigh.kd = thigh_kd;
  cmd.front_left.thigh.kp = thigh_kp;
  cmd.front_left.thigh.kd = thigh_kd;
  cmd.rear_right.thigh.kp = thigh_kp;
  cmd.rear_right.thigh.kd = thigh_kd;
  cmd.rear_left.thigh.kp = thigh_kp;
  cmd.rear_left.thigh.kd = thigh_kd;

  cmd.front_right.calf.kp = calf_kp;
  cmd.front_right.calf.kd = calf_kd;
  cmd.front_left.calf.kp = calf_kp;
  cmd.front_left.calf.kd = calf_kd;
  cmd.rear_right.calf.kp = calf_kp;
  cmd.rear_right.calf.kd = calf_kd;
  cmd.rear_left.calf.kp = calf_kp;
  cmd.rear_left.calf.kd = calf_kd;
  return cmd;
}

}  // namespace unitree_a1_fixed_stand_server
