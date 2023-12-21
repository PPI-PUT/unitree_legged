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
  this->reset();
  this->initParams();
}

void UnitreeFixedStandServer::fixedStand(LowState::SharedPtr state)
{
  this->initPose(state);
  auto targetPos = std::vector<float>(lastPos_.size(), 0.0);
  if (motiontime_ < steps_) {
    motiontime_++;
    percent_ = static_cast<float>(motiontime_) / steps_;
    for (size_t i = 0; i < targetPos.size(); i++) {
      targetPos[i] = this->jointLinearInterpolation(
        lastPos_[i], standPos_[i],
        percent_);
    }
    this->targetToCmd(targetPos);
  }
}

float UnitreeFixedStandServer::getPercent() const
{
  return percent_;
}

QuadrupedCmd UnitreeFixedStandServer::getCmd() const
{
  return cmd_;
}

void UnitreeFixedStandServer::reset()
{
  init_ = true;
  std::fill(lastPos_.begin(), lastPos_.end(), 0.0);
}

void UnitreeFixedStandServer::targetToCmd(const std::vector<float> & targetPos)
{
  cmd_.front_right.hip.q = targetPos[0];
  cmd_.front_right.thigh.q = targetPos[1];
  cmd_.front_right.calf.q = targetPos[2];
  cmd_.front_left.hip.q = targetPos[3];
  cmd_.front_left.thigh.q = targetPos[4];
  cmd_.front_left.calf.q = targetPos[5];
  cmd_.rear_right.hip.q = targetPos[6];
  cmd_.rear_right.thigh.q = targetPos[7];
  cmd_.rear_right.calf.q = targetPos[8];
  cmd_.rear_left.hip.q = targetPos[9];
  cmd_.rear_left.thigh.q = targetPos[10];
  cmd_.rear_left.calf.q = targetPos[11];
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

void UnitreeFixedStandServer::initPose(LowState::SharedPtr state)
{
  if (init_) {
    lastPos_[0] = state->motor_state.front_right.hip.q;
    lastPos_[1] = state->motor_state.front_right.thigh.q;
    lastPos_[2] = state->motor_state.front_right.calf.q;
    lastPos_[3] = state->motor_state.front_left.hip.q;
    lastPos_[4] = state->motor_state.front_left.thigh.q;
    lastPos_[5] = state->motor_state.front_left.calf.q;
    lastPos_[6] = state->motor_state.rear_right.hip.q;
    lastPos_[7] = state->motor_state.rear_right.thigh.q;
    lastPos_[8] = state->motor_state.rear_right.calf.q;
    lastPos_[9] = state->motor_state.rear_left.hip.q;
    lastPos_[10] = state->motor_state.rear_left.thigh.q;
    lastPos_[11] = state->motor_state.rear_left.calf.q;
    init_ = false;
  }
}

void UnitreeFixedStandServer::initParams()
{
  steps_ = 500.0;
  percent_ = 0.0;
  motiontime_ = 1;
  hip_kp_ = 70.0;
  hip_kd_ = 3.0;
  thigh_kp_ = 180.0;
  thigh_kd_ = 8.0;
  calf_kp_ = 300.0;
  calf_kd_ = 15.0;
  percent_ = 0.0;
  motiontime_ = 1;
  standPos_ = {-0.1, 0.8, -1.5, 0.1, 0.8, -1.5,
    -0.1, 1.0, -1.5, 0.1, 1.0, -1.5};

  cmd_.front_right.hip.kp = hip_kp_;
  cmd_.front_right.hip.kd = hip_kd_;
  cmd_.front_left.hip.kp = hip_kp_;
  cmd_.front_left.hip.kd = hip_kd_;
  cmd_.rear_right.hip.kp = hip_kp_;
  cmd_.rear_right.hip.kd = hip_kd_;
  cmd_.rear_left.hip.kp = hip_kp_;
  cmd_.rear_left.hip.kd = hip_kd_;

  cmd_.front_right.thigh.kp = thigh_kp_;
  cmd_.front_right.thigh.kd = thigh_kd_;
  cmd_.front_left.thigh.kp = thigh_kp_;
  cmd_.front_left.thigh.kd = thigh_kd_;
  cmd_.rear_right.thigh.kp = thigh_kp_;
  cmd_.rear_right.thigh.kd = thigh_kd_;
  cmd_.rear_left.thigh.kp = thigh_kp_;
  cmd_.rear_left.thigh.kd = thigh_kd_;

  cmd_.front_right.calf.kp = calf_kp_;
  cmd_.front_right.calf.kd = calf_kd_;
  cmd_.front_left.calf.kp = calf_kp_;
  cmd_.front_left.calf.kd = calf_kd_;
  cmd_.rear_right.calf.kp = calf_kp_;
  cmd_.rear_right.calf.kd = calf_kd_;
  cmd_.rear_left.calf.kp = calf_kp_;
  cmd_.rear_left.calf.kd = calf_kd_;
}

}  // namespace unitree_a1_fixed_stand_server
