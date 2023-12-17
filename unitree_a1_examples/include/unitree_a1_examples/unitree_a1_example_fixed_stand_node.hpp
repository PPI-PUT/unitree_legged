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

#ifndef UNITREE_A1_EXAMPLES__UNITREE_A1_EXAMPLES_NODE_HPP_
#define UNITREE_A1_EXAMPLES__UNITREE_A1_EXAMPLES_NODE_HPP_

#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "unitree_a1_examples/unitree_a1_examples.hpp"

namespace unitree_a1_examples
{
using UnitreeA1ExamplesPtr = std::unique_ptr<unitree_a1_examples::UnitreeA1Examples>;

class UNITREE_A1_EXAMPLES_PUBLIC UnitreeFixedStandNode : public rclcpp::Node
{
public:
  explicit UnitreeFixedStandNode(const rclcpp::NodeOptions & options);

private:
  UnitreeA1ExamplesPtr unitree_a1_examples_{nullptr};
  rclcpp::Publisher<unitree_a1_legged_msgs::msg::LowCmd>::SharedPtr cmd_;
  rclcpp::Subscription<unitree_a1_legged_msgs::msg::LowState>::SharedPtr state_;
  unitree_a1_legged_msgs::msg::LowCmd cmd_msg_;
  void stateCallback(unitree_a1_legged_msgs::msg::LowState::SharedPtr msg);
  void initParam(unitree_a1_legged_msgs::msg::LowCmd & cmd_msg);
  std::vector<float> targetPos_ = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
    0.0, 0.67, -1.3, 0.0, 0.67, -1.3};
  std::vector<float> standPos_ = {0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
    0.0, 0.67, -1.3, 0.0, 0.67, -1.3};
  std::vector<float> lastPos_ = std::vector<float>(targetPos_.size(), 0.0);
  float steps_ = 2000.0;
  float percent_ = 0.0;
  int motiontime_ = 1;
  bool init_ = true;
  float hip_kp_;
  float hip_kd_;
  float thigh_kp_;
  float thigh_kd_;
  float calf_kp_;
  float calf_kd_;
};
}  // namespace unitree_a1_examples

#endif  // UNITREE_A1_EXAMPLES__UNITREE_A1_EXAMPLES_NODE_HPP_
