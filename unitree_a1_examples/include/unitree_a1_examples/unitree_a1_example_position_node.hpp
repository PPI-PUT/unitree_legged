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

class UNITREE_A1_EXAMPLES_PUBLIC UnitreeExamplePositionNode : public rclcpp::Node
{
public:
  explicit UnitreeExamplePositionNode(const rclcpp::NodeOptions & options);

private:
  UnitreeA1ExamplesPtr unitree_a1_examples_{nullptr};
  void stateCallback(unitree_a1_legged_msgs::msg::LowState::SharedPtr msg);
  rclcpp::Subscription<unitree_a1_legged_msgs::msg::LowState>::SharedPtr state_;
  rclcpp::Publisher<unitree_a1_legged_msgs::msg::LowCmd>::SharedPtr cmd_;
  int motiontime = 0;
  float qInit[3] = {0};
  float qDes[3] = {0};
  float sin_mid_q[3] = {0.0, 1.2, -2.0};
  float Kp[3] = {0};
  float Kd[3] = {0};
  double time_consume = 0;
  int rate_count = 0;
  int sin_count = 0;
};
}  // namespace unitree_a1_examples

#endif  // UNITREE_A1_EXAMPLES__UNITREE_A1_EXAMPLES_NODE_HPP_
