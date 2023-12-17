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

class UNITREE_A1_EXAMPLES_PUBLIC UnitreeExampleTorqueNode : public rclcpp::Node
{
public:
  explicit UnitreeExampleTorqueNode(const rclcpp::NodeOptions & options);

private:
  UnitreeA1ExamplesPtr unitree_a1_examples_{nullptr};
  int motiontime_ = 0;
  const double amplitude_ = .7;
  const double frequency_ = 2.0;   // Adjust the frequency as needed
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_;
  rclcpp::Subscription<unitree_a1_legged_msgs::msg::LowState>::SharedPtr state_;
  rclcpp::Publisher<unitree_a1_legged_msgs::msg::LowCmd>::SharedPtr cmd_;
  void stateCallback(unitree_a1_legged_msgs::msg::LowState::SharedPtr msg);
};
}  // namespace unitree_a1_examples

#endif  // UNITREE_A1_EXAMPLES__UNITREE_A1_EXAMPLES_NODE_HPP_
