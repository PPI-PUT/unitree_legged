// Copyright 2024 Maciej Krupka
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

#include "unitree_a1_highlevel/unitree_a1_highlevel_node.hpp"

namespace unitree_a1_highlevel
{

UnitreeA1HighlevelNode::UnitreeA1HighlevelNode(const rclcpp::NodeOptions & options)
:  Node("unitree_a1_highlevel", options)
{
  unitree_a1_highlevel_ = std::make_unique<unitree_a1_highlevel::UnitreeA1Highlevel>();
  param_name_ = this->declare_parameter("param_name", 456);
  unitree_a1_highlevel_->foo(param_name_);
}

}  // namespace unitree_a1_highlevel

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(unitree_a1_highlevel::UnitreeA1HighlevelNode)
