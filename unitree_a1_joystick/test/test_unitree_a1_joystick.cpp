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

#include "gtest/gtest.h"
#include "unitree_a1_joystick/unitree_a1_joystick.hpp"

TEST(TestUnitreeA1Joystick, TestHello) {
  std::unique_ptr<unitree_a1_joystick::UnitreeA1Joystick> unitree_a1_joystick_ =
    std::make_unique<unitree_a1_joystick::UnitreeA1Joystick>();
  auto result = unitree_a1_joystick_->foo(999);
  EXPECT_EQ(result, 999);
}
