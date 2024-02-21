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

#ifndef UNITREE_A1_HIGHLEVEL__UNITREE_A1_HIGHLEVEL_HPP_
#define UNITREE_A1_HIGHLEVEL__UNITREE_A1_HIGHLEVEL_HPP_

#include <cstdint>

#include "unitree_a1_highlevel/visibility_control.hpp"


namespace unitree_a1_highlevel
{

class UNITREE_A1_HIGHLEVEL_PUBLIC UnitreeA1Highlevel
{
public:
  UnitreeA1Highlevel();
  int64_t foo(int64_t bar) const;
};

}  // namespace unitree_a1_highlevel

#endif  // UNITREE_A1_HIGHLEVEL__UNITREE_A1_HIGHLEVEL_HPP_
