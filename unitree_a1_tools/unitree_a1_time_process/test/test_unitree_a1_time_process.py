#!/usr/bin/env python3

# Copyright 2024 Maciej Krupka
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import pytest
from unitree_a1_time_process.unitree_a1_time_process import UnitreeA1TimeProcess


@pytest.mark.parametrize('test_input, expected', [
    (999, 999)
])
def test_foo(test_input, expected):
    unitree_a1_time_process = UnitreeA1TimeProcess()
    result = unitree_a1_time_process.foo(bar=test_input)
    assert result == expected, 'Wrong value!'
