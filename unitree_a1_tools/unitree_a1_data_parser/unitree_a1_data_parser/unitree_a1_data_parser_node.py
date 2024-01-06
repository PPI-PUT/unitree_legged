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

import rclpy
import numpy as np
import pandas as pd
import math
from rclpy.node import Node
try:
    from unitree_a1_data_parser.unitree_a1_data_parser import UnitreeA1DataParser
except ImportError:
    from unitree_a1_data_parser import UnitreeA1DataParser

from unitree_a1_legged_msgs.msg import DebugDoubleArray


class UnitreeA1DataParserNode(Node):

    def __init__(self):
        super().__init__('unitree_a1_data_parser_node')
        self.sub_action = self.create_subscription(
            DebugDoubleArray,
            '~/action',
            self.action_callback,
            1)
        self.sub_state = self.create_subscription(
            DebugDoubleArray,
            '~/state',
            self.state_callback,
            1)
        self.unitree_a1_data_parser = UnitreeA1DataParser()
        self.path_name = self.declare_parameter('file_path', '').value
        self.action_list = []
        self.state_list = []
        self.time = []

    def action_callback(self, msg):
        time = self.get_time(msg.header.stamp)
        self.time.append(time)
        self.action_list.append(np.array(msg.data))

    def state_callback(self, msg):
        self.state_list.append(np.array(msg.data))

    def get_time(self, stamp: rclpy.time.Time):
        # Calculate the time
        return math.fsum([stamp.sec, stamp.nanosec * math.pow(10, -9)])

    def __del__(self):
        # On destruction, save the data to a csv file
        df = pd.DataFrame()
        df['time'] = self.time
        df['action'] = self.action_list
        df['state'] = self.state_list
        df.to_csv(self.path_name)


def main(args=None):
    rclpy.init(args=args)
    node = UnitreeA1DataParserNode()
    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
