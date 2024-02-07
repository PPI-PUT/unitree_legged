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
from datetime import datetime
from pathlib import Path
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
        self.deadzone = self.declare_parameter('deadzone', 10.0).value
        rosbag_name = self.declare_parameter('rosbag_name', '').value
        rosbag_name = Path(rosbag_name)
        self.path_name = Path(rosbag_name.expanduser())
        self.path_name = self.path_name / \
            f'{self.path_name.name.replace("rosbag2", "log")}.csv'
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.action_list = []
        self.state_list = []
        self.time = []
        self.is_running = False
        self.last_element_time = self.get_clock().now()

    def timer_callback(self):
        current_time = self.get_clock().now()
        time_difference = current_time - self.last_element_time
        if not self.is_running:
            return 
        self.get_logger().info(f"Getting data for {self.path_name}")
        if time_difference.nanoseconds > self.deadzone * 1e9:
            self.get_logger().error(f"No new element added in the last {self.deadzone} seconds. Stopping the program.")
            self.store()
            self.destroy_node()
            rclpy.shutdown()

    def action_callback(self, msg):
        time = self.get_time(msg.header.stamp)
        self.time.append(time)
        if len(self.time) > 0:
            self.is_running = True
        self.action_list.append(np.array(msg.data))
        self.last_element_time = self.get_clock().now()

    def state_callback(self, msg):
        self.state_list.append(np.array(msg.data))

    def get_time(self, stamp: rclpy.time.Time):
        # Calculate the time
        return math.fsum([stamp.sec, stamp.nanosec * math.pow(10, -9)])

    def store(self):
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
