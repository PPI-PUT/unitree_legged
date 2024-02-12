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
from rclpy.node import Node
try:
    from unitree_a1_time_process.unitree_a1_time_process import UnitreeA1TimeProcess
except ImportError:
    from unitree_a1_time_process import UnitreeA1TimeProcess
import math
from unitree_a1_legged_msgs.msg import LowCmd
from unitree_a1_legged_msgs.msg import LowState
import pandas as pd
from pathlib import Path


class UnitreeA1TimeProcessNode(Node):

    def __init__(self):
        super().__init__('unitree_a1_time_process_node')
        self.unitree_a1_time_process = UnitreeA1TimeProcess()
        self.param_name = self.declare_parameter('param_name', 456).value
        self.unitree_a1_time_process.foo(self.param_name)
        qos = rclpy.qos.QoSProfile(depth=1, reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, durability=rclpy.qos.DurabilityPolicy.VOLATILE)

        self.nn_sub = self.create_subscription(
            LowCmd, '~/nn', self.nn_callback, qos)
        self.cmd_sub = self.create_subscription(
            LowCmd, '~/gate', self.cmd_callback, qos)
        self.state_sub = self.create_subscription(
            LowState, '~/state', self.state_callback, qos)
        self.deadzone = self.declare_parameter('deadzone', 10.0).value
        rosbag_name = self.declare_parameter('rosbag_name', '').value
        rosbag_name = Path(rosbag_name)
        self.path_name = Path(rosbag_name.expanduser())
        self.nn_path_name = self.path_name / \
            f'{self.path_name.name.replace("rosbag2", "nn_time_log")}.csv'
        self.gate_path_name = self.path_name / \
            f'{self.path_name.name.replace("rosbag2", "gate_time_log")}.csv'
        self.state_path_name = self.path_name / \
            f'{self.path_name.name.replace("rosbag2", "state_time_log")}.csv'
        self.get_logger().info("Saving data")
        self.get_logger().info(f"Saving data to {self.nn_path_name}")
        self.get_logger().info(f"Saving data to {self.gate_path_name}")
        self.get_logger().info(f"Saving data to {self.state_path_name}")
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.nn_timestamp = []
        self.nn_elapsed = []
        self.cmd_timestamp = []
        self.cmd_elapsed = []
        self.state_timestamp = []
        self.is_running = False
        self.last_element_time = self.get_clock().now()

    def nn_callback(self, msg):
        self.nn_timestamp.append(self.get_time(msg.header.stamp))
        self.nn_elapsed.append(self.get_time(msg.header_elapsed.stamp))
        if len(self.nn_timestamp) > 0:
            self.is_running = True
        self.last_element_time = self.get_clock().now()

    def state_callback(self, msg):
        self.state_timestamp.append(self.get_time(msg.header.stamp))

    def cmd_callback(self, msg):
        self.cmd_timestamp.append(self.get_time(msg.header.stamp))
        self.cmd_elapsed.append(self.get_time(msg.header_elapsed.stamp))

    def get_time(self, stamp: rclpy.time.Time):
        # Calculate the time
        return math.fsum([stamp.sec, stamp.nanosec * math.pow(10, -9)])

    def timer_callback(self):
        current_time = self.get_clock().now()
        time_difference = current_time - self.last_element_time
        if not self.is_running:
            return
        self.get_logger().info(f"Getting data for {self.path_name}")
        if time_difference.nanoseconds > self.deadzone * 1e9:
            self.get_logger().error(
                f"No new element added in the last {self.deadzone} seconds. Stopping the program.")
            self.cleanup()
            rclpy.shutdown()

    def cleanup(self):
        # On destruction, save the data to a csv file
        df = pd.DataFrame()
        df['timestamp'] = self.nn_timestamp
        df['elapsed'] = self.nn_elapsed
        df.to_csv(self.nn_path_name)
        df = pd.DataFrame()
        df['timestamp'] = self.cmd_timestamp
        df['elapsed'] = self.cmd_elapsed
        df.to_csv(self.gate_path_name)
        df = pd.DataFrame()
        df['timestamp'] = self.state_timestamp
        df.to_csv(self.state_path_name)


def main(args=None):
    rclpy.init(args=args)
    node = UnitreeA1TimeProcessNode()
    try:
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        rclpy.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
