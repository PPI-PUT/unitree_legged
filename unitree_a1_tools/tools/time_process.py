import argparse  # Import argparse for command line arguments parsing
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from rosbags.rosbag2 import Reader
import pandas as pd
from pathlib import Path
import numpy as np


def process_rosbag(bag_file_path):
    # Convert the string path to a Path object
    bag_file_path = Path(bag_file_path)
    # Create log file name from bag file name
    log_name = Path(bag_file_path.expanduser()) / \
        f'{bag_file_path.name.replace("rosbag2", "time")}.csv'
    topics = ['/unitree_a1_legged/controller/nn/cmd', "/unitree_a1_legged/cmd"]
    # Get the message type
    type = get_message("unitree_a1_legged_msgs/msg/LowCmd")

    # Initialize DataFrame and lists to store data
    nn_time = []
    cm_time = []

    # Read messages from the bag file
    with Reader(bag_file_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == topics[0]:
                deserialized_msg = deserialize_message(rawdata, type)
                header = deserialized_msg.header
                nn_time.append(
                    header.stamp.sec + header.stamp.nanosec * 1e-9)

            elif connection.topic == topics[1]:
                deserialized_msg = deserialize_message(rawdata, type)
                header = deserialized_msg.header
                cm_time.append(
                    header.stamp.sec + header.stamp.nanosec * 1e-9)

    # Equalize the lengths of the lists
    # max_len = max(len(nn_time), len(cm_time))
    # nn_time += [None] * (max_len - len(nn_time))
    # cm_time += [None] * (max_len - len(cm_time))

    # Populate the DataFrame
    df_combined = pd.DataFrame({
        'cmd_time': cm_time,
        'nn_time': nn_time,
    })

    # Save to CSV
    df_combined.to_csv(log_name)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Process a ROS2 bag file.")
    parser.add_argument('bag_file_path', type=str,
                        help="Path to the ROS2 bag file")

    args = parser.parse_args()

    process_rosbag(args.bag_file_path)
