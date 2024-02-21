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
        f'{bag_file_path.name.replace("rosbag2", "log")}.csv'
    tensor_topic = '/unitree_a1_neural_control/debug/tensor'
    action_topic = '/unitree_a1_neural_control/debug/action'

    # Get the message type
    debug_type = get_message("unitree_a1_legged_msgs/msg/DebugDoubleArray")

    # Initialize DataFrame and lists to store data
    tensor_time_list, tensor_list = [], []
    action_time_list, action_list = [], []

    # Read messages from the bag file
    with Reader(bag_file_path) as reader:
        for connection, timestamp, rawdata in reader.messages():
            if connection.topic == tensor_topic:
                deserialized_msg = deserialize_message(rawdata, debug_type)
                header = deserialized_msg.header
                tensor_list.append(np.array(deserialized_msg.data))
                tensor_time_list.append(
                    header.stamp.sec + header.stamp.nanosec * 1e-9)

            elif connection.topic == action_topic:
                deserialized_msg = deserialize_message(rawdata, debug_type)
                header = deserialized_msg.header
                action_list.append(np.array(deserialized_msg.data))
                action_time_list.append(
                    header.stamp.sec + header.stamp.nanosec * 1e-9)

    # Equalize the lengths of the lists
    max_len = max(len(tensor_list), len(action_list))
    tensor_time_list += [None] * (max_len - len(tensor_time_list))
    tensor_list += [None] * (max_len - len(tensor_list))
    action_time_list += [None] * (max_len - len(action_time_list))
    action_list += [None] * (max_len - len(action_list))

    # Populate the DataFrame
    df_states = pd.DataFrame({
        'state_time': tensor_time_list,
        'state': tensor_list
    })

    df_actions = pd.DataFrame({
        'action_time': action_time_list,
        'action': action_list
    })
    df_combined = pd.merge(df_states, df_actions, how='inner',
                           left_on='state_time', right_on='action_time')
    df_combined['time'] = df_combined['state_time'].combine_first(
        df_combined['action_time'])
    df_combined.drop(['state_time', 'action_time'], axis=1, inplace=True)

    # Save to CSV
    df_combined.to_csv(log_name)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Process a ROS2 bag file.")
    parser.add_argument('bag_file_path', type=str,
                        help="Path to the ROS2 bag file")

    args = parser.parse_args()

    process_rosbag(args.bag_file_path)
