import subprocess
import argparse
from pathlib import Path


def run_rosbag2log(rosbag_dir):
    subprocess.run(['python3', 'rosbag2log.py', rosbag_dir], check=True)


def run_pair_grid(real_log, sim_data_file, time_limit):
    subprocess.run(['python3', 'pair_grid.py', '--real_log', real_log,
                   '--sim_data', sim_data_file, '--time_limit', str(time_limit)], check=True)


def main(rosbag_dir, sim_data_file, time_limit):
    # Convert the directory path to a Path object for easier manipulation
    bag_dir = Path(rosbag_dir)
    log_file_path = Path(bag_dir.expanduser()) / \
        f'{bag_dir.name.replace("rosbag2", "log")}.csv'
    log_file_name = log_file_path.name

    # Check if the log file already exists
    if not log_file_path.exists():
        print(
            f"Log file {log_file_name} does not exist. Creating from ROS bag...")
        run_rosbag2log(rosbag_dir)
    else:
        print(
            f"Log file {log_file_name} already exists. Skipping ROS bag processing.")

    # Run pair_grid script with the generated or existing log file
    print("Running pair grid visualization...")
    run_pair_grid(str(log_file_path), sim_data_file, time_limit)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Process ROS bag to log and generate pair grid plots.")
    parser.add_argument("rosbag_dir", type=str,
                        help="Directory containing the ROS bag file")
    parser.add_argument("--sim_data", type=str, required=True,
                        help="Path to the simulation data file")
    parser.add_argument("--time_limit", type=float, default=0.3,
                        help="Time limit for processing log data")

    args = parser.parse_args()

    main(args.rosbag_dir, args.sim_data, args.time_limit)
