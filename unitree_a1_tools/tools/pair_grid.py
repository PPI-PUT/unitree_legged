import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import argparse
from pathlib import Path


def string_to_array(array_string):
    array_string = array_string.replace(
        '\n', '').replace('[', '').replace(']', '')
    result_array = np.fromstring(array_string, sep=' ')
    return result_array


def load_columns(x):
    columns = [
        "fr_hip_pos", "fr_thigh_pos", "fr_calf_pos",
        "fl_hip_pos", "fl_thigh_pos", "fl_calf_pos",
        "rr_hip_pos", "rr_thigh_pos", "rr_calf_pos",
        "rl_hip_pos", "rl_thigh_pos", "rl_calf_pos",
        "roll_vel", "pitch_vel", "yaw_vel",
        "fr_hip_vel", "fr_thigh_vel", "fr_calf_vel",
        "fl_hip_vel", "fl_thigh_vel", "fl_calf_vel",
        "rr_hip_vel", "rr_thigh_vel", "rr_calf_vel",
        "rl_hip_vel", "rl_thigh_vel", "rl_calf_vel",
        "goal_x_vel", "goal_y_vel", "goal_z_vel",
        "fl_contact", "fr_contact", "rl_contact", "rr_contact",
        "gravity_vec_1", "gravity_vec_2", "gravity_vec_3",
        "last_action_0", "last_action_1", "last_action_2", "last_action_3",
        "last_action_4", "last_action_5", "last_action_6", "last_action_7",
        "last_action_8", "last_action_9", "last_action_10", "last_action_11",
        "fr_cycles_since_last_contact", "fl_cycles_since_last_contact",
        "rr_cycles_since_last_contact", "rl_cycles_since_last_contact",
    ]
    return pd.DataFrame(x, columns=columns)


def load_log(log_file, dt=np.inf):
    df = pd.read_csv(log_file)
    df = df.sort_values(by=['time'])
    df = df[df['time'] - df['time'][0] < dt]
    df['action'] = df['action'].apply(string_to_array)
    df['state'] = df['state'].apply(string_to_array)
    state_matrix = np.vstack(df["state"].values)
    df = load_columns(state_matrix)
    df['source'] = 'Real'
    return df


def load_sim_data(log_file):
    x = np.load(log_file)
    df = load_columns(x)
    df['source'] = 'Simulator'
    return df


def part_pairgrid(data, vars, save_dir, name="xx"):
    g = sns.PairGrid(data, vars=vars, hue='source', palette=['red', 'blue'])
    g.map_diag(sns.histplot, common_norm=False)
    g.map_offdiag(sns.scatterplot)  # For the off-diagonal scatter plots
    g.add_legend()
    plt.savefig(Path(save_dir) / f"{name}_pairgrid.png")


def main(real_log_file, sim_data_file, time_limit):
    save_dir = Path(real_log_file).parent / "figures"
    save_dir.mkdir(parents=True, exist_ok=True)
    dfs = [load_log(real_log_file, dt=time_limit)]
    sim = load_sim_data(sim_data_file)
    sim = sim[:len(dfs[0])]
    dfs.append(sim)

    combined_df = pd.concat(dfs)

    # Define columns to plot
    column_groups = {
        "fr": ["fr_hip_pos", "fr_thigh_pos", "fr_calf_pos", "fr_hip_vel", "fr_thigh_vel", "fr_calf_vel"],
        "fl": ["fl_hip_pos", "fl_thigh_pos", "fl_calf_pos", "fl_hip_vel", "fl_thigh_vel", "fl_calf_vel"],
        "rr": ["rr_hip_pos", "rr_thigh_pos", "rr_calf_pos", "rr_hip_vel", "rr_thigh_vel", "rr_calf_vel"],
        "rl": ["rl_hip_pos", "rl_thigh_pos", "rl_calf_pos", "rl_hip_vel", "rl_thigh_vel", "rl_calf_vel"],
        "imu": ["roll_vel", "pitch_vel", "yaw_vel", "gravity_vec_1", "gravity_vec_2", "gravity_vec_3"],
        "contact": ["fl_contact", "fr_contact", "rl_contact", "rr_contact", "fr_cycles_since_last_contact", "fl_cycles_since_last_contact", "rr_cycles_since_last_contact", "rl_cycles_since_last_contact"],
    }

    for name, cols in column_groups.items():
        print(name)
        part_pairgrid(combined_df, cols, save_dir, name)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Load and visualize log and simulation data.")
    parser.add_argument("--real_log", type=str,
                        help="Path to the real log file")
    parser.add_argument("--sim_data", type=str,
                        help="Path to the simulator data file")
    parser.add_argument("--time_limit", type=float,
                        default=np.inf, help="Time limit for logs processing")

    args = parser.parse_args()

    main(args.real_log, args.sim_data, args.time_limit)
