# Copyright 2023 Maciej Krupka
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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def action_remap(name: str, remap: LaunchConfiguration, context):
    return [
        (f'{name}/_action/feedback',
         f'{remap.perform(context)}/_action/feedback'),
        (f'{name}/_action/status',
         f'{remap.perform(context)}/_action/status'),
        (f'{name}/_action/cancel_goal',
         f'{remap.perform(context)}/_action/cancel_goal'),
        (f'{name}/_action/get_result',
         f'{remap.perform(context)}/_action/get_result'),
        (f'{name}/_action/send_goal',
         f'{remap.perform(context)}/_action/send_goal'),
    ]


def launch_setup(context, *args, **kwargs):
    pkg_prefix = FindPackageShare('unitree_a1_legged_launch')
    unitree_legged_file = PathJoinSubstitution([
        pkg_prefix,
        "config",
        "unitree_a1_legged.param.yaml"
    ])
    unitree_legged_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare(
                    'unitree_a1_legged'), 'launch', 'unitree_a1_legged.launch.py'
            ]),
        ),
        launch_arguments={
            "unitree_a1_legged_param_file": unitree_legged_file,
            "output_state_name": "unitree_a1_legged/state",
            "input_command_name": "unitree_a1_legged/cmd",
            "output_joy_name": "unitree_a1_legged/joy",
            "output_joint_state_name": "unitree_a1_legged/joint_states",
            "output_imu_name": "unitree_a1_legged/imu",
        }.items()
    )
    unitree_legged_joystick_file = PathJoinSubstitution([
        pkg_prefix,
        "config",
        "unitree_a1_joystick.param.yaml"
    ])
    unitree_joystick_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare(
                    'unitree_a1_joystick'), 'launch', 'unitree_a1_joystick.launch.py'
            ]),
        ),
        launch_arguments={
            "unitree_a1_joystick_param_file": unitree_legged_joystick_file,
            "output_cmd_name": "unitree_a1_legged/cmd_vel",
            "input_joy_name": "unitree_a1_legged/joy",
            "service_gait_name": "unitree_a1_legged/gait"

        }.items()
    )
    state_machine = PathJoinSubstitution([
        pkg_prefix,
        "config",
        "unitree_a1_state_machine.param.yaml"
    ])
    state_machine_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare(
                    'unitree_a1_state_machine'), 'launch', 'unitree_a1_state_machine.launch.py'
            ]),
        ),
        launch_arguments={
            "unitree_a1_state_machine_param_file": state_machine,
            "input_walk_name": "/unitree_a1_legged/nn/cmd",
            "input_stand_name": "/unitree_a1_legged/fixed_stand/cmd",
            "output_command_name": "/unitree_a1_legged/cmd",
            "service_gait_name": "/unitree_a1_legged/gait",
            "service_stand_name": "/unitree_a1_legged/stand",
            "service_reset_name": "/unitree_a1_legged/nn/reset_controller",
        }.items()
    )
    stand_action = PathJoinSubstitution([
        pkg_prefix,
        "config",
        "unitree_a1_fixed_stand_server.param.yaml"
    ])
    stand_action_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare(
                    'unitree_a1_fixed_stand_server'), 'launch', 'unitree_a1_fixed_stand_server.launch.py'
            ]),
        ),
        launch_arguments={
            "unitree_a1_state_machine_param_file": stand_action,
            "output_cmd_name": "/unitree_a1_legged/fixed_stand/cmd",
            "input_state_name": "/unitree_a1_legged/state",
            "service_stand_name": "/unitree_a1_legged/stand",

        }.items()
    )
    neural_control = PathJoinSubstitution([
        pkg_prefix,
        "config",
        "unitree_a1_neural_control.param.yaml"
    ])
    neural_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            launch_file_path=PathJoinSubstitution([
                FindPackageShare(
                    'unitree_a1_neural_control'), 'launch', 'unitree_a1_neural_control.launch.py'
            ]),
        ),
        launch_arguments={
            "unitree_a1_neural_control_param_file": neural_control,
            "input_state_name": "/unitree_a1_legged/state",
            "output_cmd_name": "/unitree_a1_legged/nn/cmd",
            "input_cmd_vel_name": "/unitree_a1_legged/cmd_vel",
            "service_reset_name": "/unitree_a1_legged/nn/reset_controller",

        }.items()
    )
    return [
        unitree_legged_launch,
        unitree_joystick_launch,
        state_machine_launch,
        stand_action_launch,
        neural_control_launch
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg('with_rviz', 'False')

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
