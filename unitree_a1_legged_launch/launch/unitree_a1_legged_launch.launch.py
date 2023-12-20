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


def launch_setup(context, *args, **kwargs):
    rviz_cfg_path = PathJoinSubstitution(
        [FindPackageShare('unitree_a1_legged_launch'), 'rviz/default.rviz'])
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
            "output_joy_name": "unitree_a1_legged/joy",
            "input_command_name": "unitree_a1_legged/cmd"
        }.items()
    )
    unitree_legged_joystick_file = PathJoinSubstitution([
        pkg_prefix,
        "config",
        "unitree_a1_legged.param.yaml"
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
            "input_joy_name": "unitree_a1_legged/joy"

        }.items()
    )

    rviz2 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(rviz_cfg_path.perform(context))],
        condition=IfCondition(LaunchConfiguration('with_rviz'))
    )

    return [
        rviz2,
        unitree_legged_launch,
        unitree_joystick_launch
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
