# Copyright 2023 Maciej Krupka
# Perception for Physical Interaction Laboratory at Poznan University of Technology
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
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    param_path = LaunchConfiguration('unitree_a1_joystick_param_file').perform(context)
    if not param_path:
        param_path = PathJoinSubstitution(
            [FindPackageShare('unitree_a1_joystick'), 'config',
             'unitree_a1_joystick.param.yaml']
        ).perform(context)

    unitree_a1_joystick = Node(
        package='unitree_a1_joystick',
        executable='unitree_a1_joystick_node_exe',
        name='unitree_a1_joystick_node',
        parameters=[
            param_path
        ],
        remappings=[
            ("~/output/cmd_vel", LaunchConfiguration("output_cmd_name")),
            ("~/input/joy", LaunchConfiguration("input_joy_name")),
            ("~/service/gait", LaunchConfiguration("service_gait_name"))
        ],
        output='screen',
        arguments=['--ros-args', '--log-level',
                   'info', '--enable-stdout-logs'],
    )

    return [
        unitree_a1_joystick
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg('unitree_a1_joystick_param_file', '')
    add_launch_arg("output_cmd_name", "unitree_a1_joystick/cmd_vel")
    add_launch_arg("input_joy_name", "unitree_a1_joystick/joy")
    add_launch_arg("service_gait_name", "unitree_a1_gait/gait")
    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
