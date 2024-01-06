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

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    param_path = LaunchConfiguration('unitree_a1_data_parser_param_file').perform(context)
    if not param_path:
        param_path = PathJoinSubstitution(
            [FindPackageShare('unitree_a1_data_parser'), 'config', 'unitree_a1_data_parser.param.yaml']
        ).perform(context)

    unitree_a1_data_parser_node = Node(
        package='unitree_a1_data_parser',
        executable='unitree_a1_data_parser_node.py',
        name='unitree_a1_data_parser_node',
        parameters=[
            param_path
        ],
        remappings=[
            ("~/action", LaunchConfiguration("action_name")),
            ("~/state", LaunchConfiguration("state_name")),
        ],
        output='screen',
        arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs'],
        emulate_tty=True
    )

    return [
        unitree_a1_data_parser_node
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg('unitree_a1_data_parser_param_file', '')
    add_launch_arg('action_name', '/unitree_a1_neural_control_node/debug/action')
    add_launch_arg('state_name', '/unitree_a1_neural_control_node/debug/tensor')

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
