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
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# def contact_remap(remap: LaunchConfiguration, context):
#     remap_str = remap.perform(context)
#     return [(f'~/output/{var}_contact', f'{remap_str}_{var}')
#             for var in
#             ['fr', 'fl', 'rr', 'rl']]


def launch_setup(context, *args, **kwargs):
    param_path = LaunchConfiguration(
        'unitree_a1_legged_param_file').perform(context)
    if not param_path:
        param_path = PathJoinSubstitution(
            [FindPackageShare('unitree_a1_legged'), 'config',
             'unitree_a1_legged.param.yaml']
        ).perform(context)

    unitree_a1_legged_node = Node(
        package='unitree_a1_legged',
        executable='unitree_a1_legged_node_exe',
        name='unitree_a1_legged_node',
        parameters=[
            param_path
        ],
        remappings=[
            ("~/output/state", LaunchConfiguration("output_state_name")),
            ("~/output/joy", LaunchConfiguration("output_joy_name")),
            ("~/input/command", LaunchConfiguration("input_command_name")),
            ("~/output/joint_states", LaunchConfiguration("output_joint_state_name")),
            ("~/output/imu", LaunchConfiguration("output_imu_name")),
        ],
        output='screen',
        arguments=['--ros-args', '--log-level',
                   'info', '--enable-stdout-logs'],
    )

    return [
        unitree_a1_legged_node
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg('unitree_a1_legged_param_file', '')
    add_launch_arg('output_state_name', 'unitree_a1_legged/state')
    add_launch_arg('input_command_name', 'unitree_a1_legged/cmd')
    add_launch_arg('output_joy_name', 'unitree_a1_legged/joy')
    add_launch_arg('output_joint_state_name', 'unitree_a1_legged/joint_states')
    add_launch_arg('output_imu_name', 'unitree_a1_legged/imu')
    # add_launch_arg('output_contact_name', 'unitree_a1_legged/contact')

    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
