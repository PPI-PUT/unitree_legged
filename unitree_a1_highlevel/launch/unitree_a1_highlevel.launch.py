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


def action_remap(name: str, remap: LaunchConfiguration, context):
    remap_str = remap.perform(context)
    return [(f'{name}/_action/{var}', f'{remap_str}/_action/{var}')
            for var in
            ['feedback', 'status', 'cancel_goal', 'get_result', 'send_goal']]


def launch_setup(context, *args, **kwargs):
    param_path = LaunchConfiguration(
        'unitree_a1_highlevel_param_file').perform(context)
    if not param_path:
        param_path = PathJoinSubstitution(
            [FindPackageShare('unitree_a1_highlevel'),
             'config', 'unitree_a1_highlevel.param.yaml']
        ).perform(context)
    unitree_a1_state_machine_node = Node(
        package='unitree_a1_highlevel',
        executable='unitree_a1_highlevel_node_exe',
        name='unitree_a1_highlevel_node',
        parameters=[
            param_path
        ],
        remappings=[
            ("~/output/twist", LaunchConfiguration("output_twist_name")),
            ("~/output/cmd", LaunchConfiguration("output_command_name")),
            ("~/service/gait", LaunchConfiguration("service_gait_name")),
        ] + action_remap('~/action/fixed_stand',
                         LaunchConfiguration("action_stand_name"), context) +
        action_remap('~/action/hold_position',
                     LaunchConfiguration("action_hold_position"), context),
        output='screen',
        arguments=['--ros-args', '--log-level',
                   'info', '--enable-stdout-logs'],
    )

    return [
        unitree_a1_state_machine_node
    ]


def generate_launch_description():
    declared_arguments = []

    def add_launch_arg(name: str, default_value: str = None):
        declared_arguments.append(
            DeclareLaunchArgument(name, default_value=default_value)
        )

    add_launch_arg('unitree_a1_highlevel_param_file', '')
    add_launch_arg('output_twist_name',
                   '/unitree_a1_legged/controllers/cmd_vel')
    add_launch_arg('service_gait_name', '/unitree_a1_legged/gait')
    add_launch_arg('action_stand_name', '/unitree_a1_legged/stand')
    add_launch_arg('action_hold_position', '/unitree_a1_legged/hold_position')
    return LaunchDescription([
        *declared_arguments,
        OpaqueFunction(function=launch_setup)
    ])
