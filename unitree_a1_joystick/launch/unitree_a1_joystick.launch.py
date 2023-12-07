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

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    unitree_a1_joystick_launch_pkg_prefix = get_package_share_directory(
        "unitree_a1_joystick")

    unitree_a1_joystick_config_param = DeclareLaunchArgument(
        'unitree_a1_joystick_config_param_file',
        default_value=[unitree_a1_joystick_launch_pkg_prefix,
                       '/config/defaults.param.yaml'],
        description='Node config.'
    )

    unitree_a1_joystick_node = Node(
        package='unitree_a1_joystick',
        executable='unitree_a1_joystick_node_exe',
        name='unitree_a1_joystick',
        output='screen',
        parameters=[
            LaunchConfiguration('unitree_a1_joystick_config_param_file')
        ],
        arguments=['--ros-args', '--log-level', 'info', '--enable-stdout-logs']
    )

    ld = LaunchDescription([
        unitree_a1_joystick_config_param,
        unitree_a1_joystick_node,
    ])
    return ld
